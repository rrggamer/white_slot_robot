#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <cmath>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

// -------------------- MACROS --------------------
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }

#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
            init = uxr_millis();           \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

// -------------------- MOTOR PINS --------------------
#define AIN1 33
#define AIN2 25
#define BIN1 26
#define BIN2 27
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_AIN1 0
#define PWM_CHANNEL_AIN2 1
#define PWM_CHANNEL_BIN1 2
#define PWM_CHANNEL_BIN2 3

// -------------------- IMU --------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// -------------------- ENCODERS --------------------
const int encA_left = 34;
const int encB_left = 35;
const int encA_right = 32;
const int encB_right = 33;

volatile long count_left = 0;
volatile long count_right = 0;

void IRAM_ATTR encLeftISR() {
    int b = digitalRead(encB_left);
    count_left += (b == HIGH) ? 1 : -1;
}

void IRAM_ATTR encRightISR() {
    int b = digitalRead(encB_right);
    count_right += (b == HIGH) ? 1 : -1;
}

// -------------------- micro-ROS --------------------
rcl_publisher_t debug_motor_publisher;
rcl_publisher_t imu_pub;
rcl_publisher_t enc_pub;

geometry_msgs__msg__Twist debug_motor_msg;
geometry_msgs__msg__Twist motor_msg;

std_msgs__msg__Float32MultiArray imu_msg;
std_msgs__msg__Float32MultiArray enc_msg;

rcl_subscription_t motor_subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

// -------------------- STATE MACHINE --------------------
enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

// -------------------- FUNCTION PROTOTYPES --------------------
void rclErrorLoop();
void syncTime();
struct timespec getTime();
bool createEntities();
bool destroyEntities();
void Move();
void publishData();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void twistCallback(const void *msgin);

// -------------------- SETUP --------------------
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // Motors
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    ledcSetup(PWM_CHANNEL_AIN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_AIN2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_BIN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_BIN2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(AIN1, PWM_CHANNEL_AIN1);
    ledcAttachPin(AIN2, PWM_CHANNEL_AIN2);
    ledcAttachPin(BIN1, PWM_CHANNEL_BIN1);
    ledcAttachPin(BIN2, PWM_CHANNEL_BIN2);

    // IMU
    if (!bno.begin()) { while (1); }
    bno.setExtCrystalUse(true);

    // Encoders
    pinMode(encA_left, INPUT_PULLUP); pinMode(encB_left, INPUT_PULLUP);
    pinMode(encA_right, INPUT_PULLUP); pinMode(encB_right, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encA_left), encLeftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_right), encRightISR, CHANGE);
}

// -------------------- LOOP --------------------
void loop() {
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) destroyEntities();
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            break;
        case AGENT_DISCONNECTED:
            destroyEntities();
            state = WAITING_AGENT;
            break;
    }
}

// -------------------- CONTROL CALLBACK --------------------
void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        Move();

        // Publish motor debug
        rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL);

        // Publish IMU quaternion
        imu::Quaternion q = bno.getQuat();
        imu_msg.data.data[0] = q.w();
        imu_msg.data.data[1] = q.x();
        imu_msg.data.data[2] = q.y();
        imu_msg.data.data[3] = q.z();
        rcl_publish(&imu_pub, &imu_msg, NULL);

        // Publish encoder counts
        enc_msg.data.data[0] = (float)count_left;
        enc_msg.data.data[1] = (float)count_right;
        rcl_publish(&enc_pub, &enc_msg, NULL);
    }
}

// -------------------- TWIST CALLBACK --------------------
void twistCallback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    motor_msg = *msg;
    prev_cmd_time = millis();
}

// -------------------- CREATE / DESTROY ENTITIES --------------------
bool createEntities() {
    allocator = rcl_get_default_allocator();
    geometry_msgs__msg__Twist__init(&debug_motor_msg);
    imu_msg.data.data = (float *)malloc(4 * sizeof(float));
    imu_msg.data.size = 4;
    imu_msg.data.capacity = 4;
    enc_msg.data.data = (float *)malloc(2 * sizeof(float));
    enc_msg.data.size = 2;
    enc_msg.data.capacity = 2;

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 96);
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    RCCHECK(rclc_node_init_default(&node, "white_slot_robot_node", "", &support));

    // Publishers
    RCCHECK(rclc_publisher_init_best_effort(&debug_motor_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/white_slot/debug/cmd_move/rpm"));
    RCCHECK(rclc_publisher_init_best_effort(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/white_slot/imu"));
    RCCHECK(rclc_publisher_init_best_effort(&enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/white_slot/encoders"));

    // Subscriber
    RCCHECK(rclc_subscription_init_default(&motor_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/white_slot/cmd_move/rpm"));

    // Timer & executor
    RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(50), controlCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();
    return true;
}

bool destroyEntities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_motor_publisher, &node);

}
