#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

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

// -------------------- IMU --------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// -------------------- Encoders --------------------
const int encA_left = 5;
const int encB_left = 17;
const int encA_right = 18;
const int encB_right = 19;

volatile long count_left = 0;
volatile long count_right = 0;

void IRAM_ATTR encLeftISR() {
    int b = digitalRead(encB_left);
    if (b == HIGH) count_left++; else count_left--;
}

void IRAM_ATTR encRightISR() {
    int b = digitalRead(encB_right);
    if (b == HIGH) count_right++; else count_right--;
}

// -------------------- micro-ROS --------------------
rcl_publisher_t imu_pub;
rcl_publisher_t enc_pub;

std_msgs__msg__Float32MultiArray imu_msg;
std_msgs__msg__Float32MultiArray enc_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;

// -------------------- Function Prototypes --------------------
void rclErrorLoop();
void syncTime();
struct timespec getTime();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);

// -------------------- Setup --------------------
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // IMU
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055!");
        while (1);
    }
    bno.setExtCrystalUse(true);

    // Encoders
    pinMode(encA_left, INPUT_PULLUP);
    pinMode(encB_left, INPUT_PULLUP);
    pinMode(encA_right, INPUT_PULLUP);
    pinMode(encB_right, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encA_left), encLeftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA_right), encRightISR, CHANGE);

    // micro-ROS setup
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 96);

    RCCHECK(rclc_node_init_default(&node, "white_slot_sensor_node", "", &support));

    // Publishers
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/white_slot/imu"));

    RCCHECK(rclc_publisher_init_best_effort(
        &enc_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/white_slot/encoders"));

    // Allocate message arrays
    imu_msg.data.data = (float *)malloc(4 * sizeof(float));
    imu_msg.data.size = 4;
    imu_msg.data.capacity = 4;

    enc_msg.data.data = (float *)malloc(2 * sizeof(float));
    enc_msg.data.size = 2;
    enc_msg.data.capacity = 2;

    // Timer & executor
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(50),  // 20 Hz
        controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();
}

// -------------------- Loop --------------------
void loop() {
    EXECUTE_EVERY_N_MS(10, rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}

// -------------------- Timer Callback --------------------
void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // IMU
        imu::Quaternion q = bno.getQuat();
        imu_msg.data.data[0] = q.w();
        imu_msg.data.data[1] = q.x();
        imu_msg.data.data[2] = q.y();
        imu_msg.data.data[3] = q.z();
        rcl_publish(&imu_pub, &imu_msg, NULL);

        // Encoders
        enc_msg.data.data[0] = (float)count_left;
        enc_msg.data.data[1] = (float)count_right;
        rcl_publish(&enc_pub, &enc_msg, NULL);
    }
}

// -------------------- Time Sync --------------------
void syncTime() {
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime() {
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// -------------------- Error --------------------
void rclErrorLoop() {
    const int LED_PIN = 13;
    pinMode(LED_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}
