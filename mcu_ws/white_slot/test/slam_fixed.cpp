#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <Adafruit_BNO055.h>
#include <ESP32Encoder.h>

// ---------------------------- Hardware ----------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

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

// ---------------------------- ROS2 ----------------------------
rcl_publisher_t encoder_publisher;
geometry_msgs__msg__Twist encoder_msg;

rcl_publisher_t imu_publisher;
geometry_msgs__msg__Twist imu_msg;

rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist motor_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

// ---------------------------- States ----------------------------
enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state = WAITING_AGENT;

// ---------------------------- Helpers ----------------------------
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if (temp_rc != RCL_RET_OK)   \
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

void rclErrorLoop()
{
    const int LED_PIN = 13;
    pinMode(LED_PIN, OUTPUT);
    while (1)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

// ---------------------------- Function Prototypes ----------------------------
bool createEntities();
bool destroyEntities();
void publishData();
void Move();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void twistCallback(const void *msgin);
void syncTime();

// ---------------------------- Setup ----------------------------
void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // Motors
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    ledcSetup(PWM_CHANNEL_AIN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_AIN2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_BIN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_BIN2, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(AIN1, PWM_CHANNEL_AIN1);
    ledcAttachPin(AIN2, PWM_CHANNEL_AIN2);
    ledcAttachPin(BIN1, PWM_CHANNEL_BIN1);
    ledcAttachPin(BIN2, PWM_CHANNEL_BIN2);

    // Encoders
    encoderLeft.attachFullQuad(5, 17);
    encoderLeft.clearCount();
    encoderRight.attachFullQuad(18, 19);
    encoderRight.clearCount();

    // BNO055
    Wire.begin(21, 22);
    if (!bno.begin())
    {
        Serial.println("Failed to initialize BNO055!");
        while (1)
            ;
    }
    bno.setExtCrystalUse(true);
}

// ---------------------------- Loop ----------------------------
void loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        Serial.println("Waiting for micro-ROS agent...");
        EXECUTE_EVERY_N_MS(500,
                           state = (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;

    case AGENT_AVAILABLE:
        Serial.println("Agent found. Creating ROS entities...");
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200,
                           state = (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        }
        break;

    case AGENT_DISCONNECTED:
        Serial.println("Agent disconnected! Cleaning up...");
        destroyEntities();
        state = WAITING_AGENT;
        break;
    }
}

// ---------------------------- ROS Callbacks ----------------------------
void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        Move();
        publishData();
    }
}

void twistCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    motor_msg = *msg;
    prev_cmd_time = millis();
}

// ---------------------------- Create / Destroy Entities ----------------------------
bool createEntities()
{
    allocator = rcl_get_default_allocator();

    geometry_msgs__msg__Twist__init(&encoder_msg);
    geometry_msgs__msg__Twist__init(&imu_msg);

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 96);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    RCCHECK(rclc_node_init_default(&node, "white_slot_shooter_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(&encoder_publisher,
                                           &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                           "/white_slot/encoder"));

    RCCHECK(rclc_publisher_init_best_effort(&imu_publisher,
                                           &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                           "/white_slot/imu"));

    RCCHECK(rclc_subscription_init_default(&motor_subscriber,
                                          &node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                          "/white_slot/cmd_move/rpm"));

    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(&control_timer,
                                    &support,
                                    RCL_MS_TO_NS(control_timeout),
                                    controlCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &twistCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();
    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&encoder_publisher, &node);
    rcl_subscription_fini(&motor_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

// ---------------------------- Motors ----------------------------
void Move()
{
    float motor1Speed = motor_msg.linear.x;
    float motor2Speed = motor_msg.linear.y;
    float max_rpm = 150.0;

    uint8_t duty1 = (uint8_t)((fabs(motor1Speed) / max_rpm) * 255.0);
    uint8_t duty2 = (uint8_t)((fabs(motor2Speed) / max_rpm) * 255.0);

    // Motor A
    if (motor1Speed > 0)
    {
        ledcWrite(PWM_CHANNEL_AIN1, duty1);
        ledcWrite(PWM_CHANNEL_AIN2, 0);
    }
    else if (motor1Speed < 0)
    {
        ledcWrite(PWM_CHANNEL_AIN1, 0);
        ledcWrite(PWM_CHANNEL_AIN2, duty1);
    }
    else
    {
        ledcWrite(PWM_CHANNEL_AIN1, 0);
        ledcWrite(PWM_CHANNEL_AIN2, 0);
    }

    // Motor B
    if (motor2Speed > 0)
    {
        ledcWrite(PWM_CHANNEL_BIN1, duty2);
        ledcWrite(PWM_CHANNEL_BIN2, 0);
    }
    else if (motor2Speed < 0)
    {
        ledcWrite(PWM_CHANNEL_BIN1, 0);
        ledcWrite(PWM_CHANNEL_BIN2, duty2);
    }
    else
    {
        ledcWrite(PWM_CHANNEL_BIN1, 0);
        ledcWrite(PWM_CHANNEL_BIN2, 0);
    }
}

// ---------------------------- Publish ----------------------------
void publishData()
{
    encoder_msg.linear.x = encoderLeft.getCount();
    encoder_msg.linear.y = encoderRight.getCount();
    rcl_publish(&encoder_publisher, &encoder_msg, NULL);

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    imu_msg.angular.z = orientationData.orientation.x;
    imu_msg.angular.x = orientationData.orientation.z;
    imu_msg.angular.y = orientationData.orientation.y;
    rcl_publish(&imu_publisher, &imu_msg, NULL);
}

// ---------------------------- Time ----------------------------
void syncTime()
{
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}
