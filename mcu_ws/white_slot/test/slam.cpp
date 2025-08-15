#include <Arduino.h>
#include <Wire.h>
#include <cmath>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include <Adafruit_BNO055.h>
#include <ESP32Encoder.h>



Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;


#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)


#define AIN1 33
#define AIN2 25
#define BIN1 26
#define BIN2 27


#define PWM_FREQ 5000        // 5 kHz PWM frequency
#define PWM_RESOLUTION 8     // 8-bit resolution
#define PWM_CHANNEL_AIN1 0
#define PWM_CHANNEL_AIN2 1
#define PWM_CHANNEL_BIN1 2
#define PWM_CHANNEL_BIN2 3

// const int encA_left = 5;
// const int encB_left = 17;
// const int encA_right = 18;
// const int encB_right = 19;

int LeftEncoderCount;
int RightEncoderCount;




//------------------------------ < Define > -------------------------------------//

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
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;





//------------------------------ < Fuction Prototype > ------------------------------//

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();
void fullStop();

void Move();
//------------------------------ < Main > -------------------------------------//



void setup()
{   

    
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  Wire.begin(21,22);

  encoderLeft.attachFullQuad(5,17);
  encoderLeft.clearCount();

  encoderRight.attachFullQuad(18,19);
  encoderRight.clearCount();

// if (!bno.begin()) {
//     Serial.println("Failed to initialize BNO055!");
//     while (1);
// }
bno.setExtCrystalUse(true);

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

}

void loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
    case AGENT_DISCONNECTED:
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//


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
    prev_cmd_time = millis();
}




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

    RCCHECK(rclc_publisher_init_best_effort(
        &encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/white_slot/encoder"));

    RCCHECK(rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/white_slot/imu"));

    RCCHECK(rclc_subscription_init_default(
        &motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));





    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &motor_subscriber,
        &motor_msg,
        &twistCallback,
        ON_NEW_DATA));
        
    
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
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}




void Move(){
    float motor1Speed = motor_msg.linear.x;
    float motor2Speed = motor_msg.linear.y;

    float max_rpm = 150.0;

uint8_t duty1 = (uint8_t)((fabs(motor1Speed) / max_rpm) * 255.0);
uint8_t duty2 = (uint8_t)((fabs(motor2Speed) / max_rpm) * 255.0);


    // Motor A control
    if (motor1Speed > 0) {
        ledcWrite(PWM_CHANNEL_AIN1, duty1);
        ledcWrite(PWM_CHANNEL_AIN2, 0);
    } else if (motor1Speed < 0) {
        ledcWrite(PWM_CHANNEL_AIN1, 0);
        ledcWrite(PWM_CHANNEL_AIN2, duty1);
    } else {
        ledcWrite(PWM_CHANNEL_AIN1, 0);
        ledcWrite(PWM_CHANNEL_AIN2, 0);
    }

    // Motor B control
    if (motor2Speed > 0) {
        ledcWrite(PWM_CHANNEL_BIN1, duty2);
        ledcWrite(PWM_CHANNEL_BIN2, 0);
    } else if (motor2Speed < 0) {
        ledcWrite(PWM_CHANNEL_BIN1, 0);
        ledcWrite(PWM_CHANNEL_BIN2, duty2);
    } else {
        ledcWrite(PWM_CHANNEL_BIN1, 0);
        ledcWrite(PWM_CHANNEL_BIN2, 0);
    }

}


void publishData()
{

    encoder_msg.linear.x = encoderLeft.getCount();
    encoder_msg.linear.y = encoderRight.getCount();

    
    rcl_publish(&encoder_publisher,&encoder_msg, NULL);

        
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); 

    // yaw (heading) is .orientation.x from BNO055 in Euler mode
    imu_msg.angular.z = orientationData.orientation.x; 

    // You can also add pitch & roll if needed
    imu_msg.angular.x = orientationData.orientation.z; // roll
    imu_msg.angular.y = orientationData.orientation.y; // pitch

    rcl_publish(&imu_publisher, &imu_msg, NULL);
}

void syncTime()
{

    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    // Example implementation: blink LED rapidly then reboot or halt

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