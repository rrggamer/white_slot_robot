#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Encoder.h> // Your encoder reading library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// --- Robot parameters ---
double wheel_radius = 0.033; // meters
double wheel_base = 0.12;    // distance between wheels (m)

// --- Robot pose ---
double x = 0.0, y = 0.0, theta = 0.0;

// --- Loop timing ---
const int LOOP_DELAY_MS = 50;

// --- Hardware objects ---
Encoder leftEncoder(LEFT_PIN_A, LEFT_PIN_B);
Encoder rightEncoder(RIGHT_PIN_A, RIGHT_PIN_B);
IMU imuSensor;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

void setup() {
  // Initialize micro-ROS transport
  set_microros_transports();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rcl_node_t node;
  rclc_node_init_default(&node, "esp32_odom_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"
  );

  nav_msgs__msg__Odometry__init(&odom_msg);
  odom_msg.header.frame_id.data = strdup("odom");
  odom_msg.child_frame_id.data = strdup("base_link");

  // Initialize encoders and IMU
  leftEncoder.begin();
  rightEncoder.begin();
  imuSensor.begin();
}

void loop() {
  // Read encoder distances
  double left_dist = leftEncoder.getDistance() * wheel_radius;
  double right_dist = rightEncoder.getDistance() * wheel_radius;

  // Compute velocities
  double dt = LOOP_DELAY_MS / 1000.0;
  double v = (right_dist + left_dist) / 2.0 / dt;
  double omega = (right_dist - left_dist) / wheel_base / dt;

  // Integrate pose
  x += v * cos(theta) * dt;
  y += v * sin(theta) * dt;
  theta += omega * dt;

  // Fill odometry message
  uint32_t t_sec = millis() / 1000;
  uint32_t t_nsec = (millis() % 1000) * 1000000;
  odom_msg.header.stamp.sec = t_sec;
  odom_msg.header.stamp.nanosec = t_nsec;

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = omega;

  // Publish
  rcl_publish(&odom_publisher, &odom_msg, NULL);

  delay(LOOP_DELAY_MS);
}
