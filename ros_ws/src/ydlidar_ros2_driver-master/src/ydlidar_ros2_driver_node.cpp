#include "src/CYdLidar.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>

#define ROS2Version "1.0.1"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

    RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s", ROS2Version);

    CYdLidar laser;

    // --- HARD-CODED SETTINGS ---
    std::string port = "/dev/ydlidar";
    laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

    int baudrate = 115200;
    laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

    int lidar_type = TYPE_TRIANGLE;
    laser.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

    int device_type = YDLIDAR_TYPE_SERIAL;
    laser.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));

    int sample_rate = 3;
    laser.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));

    bool fixed_res = true;
    laser.setlidaropt(LidarPropFixedResolution, &fixed_res, sizeof(bool));

    bool reversion = false;
    laser.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));

    bool inverted = true;
    laser.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));

    bool auto_reconnect = true;
    laser.setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));

    bool single_channel = true;
    laser.setlidaropt(LidarPropSingleChannel, &single_channel, sizeof(bool));

    bool intensity = false;
    laser.setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));

    float angle_max = 180.0f;
    laser.setlidaropt(LidarPropMaxAngle, &angle_max, sizeof(float));

    float angle_min = -180.0f;
    laser.setlidaropt(LidarPropMinAngle, &angle_min, sizeof(float));

    float range_max = 64.0f;
    laser.setlidaropt(LidarPropMaxRange, &range_max, sizeof(float));

    float range_min = 0.01f;
    laser.setlidaropt(LidarPropMinRange, &range_min, sizeof(float));

    float frequency = 10.0f;
    laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));
    // ---------------------------

    // Initialize and start the LiDAR
    if (!laser.initialize()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize LiDAR!");
        return -1;
    }

    if (!laser.turnOn()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start LiDAR scan!");
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "LiDAR running correctly!");

    // Publisher
    auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));

    // Services to start/stop scan
    auto stop_scan_service =
        [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                 std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
    {
        return laser.turnOff();
    };
    auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_service);

    auto start_scan_service =
        [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                 std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
    {
        return laser.turnOn();
    };
    auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_service);

    rclcpp::WallRate loop_rate(20);

    // Main loop
    while (rclcpp::ok()) {
        LaserScan scan;
        if (laser.doProcessSimple(scan)) {
            auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
            scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
            scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
            scan_msg->header.frame_id = "laser";
            scan_msg->angle_min = scan.config.min_angle;
            scan_msg->angle_max = scan.config.max_angle;
            scan_msg->angle_increment = scan.config.angle_increment;
            scan_msg->scan_time = scan.config.scan_time;
            scan_msg->time_increment = scan.config.time_increment;
            scan_msg->range_min = scan.config.min_range;
            scan_msg->range_max = scan.config.max_range;

            int size = static_cast<int>((scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment) + 1;
            scan_msg->ranges.resize(size);
            scan_msg->intensities.resize(size);

            for (size_t i = 0; i < scan.points.size(); i++) {
                int index = std::ceil((scan.points[i].angle - scan.config.min_angle) / scan.config.angle_increment);
                if (index >= 0 && index < size) {
                    scan_msg->ranges[index] = scan.points[i].range;
                    scan_msg->intensities[index] = scan.points[i].intensity;
                }
            }
            laser_pub->publish(*scan_msg);
        } else {
            RCLCPP_WARN(node->get_logger(), "Failed to get scan");
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Stopping LiDAR ...");
    laser.turnOff();
    laser.disconnecting();
    rclcpp::shutdown();

    return 0;
}
