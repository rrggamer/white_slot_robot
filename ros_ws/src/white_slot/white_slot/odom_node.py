#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped

class OdomNode(Node):
    def __init__(self):
        super().__init__('white_slot_odometry')

        # Robot params (adjust to your robot)
        self.wheel_radius = 0.05  # meters
        self.wheel_base = 0.20    # distance between wheels in meters
        self.encoder_cpr = 2048   # counts per revolution

        # State vars
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # heading (rad)
        self.prev_left_count = None
        self.prev_right_count = None

        # Subscribers
        self.encoder_sub = self.create_subscription(
            Twist, '/white_slot/encoder', self.encoder_callback, 10)
        self.imu_sub = self.create_subscription(
            Twist, '/white_slot/imu', self.imu_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_time = self.get_clock().now()

    def imu_callback(self, msg):
        # Assuming IMU gives yaw in msg.angular.z (degrees)
        self.theta = math.radians(msg.angular.z)

    def encoder_callback(self, msg):
        left_count = msg.linear.x
        right_count = msg.linear.y

        if self.prev_left_count is None:
            self.prev_left_count = left_count
            self.prev_right_count = right_count
            return

        # Δcounts
        delta_left = left_count - self.prev_left_count
        delta_right = right_count - self.prev_right_count
        self.prev_left_count = left_count
        self.prev_right_count = right_count

        # Distance per wheel
        left_dist = (2 * math.pi * self.wheel_radius) * (delta_left / self.encoder_cpr)
        right_dist = (2 * math.pi * self.wheel_radius) * (delta_right / self.encoder_cpr)

        # Δpose
        delta_s = (right_dist + left_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.wheel_base

        # Integrate position
        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Publish odom
        self.publish_odometry()

    def publish_odometry(self):
        current_time = self.get_clock().now()

        # Quaternion from yaw
        q = quaternion_from_euler(0, 0, self.theta)

        # Publish /odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
