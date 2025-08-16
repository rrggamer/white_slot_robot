#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Subscribe to Nav2 velocity
        self.sub_nav = self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_callback, 10)

        # Publish to your robot's cmd_vel
        self.pub_robot = self.create_publisher(Twist, '/cmd_vel', 10)

    def nav_callback(self, msg):
        # Forward navigation velocity to robot
        cmd_vel_move = Twist()

        cmd_vel_move.linear.x = float(msg.linear.x * 10)
        cmd_vel_move.angular.z = float(msg.angular.z * 3)

        self.pub_robot.publish(cmd_vel_move)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
