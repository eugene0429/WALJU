#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
RViz Goal Printer

Listens to /goal_pose topic from RViz2 and prints the coordinates
in a format suitable for the goal_sender node.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import sys

def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle (radians)."""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class RvizGoalPrinter(Node):
    def __init__(self):
        super().__init__('rviz_goal_printer')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('Ready to receive goals from RViz2 (/goal_pose)...')
        self.get_logger().info('Click "2D Goal Pose" in RViz to see coordinates.')

    def listener_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        
        yaw_rad = quaternion_to_yaw(orientation)
        yaw_deg = math.degrees(yaw_rad)
        
        print("\n" + "="*60)
        print(f"📍 Received Goal from RViz:")
        print(f"   Position: x={position.x:.4f}, y={position.y:.4f}, z={position.z:.4f}")
        print(f"   Orientation: yaw={yaw_rad:.4f} rad ({yaw_deg:.2f}°)")
        print("-" * 60)
        print("📋 Command to reproduce this goal:")
        print(f"\nros2 run nvblox_integration goal_sender --ros-args -p x:={position.x:.4f} -p y:={position.y:.4f} -p yaw:={yaw_rad:.4f}\n")
        print("="*60 + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = RvizGoalPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
