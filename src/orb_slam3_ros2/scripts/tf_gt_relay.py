#!/usr/bin/python3
"""
TF Ground Truth Relay Node

Subscribes to a TF topic and republishes transforms to another TF topic.
Supports frame filtering and renaming.

Features:
- Relay /tf_gt (map -> base_link) to /tf for real-time robot pose updates
- Relay /tf (base_link -> Camera_*) to /tf with frame renaming (-> camera_link)

Usage:
    # Relay /tf_gt to /tf
    ros2 run orb_slam3_ros2 tf_gt_relay.py --ros-args \
        -p source_topic:=/tf_gt \
        -p target_topic:=/tf \
        -p child_frame_filter:=base_link
    
    # Relay camera frame with renaming
    ros2 run orb_slam3_ros2 tf_gt_relay.py --ros-args \
        -p source_topic:=/tf \
        -p target_topic:=/tf \
        -p child_frame_filter:=Camera_OmniVision_OV9782_Left \
        -p rename_child_frame:=camera_link
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class TfGtRelay(Node):
    def __init__(self):
        super().__init__('tf_gt_relay')
        
        # Parameters
        self.declare_parameter('source_topic', '/tf_gt')
        self.declare_parameter('target_topic', '/tf')
        self.declare_parameter('child_frame_filter', 'base_link')  # Only relay this child frame
        self.declare_parameter('rename_child_frame', '')  # If set, rename child frame to this
        self.declare_parameter('rename_parent_frame', '')  # If set, rename parent frame to this
        
        source_topic = self.get_parameter('source_topic').get_parameter_value().string_value
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self.child_frame_filter = self.get_parameter('child_frame_filter').get_parameter_value().string_value
        self.rename_child_frame = self.get_parameter('rename_child_frame').get_parameter_value().string_value
        self.rename_parent_frame = self.get_parameter('rename_parent_frame').get_parameter_value().string_value
        
        self.get_logger().info(f'TF Relay: {source_topic} -> {target_topic}')
        self.get_logger().info(f'  Child frame filter: {self.child_frame_filter}')
        if self.rename_child_frame:
            self.get_logger().info(f'  Rename child frame: {self.child_frame_filter} -> {self.rename_child_frame}')
        if self.rename_parent_frame:
            self.get_logger().info(f'  Rename parent frame: -> {self.rename_parent_frame}')
        
        # QoS for TF topics
        tf_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # Subscriber
        self.subscription = self.create_subscription(
            TFMessage,
            source_topic,
            self.tf_callback,
            tf_qos
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            TFMessage,
            target_topic,
            tf_qos
        )
        
        self.msg_count = 0
        
    def tf_callback(self, msg: TFMessage):
        """Relay transforms with optional frame renaming"""
        relayed_transforms = []
        
        for transform in msg.transforms:
            # Filter by child frame if specified
            if self.child_frame_filter and transform.child_frame_id != self.child_frame_filter:
                continue
            
            # Create a copy of the transform
            relayed = TransformStamped()
            relayed.header.stamp = transform.header.stamp  # Keep original timestamp
            relayed.transform = transform.transform
            
            # Rename parent frame if specified
            if self.rename_parent_frame:
                relayed.header.frame_id = self.rename_parent_frame
            else:
                relayed.header.frame_id = transform.header.frame_id
            
            # Rename child frame if specified
            if self.rename_child_frame:
                relayed.child_frame_id = self.rename_child_frame
            else:
                relayed.child_frame_id = transform.child_frame_id
            
            relayed_transforms.append(relayed)
        
        if relayed_transforms:
            out_msg = TFMessage()
            out_msg.transforms = relayed_transforms
            self.publisher.publish(out_msg)
            
            self.msg_count += 1
            if self.msg_count == 1 or self.msg_count % 500 == 0:
                t = relayed_transforms[0]
                self.get_logger().info(
                    f'Relayed {self.msg_count}: '
                    f'{t.header.frame_id} -> {t.child_frame_id} '
                    f'[{t.transform.translation.x:.3f}, {t.transform.translation.y:.3f}, {t.transform.translation.z:.3f}]'
                )


def main(args=None):
    rclpy.init(args=args)
    node = TfGtRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
