#!/usr/bin/env python3
"""
Goal Pose & Path Z-Corrector Node
RViz에서 설정한 goal pose와 Nav2 경로의 z좌표를 실제 지면 높이로 보정합니다.
/tf_gt에서 base_link의 z 위치를 자동으로 감지하여 사용합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage


class GoalPoseCorrectorNode(Node):
    def __init__(self):
        super().__init__('goal_pose_corrector')
        
        # Parameters
        self.declare_parameter('ground_height', -1.0)  # -1 = auto-detect from tf_gt
        self.declare_parameter('input_topic', '/goal_pose_raw')
        self.declare_parameter('output_topic', '/goal_pose')
        self.declare_parameter('base_frame', 'base_link')
        
        self.ground_height = self.get_parameter('ground_height').value
        self.auto_detect = (self.ground_height < 0)  # 음수면 자동 감지
        self.ground_height_detected = False
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Subscriber: /tf_gt for ground truth base_link position
        if self.auto_detect:
            self.tf_gt_sub = self.create_subscription(
                TFMessage,
                '/tf_gt',
                self.tf_gt_callback,
                10
            )
            self.get_logger().info('Auto-detecting ground height from /tf_gt...')
        
        # Subscriber: RViz에서 오는 raw goal pose
        self.goal_sub = self.create_subscription(
            PoseStamped,
            input_topic,
            self.goal_callback,
            10
        )
        
        # Publisher: 보정된 goal pose
        self.goal_pub = self.create_publisher(
            PoseStamped,
            output_topic,
            10
        )
        
        # ===== Path Z-correction =====
        # Nav2 planner가 발행하는 원본 경로
        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )
        
        # 보정된 경로 (RViz 시각화용)
        self.plan_pub = self.create_publisher(
            Path,
            '/plan_corrected',
            10
        )
        
        # bt_navigator의 경로도 보정
        self.bt_plan_sub = self.create_subscription(
            Path,
            '/transformed_global_plan',
            self.bt_plan_callback,
            10
        )
        self.bt_plan_pub = self.create_publisher(
            Path,
            '/transformed_global_plan_corrected',
            10
        )
        
        mode_str = "auto-detect" if self.auto_detect else f"fixed={self.ground_height}m"
        self.get_logger().info(
            f'Goal Pose & Path Corrector started: '
            f'ground_height={mode_str}, '
            f'{input_topic} -> {output_topic}'
        )
    
    def tf_gt_callback(self, msg: TFMessage):
        """Extract base_link z position from /tf_gt (first value only)"""
        if self.ground_height_detected:
            return  # 이미 감지됨, 더 이상 처리 안함
        
        for transform in msg.transforms:
            if transform.child_frame_id == self.base_frame:
                self.ground_height = transform.transform.translation.z
                self.ground_height_detected = True
                self.get_logger().info(
                    f'Ground height detected from /tf_gt: {self.ground_height:.3f}m '
                    f'(base_link z position)'
                )
                # 감지 후 구독 해제 (리소스 절약)
                self.destroy_subscription(self.tf_gt_sub)
                break
    
    def get_effective_height(self):
        """Get the effective ground height to use"""
        if self.auto_detect and not self.ground_height_detected:
            return 0.0  # 아직 감지 안됨
        return self.ground_height
    
    def goal_callback(self, msg: PoseStamped):
        effective_height = self.get_effective_height()
        
        # ground_height가 아직 감지되지 않았으면 경고
        if self.auto_detect and not self.ground_height_detected:
            self.get_logger().warn(
                'Ground height not yet detected from /tf_gt. '
                'Using z=0.0 temporarily. Waiting for /tf_gt...'
            )
        
        # Z좌표를 지면 높이로 보정
        corrected_msg = PoseStamped()
        corrected_msg.header = msg.header
        corrected_msg.pose.position.x = msg.pose.position.x
        corrected_msg.pose.position.y = msg.pose.position.y
        corrected_msg.pose.position.z = effective_height  # 지면 높이로 설정
        
        # Orientation은 그대로 유지 (2D이므로 roll, pitch는 0)
        corrected_msg.pose.orientation = msg.pose.orientation
        
        self.goal_pub.publish(corrected_msg)
        
        self.get_logger().info(
            f'Goal corrected: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, '
            f'{msg.pose.position.z:.2f}) -> z={effective_height:.3f}m'
        )
    
    def correct_path(self, msg: Path) -> Path:
        """Correct z coordinates of all poses in the path"""
        effective_height = self.get_effective_height()
        
        corrected_path = Path()
        corrected_path.header = msg.header
        
        for pose_stamped in msg.poses:
            corrected_pose = PoseStamped()
            corrected_pose.header = pose_stamped.header
            corrected_pose.pose.position.x = pose_stamped.pose.position.x
            corrected_pose.pose.position.y = pose_stamped.pose.position.y
            corrected_pose.pose.position.z = effective_height  # z 보정
            corrected_pose.pose.orientation = pose_stamped.pose.orientation
            corrected_path.poses.append(corrected_pose)
        
        return corrected_path
    
    def plan_callback(self, msg: Path):
        """Correct and republish the global plan"""
        corrected_path = self.correct_path(msg)
        self.plan_pub.publish(corrected_path)
    
    def bt_plan_callback(self, msg: Path):
        """Correct and republish the transformed global plan"""
        corrected_path = self.correct_path(msg)
        self.bt_plan_pub.publish(corrected_path)


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseCorrectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
