#!/usr/bin/env python3
"""
Goal Reached Stop Controller
Nav2가 goal에 도달했을 때 강제로 로봇을 정지시킵니다.
NavigateToPose action의 결과를 모니터링하여 성공 시 zero cmd_vel 발행
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Empty
import time


class GoalReachedStopController(Node):
    def __init__(self):
        super().__init__('goal_reached_stop_controller')
        
        # Parameters
        self.declare_parameter('stop_duration', 1.0)  # 정지 명령 발행 시간 (초)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        
        self.stop_duration = self.get_parameter('stop_duration').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # QoS 설정
        qos = QoSProfile(depth=10)
        
        # Publisher: 정지 명령
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, qos)
        
        # Subscriber: NavigateToPose action status
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            qos
        )
        
        # 상태 추적
        self.last_goal_id = None
        self.goal_reached = False
        self.stop_timer = None
        self.stop_count = 0
        
        self.get_logger().info(
            f'Goal Reached Stop Controller started. '
            f'Will send zero cmd_vel for {self.stop_duration}s after goal reached.'
        )
    
    def status_callback(self, msg: GoalStatusArray):
        """NavigateToPose action status 모니터링"""
        if not msg.status_list:
            return
        
        # 가장 최근 status만 처리 (중복 방지)
        status = msg.status_list[-1]
        goal_id = bytes(status.goal_info.goal_id.uuid).hex()
        
        # 새로운 goal이면 상태 리셋
        if goal_id != self.last_goal_id:
            self.last_goal_id = goal_id
            self.goal_reached = False
            self.get_logger().debug(f'New goal detected: {goal_id[:8]}...')
        
        # Status codes:
        # 1 = ACCEPTED, 2 = EXECUTING, 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status.status == 4 and not self.goal_reached:  # SUCCEEDED
            self.goal_reached = True
            self.get_logger().info('🎯 Goal REACHED! Sending stop command...')
            self.start_stop_sequence()
        elif status.status == 5 and not self.goal_reached:  # CANCELED
            self.get_logger().info('Goal canceled')
            self.goal_reached = True  # 중복 로그 방지
        elif status.status == 6 and not self.goal_reached:  # ABORTED
            self.get_logger().info('Goal aborted')
            self.goal_reached = True  # 중복 로그 방지
    
    def start_stop_sequence(self):
        """정지 명령 시퀀스 시작"""
        self.stop_count = 0
        # 10Hz로 stop_duration 동안 정지 명령 발행
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(0.1, self.send_stop_cmd)
    
    def send_stop_cmd(self):
        """Zero velocity 발행"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(stop_cmd)
        self.stop_count += 1
        
        # stop_duration 동안 발행 후 종료
        if self.stop_count >= self.stop_duration * 10:
            self.stop_timer.cancel()
            self.stop_timer = None
            self.get_logger().info(f'✓ Stop sequence complete ({self.stop_count} msgs sent)')


def main(args=None):
    rclpy.init(args=args)
    node = GoalReachedStopController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
