#!/usr/bin/env python3
"""
Initial Rotation Controller (cmd_vel Mux 방식)
새로운 목표가 설정되면 Nav2 경로 추종 전에 로봇을 목표 방향으로 회전시킵니다.

=================================================================================
동작 원리 (토픽 분리 방식):
=================================================================================
1. Nav2 controller_server는 /nav2_cmd_vel로 발행하도록 설정
2. 이 노드는 /nav2_cmd_vel을 구독하고 /cmd_vel로 재발행 (패스스루)
3. 회전 필요 시:
   - Nav2 cmd_vel 패스스루 중지
   - 직접 회전 명령을 /cmd_vel로 발행
   - 회전 완료 후 패스스루 재개

장점:
- Nav2 lifecycle을 건드리지 않음 → action이 실패하지 않음
- bt_navigator와 controller_server는 정상 동작 유지
- 깔끔한 제어권 전환

설정 필요:
- sim_nav2.yaml에서 controller_server의 cmd_vel_topic을 /nav2_cmd_vel로 변경
=================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray
import tf2_ros
from tf2_ros import TransformException
import math
import numpy as np


def quaternion_to_yaw(q):
    """쿼터니언에서 yaw 각도 추출"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """각도를 -π ~ π 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class InitialRotationController(Node):
    def __init__(self):
        super().__init__('initial_rotation_controller')
        
        # =================================================================
        # Parameters
        # =================================================================
        self.declare_parameter('max_angular_velocity', 0.8)  # 최대 회전 속도 (rad/s)
        self.declare_parameter('angular_threshold', 0.3)  # 회전 시작 임계값 (rad) ~17도
        self.declare_parameter('angular_tolerance', 0.1)  # 회전 완료 허용오차 (rad) ~6도
        self.declare_parameter('control_frequency', 20.0)  # 제어 주기 (Hz)
        self.declare_parameter('min_angular_velocity', 0.15)  # 최소 각속도
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('global_frame', 'odom')
        self.declare_parameter('use_path_direction', False)  # True: 경로 lookahead 방향, False: 최종 목표 위치 방향 (권장)
        self.declare_parameter('path_lookahead_distance', 1.5)  # 경로에서 참조할 거리 (m) — use_path_direction=True일 때만 사용
        self.declare_parameter('enabled', True)
        
        # 토픽 설정
        self.declare_parameter('nav2_cmd_vel_topic', '/nav2_cmd_vel')  # Nav2 출력
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')  # 로봇 입력
        # Logging verbosity
        self.declare_parameter('verbose', False)  # Enable verbose logging
        
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.angular_tolerance = self.get_parameter('angular_tolerance').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.min_angular_velocity = self.get_parameter('min_angular_velocity').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.use_path_direction = self.get_parameter('use_path_direction').value
        self.path_lookahead_distance = self.get_parameter('path_lookahead_distance').value
        self.enabled = self.get_parameter('enabled').value
        
        nav2_cmd_vel_topic = self.get_parameter('nav2_cmd_vel_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.verbose = self.get_parameter('verbose').value
        
        # =================================================================
        # TF2 Buffer
        # =================================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # =================================================================
        # QoS Profiles
        # =================================================================
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # =================================================================
        # Publishers
        # =================================================================
        # 최종 cmd_vel (로봇으로 전송)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # =================================================================
        # Subscribers
        # =================================================================
        # Nav2 controller의 cmd_vel 구독 (패스스루용)
        self.nav2_cmd_vel_sub = self.create_subscription(
            Twist, nav2_cmd_vel_topic, self.nav2_cmd_vel_callback, 10)
        
        # 새 목표 감지
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, qos_reliable)
        
        # 경로 구독 (경로 방향 참조용)
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, qos_reliable)
        
        # Nav2 action status 모니터링
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            qos_reliable
        )
        
        # =================================================================
        # State Variables
        # =================================================================
        self.current_goal = None
        self.current_path = None
        self.target_yaw = None
        self.is_rotating = False
        self.rotation_complete = True   # True로 시작 — goal 수신 전까지 패스스루 허용
        self.passthrough_enabled = True  # Nav2 cmd_vel 패스스루 활성화
        self.last_goal_id = None
        self.nav2_executing = False
        self.waiting_for_path = False
        self._goal_from_topic = False   # /goal_pose 토픽에서 goal 수신 여부 추적
        
        # 안정적인 완료 판정을 위한 카운터
        self.stable_count = 0
        self.required_stable_count = 5  # 연속 5회 허용오차 내에 있어야 완료
        
        # 회전 시작 전 정지 확인
        self.stop_confirmed = False
        self.stop_count = 0
        self.required_stop_count = 3  # 정지 명령 3회 발행 후 회전 시작
        
        # 제어 타이머 (비활성 상태로 시작)
        self.control_timer = None
        
        self.get_logger().info('🔄 Initial Rotation Controller started')
        if self.verbose:
            self.get_logger().info(
                f'   Nav2 cmd_vel: {nav2_cmd_vel_topic} → {cmd_vel_topic}\n'
                f'   Max angular velocity: {self.max_angular_velocity} rad/s\n'
                f'   Min angular velocity: {self.min_angular_velocity} rad/s\n'
                f'   Angular threshold: {self.angular_threshold} rad ({math.degrees(self.angular_threshold):.1f}°)\n'
                f'   Angular tolerance: {self.angular_tolerance} rad ({math.degrees(self.angular_tolerance):.1f}°)\n'
                f'   Control frequency: {self.control_frequency} Hz\n'
                f'   Use path direction: {self.use_path_direction}\n'
                f'   Enabled: {self.enabled}'
            )
    
    # =================================================================
    # cmd_vel Mux (패스스루)
    # =================================================================
    def nav2_cmd_vel_callback(self, msg: Twist):
        """Nav2 cmd_vel을 로봇으로 패스스루 (회전 중이 아닐 때만)"""
        # 회전 중이거나 회전이 완료되지 않았으면 Nav2 명령 완전 차단
        if self.is_rotating:
            # 회전 중에는 Nav2 명령 완전 무시 (정지 명령도 발행하지 않음 - control_loop에서 관리)
            if self.verbose:
                self.get_logger().debug('Blocking Nav2 cmd_vel: rotation in progress')
            return
        
        if not self.rotation_complete:
            # 회전 대기 중에는 Nav2 명령 차단
            if self.verbose:
                self.get_logger().debug('Blocking Nav2 cmd_vel: waiting for rotation')
            return
        
        # 회전 완료 후에만 패스스루
        if self.passthrough_enabled:
            self.cmd_vel_pub.publish(msg)
    
    # =================================================================
    # Goal & Path Callbacks
    # =================================================================
    def goal_callback(self, msg: PoseStamped):
        """새 목표 수신 시 콜백 (/goal_pose 토픽)"""
        if not self.enabled:
            return
        
        self._goal_from_topic = True  # action status_callback에서 중복 처리 방지
        self.current_goal = msg
        self.rotation_complete = False
        self.current_path = None
        self.waiting_for_path = self.use_path_direction
        self.stable_count = 0
        self.stop_confirmed = False
        self.stop_count = 0
        self.passthrough_enabled = False  # 새 목표 수신 시 즉시 패스스루 차단
        
        if self.verbose:
            self.get_logger().info(
                f'📍 New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            )
        
        # 경로를 사용하지 않으면 즉시 목표 방향 계산
        if not self.use_path_direction:
            self.calculate_target_direction_from_goal()
    
    def path_callback(self, msg: Path):
        """경로 수신 시 콜백"""
        if not self.enabled:
            return
        
        if len(msg.poses) < 2:
            self.get_logger().warn('Path too short, ignoring')
            return
        
        self.current_path = msg
        
        # Action API로 goal이 전송된 경우 경로 마지막 점에서 목표 위치 추출
        if self.current_goal is None:
            self.current_goal = msg.poses[-1]
            if self.verbose:
                gp = self.current_goal.pose.position
                self.get_logger().info(
                    f'📍 Goal position extracted from path: ({gp.x:.2f}, {gp.y:.2f})')
        
        # 이미 회전 완료했으면 무시
        if self.rotation_complete:
            return
        
        # 경로 대기 중이었으면 방향 계산
        if self.waiting_for_path:
            self.waiting_for_path = False
            if self.use_path_direction:
                self.calculate_target_direction_from_path()
            else:
                # 경로에서 목표 위치를 추출했으므로 목표 방향으로 회전
                self.calculate_target_direction_from_goal()
    
    def status_callback(self, msg: GoalStatusArray):
        """Nav2 action status 모니터링
        
        Action API(goal_sender)로 보낸 goal은 /goal_pose 토픽을 거치지 않으므로
        여기서 새 goal을 감지하여 회전 로직을 트리거해야 합니다.
        """
        if not msg.status_list:
            return
        
        # 가장 최근 status만 처리
        status = msg.status_list[-1]
        goal_id = bytes(status.goal_info.goal_id.uuid).hex()
        
        # 새로운 goal 감지
        if goal_id != self.last_goal_id:
            self.last_goal_id = goal_id
            
            if self._goal_from_topic:
                # /goal_pose에서 이미 처리됨 — 중복 처리 방지
                self._goal_from_topic = False
                if self.verbose:
                    self.get_logger().info('Action goal matches /goal_pose — already handled')
            else:
                # Action API로 직접 보낸 goal → /goal_pose 미경유
                # goal_callback과 동일한 상태 초기화 수행
                if self.enabled:
                    self.get_logger().info(
                        '📍 New action goal detected (no /goal_pose) — setting up rotation')
                    self.current_goal = None  # 목표 위치 모름 → path에서 추출
                    self.rotation_complete = False
                    self.current_path = None
                    self.stable_count = 0
                    self.stop_confirmed = False
                    self.stop_count = 0
                    self.passthrough_enabled = False  # Nav2 cmd_vel 차단
                    # 목표 위치를 path에서 추출해야 하므로 항상 path 대기
                    self.waiting_for_path = True
        
        # Status: 2 = EXECUTING
        self.nav2_executing = (status.status == 2)
        
        # Goal 완료/취소/실패 시에만 패스스루 재활성화 (회전 중이 아닐 때)
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status.status in [4, 5, 6] and not self.is_rotating:
            self.passthrough_enabled = True
            self.rotation_complete = True
    
    # =================================================================
    # Direction Calculation
    # =================================================================
    def calculate_target_direction_from_goal(self):
        """목표 위치 기준으로 방향 계산"""
        if self.current_goal is None:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            goal_x = self.current_goal.pose.position.x
            goal_y = self.current_goal.pose.position.y
            
            dx = goal_x - robot_x
            dy = goal_y - robot_y
            
            if abs(dx) < 0.01 and abs(dy) < 0.01:
                self.get_logger().warn('Goal too close, skipping rotation')
                self.rotation_complete = True
                return
            
            self.target_yaw = math.atan2(dy, dx)
            self.start_rotation_if_needed()
            
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
    
    def calculate_target_direction_from_path(self):
        """경로 기준으로 방향 계산 (lookahead 거리의 포인트 방향)"""
        if self.current_path is None or len(self.current_path.poses) < 2:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # 경로에서 lookahead 거리만큼 떨어진 점 찾기
            target_point = None
            accumulated_dist = 0.0
            
            for i in range(1, len(self.current_path.poses)):
                p0 = self.current_path.poses[i-1].pose.position
                p1 = self.current_path.poses[i].pose.position
                
                segment_dist = math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2)
                accumulated_dist += segment_dist
                
                if accumulated_dist >= self.path_lookahead_distance:
                    target_point = p1
                    break
            
            if target_point is None:
                target_point = self.current_path.poses[-1].pose.position
            
            dx = target_point.x - robot_x
            dy = target_point.y - robot_y
            
            if abs(dx) < 0.01 and abs(dy) < 0.01:
                self.get_logger().warn('Path target too close, skipping rotation')
                self.rotation_complete = True
                return
            
            self.target_yaw = math.atan2(dy, dx)
            
            if self.verbose:
                self.get_logger().info(
                    f'📐 Target direction from path: {math.degrees(self.target_yaw):.1f}° '
                    f'(lookahead point: {target_point.x:.2f}, {target_point.y:.2f})'
                )
            
            self.start_rotation_if_needed()
            
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
    
    # =================================================================
    # Rotation Control
    # =================================================================
    def start_rotation_if_needed(self):
        """필요시 회전 시작"""
        if self.target_yaw is None or self.rotation_complete:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            current_yaw = quaternion_to_yaw(transform.transform.rotation)
            angle_diff = normalize_angle(self.target_yaw - current_yaw)
            
            if self.verbose:
                self.get_logger().info(
                    f'🧭 Current yaw: {math.degrees(current_yaw):.1f}°, '
                    f'Target yaw: {math.degrees(self.target_yaw):.1f}°, '
                    f'Diff: {math.degrees(angle_diff):.1f}°'
                )
            
            if abs(angle_diff) > self.angular_threshold:
                # 먼저 패스스루 중지하고 정지 명령 발행
                self.passthrough_enabled = False
                self.is_rotating = True
                self.stop_confirmed = False
                self.stop_count = 0
                self.stable_count = 0  # 안정화 카운터 리셋
                
                # 즉시 정지 명령 발행
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.linear.y = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(stop_cmd)
                
                self.get_logger().info(
                    f'🔄 Starting rotation: {math.degrees(angle_diff):.1f}° (stopping first...)'
                )
                
                # 제어 타이머 시작
                if self.control_timer is None:
                    self.control_timer = self.create_timer(
                        1.0 / self.control_frequency,
                        self.control_loop
                    )
            else:
                if self.verbose:
                    self.get_logger().info(
                        f'✓ Already aligned (diff: {math.degrees(angle_diff):.1f}°), skipping rotation'
                    )
                self.rotation_complete = True
                self.passthrough_enabled = True
                
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
    
    def control_loop(self):
        """회전 제어 루프"""
        if not self.is_rotating or self.target_yaw is None:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            current_yaw = quaternion_to_yaw(transform.transform.rotation)
            angle_diff = normalize_angle(self.target_yaw - current_yaw)
            
            # 먼저 정지 확인 (회전 시작 전 로봇이 정지했는지 확인)
            if not self.stop_confirmed:
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.linear.y = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(stop_cmd)
                self.stop_count += 1
                
                if self.stop_count >= self.required_stop_count:
                    self.stop_confirmed = True
                    self.get_logger().info('🛑 Robot stopped, starting pure rotation')
                return
            
            # 허용오차 내에 들어오면 안정화 카운터 증가
            if abs(angle_diff) <= self.angular_tolerance:
                self.stable_count += 1
                
                # 안정화 카운터가 충분히 쌓이면 회전 완료
                if self.stable_count >= self.required_stable_count:
                    self.complete_rotation()
                    return
                else:
                    # 아직 안정화 중 - 정지 상태 유지
                    stop_cmd = Twist()
                    stop_cmd.linear.x = 0.0
                    stop_cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(stop_cmd)
                    if self.verbose:
                        self.get_logger().info(
                            f'⏳ Stabilizing... ({self.stable_count}/{self.required_stable_count})'
                        )
                    return
            else:
                # 허용오차 밖이면 카운터 리셋
                self.stable_count = 0
            
            # 회전 속도 계산 (순수 회전만, linear는 항상 0)
            angular_vel = self.max_angular_velocity * np.sign(angle_diff)
            
            # 목표에 가까워지면 속도 줄이기 (부드러운 정지)
            if abs(angle_diff) < 0.5:  # ~30도 이내
                scale = abs(angle_diff) / 0.5
                angular_vel = angular_vel * max(scale, self.min_angular_velocity / self.max_angular_velocity)
            
            # 최소 속도 보장
            if abs(angular_vel) < self.min_angular_velocity:
                angular_vel = self.min_angular_velocity * np.sign(angle_diff)
            
            # 순수 회전 명령 발행 (linear.x = 0 보장)
            cmd = Twist()
            cmd.linear.x = 0.0  # 명시적으로 0
            cmd.linear.y = 0.0
            cmd.angular.z = float(angular_vel)
            self.cmd_vel_pub.publish(cmd)
            
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed in control loop: {e}')
    
    def complete_rotation(self):
        """회전 완료 처리"""
        self.is_rotating = False
        self.rotation_complete = True
        self.passthrough_enabled = True  # Nav2 cmd_vel 패스스루 재개
        self.stable_count = 0
        self.stop_confirmed = False
        self.stop_count = 0
        
        # 정지 명령 여러 번 발행하여 확실히 정지
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.angular.z = 0.0
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_cmd)
        
        # 타이머 중지
        if self.control_timer is not None:
            self.control_timer.cancel()
            self.control_timer = None
        
        self.get_logger().info('✅ Rotation complete, handing over to Nav2')


def main(args=None):
    rclpy.init(args=args)
    node = InitialRotationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
