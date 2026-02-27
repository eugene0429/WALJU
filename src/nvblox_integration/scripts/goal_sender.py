#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Goal Sender Node for Nav2 Navigation

A node that allows sending navigation goals to Nav2 via:
1. Command line arguments (single goal)
2. Interactive input mode
3. ROS2 topic commands
4. Sequential waypoint mode (multiple goals with wait between each)

Usage:
    # Send a single goal
    ros2 run nvblox_integration goal_sender --ros-args -p x:=5.0 -p y:=3.0 -p yaw:=1.57
    
    # Interactive mode
    ros2 run nvblox_integration goal_sender --ros-args -p interactive:=true
    
    # Sequential waypoint mode (JSON list: [[x, y, yaw_rad], ...])
    ros2 run nvblox_integration goal_sender --ros-args \\
        -p waypoints:='[[5.0,3.0,0.0],[10.0,-2.0,1.57],[0.0,0.0,0.0]]' \\
        -p wait_between_goals:=3.0 \\
        -p loop:=false
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

import json
import math
import sys
import threading
import time


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert Euler angles (in radians) to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class GoalSender(Node):
    """Node that sends navigation goals to Nav2."""
    
    def __init__(self):
        super().__init__('goal_sender')
        
        # Declare parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', 0.0)  # in radians
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('interactive', False)
        # Note: 'use_sim_time' is automatically declared in ROS 2 Jazzy

        # Sequential waypoint parameters
        self.declare_parameter('waypoints', '')          # JSON: [[x,y,yaw], ...]
        self.declare_parameter('waypoints_file', '')     # Path to JSON file
        self.declare_parameter('wait_between_goals', 3.0)  # seconds to wait after each goal
        self.declare_parameter('loop', False)            # repeat waypoint list endlessly
        
        # Get parameters
        self.frame_id = self.get_parameter('frame_id').value
        self.interactive = self.get_parameter('interactive').value
        self.wait_between_goals = self.get_parameter('wait_between_goals').value
        self.loop = self.get_parameter('loop').value
        
        # Create action client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create subscriber for goal commands (topic-based)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.goal_sub = self.create_subscription(
            String,
            'nav_goal_command',
            self.goal_command_callback,
            qos
        )
        
        # Create publisher for navigation status
        self.status_pub = self.create_publisher(String, 'nav_status', 10)
        
        # Goal tracking
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._current_goal_str = ''  # e.g. '(20.00, -6.00) yaw=-16.7°'

        # Sequential waypoint state
        self._waypoint_list = []
        self._waypoint_index = 0
        self._sequential_active = False
        self._nav_done_event = threading.Event()
        self._last_nav_result = None  # 'SUCCEEDED', 'ABORTED', 'CANCELED', ...
        
        self.get_logger().info('Goal Sender Node initialized')
        self.get_logger().info(f'Frame ID: {self.frame_id}')
        self.get_logger().info(f'Interactive mode: {self.interactive}')
        
        # Wait for Nav2 action server
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Nav2 action server not available! Goals may fail.')
        else:
            self.get_logger().info('Nav2 action server connected!')

        # Parse waypoints
        waypoints_str = self.get_parameter('waypoints').value
        waypoints_file = self.get_parameter('waypoints_file').value
        waypoints = self._parse_waypoints(waypoints_str, waypoints_file)
        
        # Decide mode
        if waypoints:
            self._waypoint_list = waypoints
            self.get_logger().info(f'Sequential waypoint mode: {len(waypoints)} goals, '
                                   f'wait={self.wait_between_goals}s, loop={self.loop}')
            for i, wp in enumerate(waypoints):
                self.get_logger().info(f'  WP[{i}]: ({wp[0]:.2f}, {wp[1]:.2f}) yaw={math.degrees(wp[2]):.1f}°')
            self._start_sequential_mode()
        elif self.interactive:
            self._start_interactive_mode()
        else:
            # Send goal from parameters if provided
            x = self.get_parameter('x').value
            y = self.get_parameter('y').value
            z = self.get_parameter('z').value
            yaw = self.get_parameter('yaw').value
            
            if x != 0.0 or y != 0.0:
                self.get_logger().info(f'Sending goal from parameters: x={x}, y={y}, z={z}, yaw={yaw}')
                self.send_goal(x, y, z, yaw)

    def _parse_waypoints(self, waypoints_str: str, waypoints_file: str) -> list:
        """Parse waypoints from JSON string or file.
        
        Supported formats:
          - [[x, y, yaw], [x, y, yaw], ...]       (yaw in radians)
          - [[x, y], [x, y], ...]                  (yaw defaults to 0)
          - [{"x": 1, "y": 2, "yaw": 0.5}, ...]   (dict format)
        """
        raw = None
        
        # Try file first
        if waypoints_file:
            try:
                with open(waypoints_file, 'r') as f:
                    raw = json.load(f)
                self.get_logger().info(f'Loaded waypoints from file: {waypoints_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load waypoints file "{waypoints_file}": {e}')
        
        # Then try string parameter
        if raw is None and waypoints_str:
            try:
                raw = json.loads(waypoints_str)
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Failed to parse waypoints JSON: {e}')
                return []
        
        if raw is None or not isinstance(raw, list) or len(raw) == 0:
            return []
        
        result = []
        for i, wp in enumerate(raw):
            try:
                if isinstance(wp, dict):
                    x = float(wp.get('x', 0.0))
                    y = float(wp.get('y', 0.0))
                    yaw = float(wp.get('yaw', 0.0))
                elif isinstance(wp, (list, tuple)):
                    x = float(wp[0])
                    y = float(wp[1])
                    yaw = float(wp[2]) if len(wp) >= 3 else 0.0
                else:
                    self.get_logger().warn(f'Skipping invalid waypoint[{i}]: {wp}')
                    continue
                result.append((x, y, yaw))
            except (IndexError, ValueError, TypeError) as e:
                self.get_logger().warn(f'Skipping waypoint[{i}]: {e}')
        
        return result

    def _start_sequential_mode(self):
        """Start sequential waypoint execution in a background thread."""
        thread = threading.Thread(target=self._sequential_worker, daemon=True)
        thread.start()

    def _sequential_worker(self):
        """Background thread that sends waypoints one by one."""
        self._sequential_active = True
        self._waypoint_index = 0
        total = len(self._waypoint_list)
        iteration = 0

        while rclpy.ok() and self._sequential_active:
            iteration += 1
            self.get_logger().info('')
            self.get_logger().info('=' * 55)
            if self.loop:
                self.get_logger().info(f'  Sequential Navigation — Loop #{iteration}')
            else:
                self.get_logger().info(f'  Sequential Navigation — {total} waypoints')
            self.get_logger().info('=' * 55)

            for idx in range(total):
                if not rclpy.ok() or not self._sequential_active:
                    break

                self._waypoint_index = idx
                x, y, yaw = self._waypoint_list[idx]

                self.get_logger().info('')
                self.get_logger().info(f'━━━ Waypoint [{idx + 1}/{total}]: '
                                       f'({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.1f}° ━━━')

                # Clear event and send goal
                self._nav_done_event.clear()
                self._last_nav_result = None
                self.send_goal(x, y, 0.0, yaw)

                # Block until navigation finishes
                self._nav_done_event.wait()
                result = self._last_nav_result or 'UNKNOWN'

                self.get_logger().info(f'  Waypoint [{idx + 1}/{total}] result: {result}')

                if result == 'ABORTED':
                    self.get_logger().warn(f'  Navigation aborted at WP[{idx + 1}]. Skipping to next.')

                # Wait between goals (unless it's the very last one and not looping)
                is_last = (idx == total - 1)
                if not is_last or self.loop:
                    if rclpy.ok() and self._sequential_active:
                        next_wp = idx + 2 if not is_last else 1
                        self.get_logger().info(
                            f'  Waiting {self.wait_between_goals:.1f}s before WP[{next_wp}]...')
                        time.sleep(self.wait_between_goals)

            if not self.loop:
                break

        self._sequential_active = False
        self.get_logger().info('')
        self.get_logger().info('=' * 55)
        self.get_logger().info('  ✅ Sequential navigation complete!')
        self.get_logger().info('=' * 55)

    def goal_command_callback(self, msg: String):
        """Handle goal commands from topic.
        
        Format: "x y yaw" or "x y z yaw" (space separated)
        Examples: "5.0 3.0 0.0" or "5.0 3.0 0.0 1.57"
        """
        try:
            parts = msg.data.strip().split()
            if len(parts) == 3:
                x, y, yaw = float(parts[0]), float(parts[1]), float(parts[2])
                z = 0.0
            elif len(parts) == 4:
                x, y, z, yaw = [float(p) for p in parts]
            else:
                self.get_logger().error(f'Invalid goal format: {msg.data}')
                return
            
            self.get_logger().info(f'Received goal command: x={x}, y={y}, z={z}, yaw={yaw}')
            self.send_goal(x, y, z, yaw)
            
        except ValueError as e:
            self.get_logger().error(f'Failed to parse goal: {e}')

    def send_goal(self, x: float, y: float, z: float = 0.0, yaw: float = 0.0):
        """Send a navigation goal to Nav2."""
        if not self._action_client.server_is_ready():
            self.get_logger().error('Nav2 action server not ready!')
            self._publish_status('ERROR: Nav2 server not ready')
            return
        
        # Create goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position = Point(x=x, y=y, z=z)
        goal_msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw)
        
        yaw_deg = math.degrees(yaw)
        self._current_goal_str = f'({x:.2f}, {y:.2f}) yaw={yaw_deg:.1f}°'
        self.get_logger().info(
            f'Sending goal: position=({x:.2f}, {y:.2f}, {z:.2f}), yaw={yaw_deg:.1f}°'
        )
        self._publish_status(f'SENDING: {self._current_goal_str}')
        
        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle response from Nav2 when goal is accepted/rejected."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self._publish_status('REJECTED')
            return
        
        self.get_logger().info('Goal accepted!')
        self._publish_status(f'ACCEPTED: {self._current_goal_str}')
        self._goal_handle = goal_handle
        
        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        """Handle feedback during navigation."""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        pos = current_pose.position
        distance = feedback.distance_remaining
        
        self.get_logger().info(
            f'Navigating... Position: ({pos.x:.2f}, {pos.y:.2f}), '
            f'Distance remaining: {distance:.2f}m',
            throttle_duration_sec=2.0
        )
        self._publish_status(f'NAVIGATING: {distance:.2f}m remaining')

    def _get_result_callback(self, future):
        """Handle the final result of navigation."""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            self._publish_status(f'SUCCEEDED: {self._current_goal_str}')
            self._last_nav_result = 'SUCCEEDED'
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted!')
            self._publish_status(f'ABORTED: {self._current_goal_str}')
            self._last_nav_result = 'ABORTED'
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled!')
            self._publish_status(f'CANCELED: {self._current_goal_str}')
            self._last_nav_result = 'CANCELED'
        else:
            self.get_logger().warn(f'Navigation finished with status: {status}')
            self._publish_status(f'FINISHED: status={status}')
            self._last_nav_result = f'FINISHED:{status}'
        
        self._goal_handle = None
        # Signal sequential mode that this goal is done
        self._nav_done_event.set()

    def cancel_goal(self):
        """Cancel the current navigation goal."""
        if self._goal_handle is not None:
            self.get_logger().info('Canceling navigation goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
        else:
            self.get_logger().warn('No active goal to cancel')

    def _cancel_done_callback(self, future):
        """Handle cancel response."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self._publish_status('CANCELED')
        else:
            self.get_logger().warn('Goal cancel request failed')

    def _publish_status(self, status: str):
        """Publish navigation status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _start_interactive_mode(self):
        """Start interactive mode for entering goals."""
        def input_thread():
            self.get_logger().info('')
            self.get_logger().info('=' * 50)
            self.get_logger().info('INTERACTIVE MODE - Enter navigation goals')
            self.get_logger().info('=' * 50)
            self.get_logger().info('Commands:')
            self.get_logger().info('  x y [yaw]     - Send goal (yaw in degrees, default=0)')
            self.get_logger().info('  cancel        - Cancel current goal')
            self.get_logger().info('  quit / q      - Exit')
            self.get_logger().info('=' * 50)
            
            while rclpy.ok():
                try:
                    user_input = input('\n[Goal] Enter "x y [yaw_deg]" or command: ').strip()
                    
                    if user_input.lower() in ['quit', 'q', 'exit']:
                        self.get_logger().info('Exiting interactive mode...')
                        rclpy.shutdown()
                        break
                    
                    if user_input.lower() == 'cancel':
                        self.cancel_goal()
                        continue
                    
                    if not user_input:
                        continue
                    
                    parts = user_input.split()
                    if len(parts) == 2:
                        x, y = float(parts[0]), float(parts[1])
                        yaw_deg = 0.0
                    elif len(parts) == 3:
                        x, y, yaw_deg = float(parts[0]), float(parts[1]), float(parts[2])
                    else:
                        self.get_logger().error('Invalid format. Use: x y [yaw_deg]')
                        continue
                    
                    yaw_rad = math.radians(yaw_deg)
                    self.get_logger().info(f'Sending goal: ({x}, {y}) yaw={yaw_deg}°')
                    self.send_goal(x, y, 0.0, yaw_rad)
                    
                except ValueError:
                    self.get_logger().error('Invalid numbers. Use: x y [yaw_deg]')
                except EOFError:
                    break
                except KeyboardInterrupt:
                    break
        
        # Run input in separate thread to not block ROS2 callbacks
        thread = threading.Thread(target=input_thread, daemon=True)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    
    goal_sender = GoalSender()
    
    try:
        rclpy.spin(goal_sender)
    except KeyboardInterrupt:
        pass
    finally:
        goal_sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
