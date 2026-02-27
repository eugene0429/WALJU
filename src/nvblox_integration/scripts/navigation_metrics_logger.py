#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Navigation Metrics Logger

Logs per-run navigation metrics to a single CSV file, accumulating across runs.
For each navigation goal (start → goal reached), records:
  - Localization error (mean absolute error per axis: X, Y, Z)
  - Depth estimation error (mean abs_rel, rmse, mae)
  - Elapsed time (seconds)
  - Actual distance traveled (meters)

Detection methods (works with or without goal_sender):
  1. /nav_status topic  — from goal_sender (SENDING / ACCEPTED / SUCCEEDED …)
  2. /goal_pose topic   — detect goal from RViz 2D Nav Goal
  3. navigate_to_pose/_action/status — detect Nav2 action state changes

Subscribed Topics:
  /nav_status                             (std_msgs/String)
  /goal_pose                              (geometry_msgs/PoseStamped)
  /navigate_to_pose/_action/status        (action_msgs/GoalStatusArray)
  /tf_gt                                  (tf2_msgs/TFMessage)
  /tf                                     (tf2_msgs/TFMessage)
  /tf_static                              (tf2_msgs/TFMessage)
  /depth/metrics                          (std_msgs/String)

Parameters:
  output_file       (str)  — CSV file path
  tf_gt_topic       (str)  — GT TF topic name (default: /tf_gt)
  tf_gt_child_frame (str)  — GT child frame (default: base_link)
  camera_frame      (str)  — SLAM camera frame (default: camera_link)
"""

import os
import csv
import json
import math
import threading
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray, GoalStatus
from tf2_msgs.msg import TFMessage


def quaternion_to_yaw(qx, qy, qz, qw):
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_rotate_point(qx, qy, qz, qw, point):
    """Rotate a 3D point by a quaternion."""
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ])
    return R @ np.array(point)


def quaternion_multiply(q1, q2):
    """Multiply two quaternions in (x,y,z,w) format."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


# ── CSV column definitions ──────────────────────────────────────────────
CSV_HEADER = [
    'run_id',
    'timestamp',
    'result',
    'goal_x', 'goal_y', 'goal_yaw_deg',
    'duration_sec',
    'distance_traveled_m',
    'loc_mean_abs_err_x_m', 'loc_mean_abs_err_y_m', 'loc_mean_abs_err_z_m',
    'loc_rmse_x_m', 'loc_rmse_y_m', 'loc_rmse_z_m',
    'loc_rmse_3d_m',
    'depth_mean_abs_rel', 'depth_mean_rmse_m', 'depth_mean_mae_m',
    'depth_mean_a1',
    'num_pose_samples', 'num_depth_samples',
]


class NavigationMetricsLogger(Node):
    """Accumulates per-run navigation metrics into a CSV file."""

    def __init__(self):
        super().__init__('navigation_metrics_logger')

        # ── Parameters ───────────────────────────────────────────────────
        default_csv = os.path.join(
            os.environ.get('ROS2_WS', os.path.expanduser('~/WALJU')),
            'navigation_metrics.csv',
        )
        self.declare_parameter('output_file', default_csv)
        self.declare_parameter('tf_gt_topic', '/tf_gt')
        self.declare_parameter('tf_gt_child_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        # Note: use_sim_time is auto-declared by ROS2 Jazzy; don't re-declare

        self.output_file = self.get_parameter('output_file').value
        self.tf_gt_topic = self.get_parameter('tf_gt_topic').value
        self.tf_gt_child_frame = self.get_parameter('tf_gt_child_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # ── State ────────────────────────────────────────────────────────
        self.lock = threading.Lock()
        self._navigating = False
        self._nav_status_available = False   # True if /nav_status topic is active

        # Static TF: camera_link → base_link
        self._static_tf_received = False
        self._cam_to_base_t = np.zeros(3)
        self._cam_to_base_q = np.array([0.0, 0.0, 0.0, 1.0])

        # Track Nav2 action goal IDs that have been finalized
        self._known_goal_ids = set()
        self._active_goal_id = None

        # Goal pose — persistent across runs, updated only by callbacks
        self._goal_pose = None          # (x, y, yaw_deg)

        # Per-run accumulators (reset on each navigation start)
        self._reset_accumulators()

        # Figure out next run_id from existing CSV
        self._next_run_id = self._get_next_run_id()

        # ── QoS profiles ─────────────────────────────────────────────────
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=100,
        )
        static_tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=100,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Subscribers ──────────────────────────────────────────────────

        # Method 1: /nav_status from goal_sender (if running)
        self.create_subscription(
            String, '/nav_status', self._nav_status_cb, reliable_qos)

        # Method 2: /goal_pose — detect goal from RViz or goal_pose_corrector
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_pose_cb, reliable_qos)

        # Method 3: Nav2 action status — detect ACCEPTED / SUCCEEDED / ABORTED
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._action_status_cb,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ))

        # TF subscribers
        self.create_subscription(
            TFMessage, self.tf_gt_topic, self._tf_gt_cb, tf_qos)
        self.create_subscription(
            TFMessage, '/tf', self._tf_cb, tf_qos)
        self.create_subscription(
            TFMessage, '/tf_static', self._tf_static_cb, static_tf_qos)

        # Depth metrics
        self.create_subscription(
            String, '/depth/metrics', self._depth_metrics_cb, reliable_qos)

        self.get_logger().info(f'Navigation Metrics Logger started')
        self.get_logger().info(f'  Output file : {self.output_file}')
        self.get_logger().info(f'  GT topic    : {self.tf_gt_topic} (child={self.tf_gt_child_frame})')
        self.get_logger().info(f'  Camera frame: {self.camera_frame}')
        self.get_logger().info(f'  Next run_id : {self._next_run_id}')

    # ─── helpers ─────────────────────────────────────────────────────────
    def _reset_accumulators(self):
        """Clear per-run data. Note: _goal_pose is NOT reset here."""
        self._start_time = None

        # Localization errors (GT - estimated)
        self._loc_errors_x = []
        self._loc_errors_y = []
        self._loc_errors_z = []

        # Latest GT pose for error computation
        self._latest_gt_pos = None      # np.array([x, y, z])

        # Odometry for distance tracking
        self._prev_gt_pos = None
        self._distance_traveled = 0.0

        # Depth metrics per frame
        self._depth_abs_rel_list = []
        self._depth_rmse_list = []
        self._depth_mae_list = []
        self._depth_a1_list = []

    def _get_next_run_id(self) -> int:
        """Read existing CSV and return the next sequential run_id."""
        if not os.path.isfile(self.output_file):
            return 1
        try:
            with open(self.output_file, 'r') as f:
                reader = csv.DictReader(f)
                max_id = 0
                for row in reader:
                    try:
                        max_id = max(max_id, int(row['run_id']))
                    except (ValueError, KeyError):
                        pass
                return max_id + 1
        except Exception:
            return 1

    def _now_sec(self) -> float:
        """Current time as float seconds."""
        t = self.get_clock().now().nanoseconds
        return t * 1e-9

    def _start_navigation(self):
        """Begin a new navigation run (must be called with lock held)."""
        if self._navigating:
            # Already in progress (e.g. action_status replay already started it).
            # Don't reset accumulators; just let the goal be updated externally.
            self.get_logger().debug(
                '[Metrics] Navigation already in progress, skipping restart')
            return
        self._reset_accumulators()
        self._navigating = True
        self._start_time = self._now_sec()
        self.get_logger().info('[Metrics] ▶ Navigation started — collecting metrics')

    def _stop_navigation(self, result: str):
        """Finish a navigation run (must be called with lock held)."""
        if not self._navigating:
            return
        self._navigating = False
        self._finalize_run(result)

    # ─── helpers — goal parsing ──────────────────────────────────────────
    @staticmethod
    def _parse_goal_from_status(status: str):
        """Extract (x, y, yaw_deg) from a status string like
        'ACCEPTED: (20.00, -6.00) yaw=-16.7°'. Returns None on failure."""
        try:
            if '(' not in status or ')' not in status or 'yaw=' not in status:
                return None
            inner = status.split('(')[1].split(')')[0]
            parts = inner.split(',')
            gx = float(parts[0].strip())
            gy = float(parts[1].strip())
            yaw_str = status.split('yaw=')[1].replace('°', '').strip()
            gyaw = float(yaw_str)
            return (gx, gy, gyaw)
        except (IndexError, ValueError):
            return None

    # ─── /goal_pose callback (RViz / goal_pose_corrector) ────────────────
    def _goal_pose_cb(self, msg: PoseStamped):
        """Capture goal pose from /goal_pose topic (works without goal_sender)."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q = msg.pose.orientation
        gyaw = math.degrees(quaternion_to_yaw(q.x, q.y, q.z, q.w))

        with self.lock:
            self._goal_pose = (gx, gy, gyaw)
        self.get_logger().info(
            f'[Metrics] Goal detected from /goal_pose: ({gx:.2f}, {gy:.2f}) yaw={gyaw:.1f}°')

    # ─── /nav_status callback (from goal_sender, if running) ─────────────
    def _nav_status_cb(self, msg: String):
        status = msg.data.strip()
        self._nav_status_available = True

        # Try to extract goal coordinates from any status message.
        # Format: "<STATUS>: (x, y) yaw=deg°"  (SENDING, ACCEPTED, SUCCEEDED …)
        goal_parsed = self._parse_goal_from_status(status)

        if status.startswith('SENDING'):
            if goal_parsed:
                with self.lock:
                    self._goal_pose = goal_parsed
                self.get_logger().info(
                    f'[Metrics] Goal from /nav_status: '
                    f'({goal_parsed[0]:.2f}, {goal_parsed[1]:.2f}) '
                    f'yaw={goal_parsed[2]:.1f}°')

        elif status.startswith('ACCEPTED'):
            with self.lock:
                if goal_parsed:
                    self._goal_pose = goal_parsed
                self._start_navigation()

        elif (status.startswith('SUCCEEDED') or
              status.startswith('ABORTED') or
              status.startswith('CANCELED')):
            with self.lock:
                # Last chance to capture goal before finalizing
                if goal_parsed and self._goal_pose is None:
                    self._goal_pose = goal_parsed
                result = status.split(':')[0].strip()
                self._stop_navigation(result)

    # ─── Nav2 action status callback (primary detection method) ──────────
    def _action_status_cb(self, msg: GoalStatusArray):
        """Monitor navigate_to_pose action status to detect start/end of navigation.

        This works regardless of whether goal_sender is running.
        Status codes:
          STATUS_ACCEPTED  = 1
          STATUS_EXECUTING = 2
          STATUS_SUCCEEDED = 4
          STATUS_CANCELED  = 5
          STATUS_ABORTED   = 6
        """
        if not msg.status_list:
            return

        # Look at the latest goal status
        latest = msg.status_list[-1]
        goal_id = bytes(latest.goal_info.goal_id.uuid).hex()
        status = latest.status

        with self.lock:
            # If /nav_status is active (goal_sender running), let it handle start/stop
            # to avoid duplicate triggers. But we still use action status as fallback.
            if self._nav_status_available:
                return

            if status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                if goal_id != self._active_goal_id:
                    # New goal accepted
                    self._active_goal_id = goal_id
                    if goal_id not in self._known_goal_ids:
                        self._start_navigation()

            elif status == GoalStatus.STATUS_SUCCEEDED:
                if goal_id not in self._known_goal_ids:
                    self._known_goal_ids.add(goal_id)
                    self._active_goal_id = None
                    self._stop_navigation('SUCCEEDED')

            elif status == GoalStatus.STATUS_CANCELED:
                if goal_id not in self._known_goal_ids:
                    self._known_goal_ids.add(goal_id)
                    self._active_goal_id = None
                    self._stop_navigation('CANCELED')

            elif status == GoalStatus.STATUS_ABORTED:
                if goal_id not in self._known_goal_ids:
                    self._known_goal_ids.add(goal_id)
                    self._active_goal_id = None
                    self._stop_navigation('ABORTED')

    # ─── /tf_gt callback ────────────────────────────────────────────────
    def _tf_gt_cb(self, msg: TFMessage):
        for tf in msg.transforms:
            if tf.child_frame_id != self.tf_gt_child_frame:
                continue
            t = tf.transform.translation
            gt_pos = np.array([t.x, t.y, t.z])

            with self.lock:
                self._latest_gt_pos = gt_pos

                if not self._navigating:
                    return

                # Accumulate distance traveled
                if self._prev_gt_pos is not None:
                    d = np.linalg.norm(gt_pos[:2] - self._prev_gt_pos[:2])
                    if d < 2.0:  # Sanity: ignore teleports
                        self._distance_traveled += d
                self._prev_gt_pos = gt_pos.copy()

    # ─── /tf_static callback ────────────────────────────────────────────
    def _tf_static_cb(self, msg: TFMessage):
        for tf in msg.transforms:
            if (tf.header.frame_id == self.camera_frame and
                    tf.child_frame_id == 'base_link'):
                t = tf.transform.translation
                r = tf.transform.rotation
                self._cam_to_base_t = np.array([t.x, t.y, t.z])
                self._cam_to_base_q = np.array([r.x, r.y, r.z, r.w])
                self._static_tf_received = True
                self.get_logger().info(
                    f'[Metrics] Static TF captured: {self.camera_frame} → base_link')

    # ─── /tf callback (SLAM estimated pose) ─────────────────────────────
    def _tf_cb(self, msg: TFMessage):
        if not self._static_tf_received:
            return

        for tf in msg.transforms:
            if not (tf.header.frame_id == 'map' and
                    tf.child_frame_id == self.camera_frame):
                continue

            t = tf.transform.translation
            r = tf.transform.rotation
            cam_pos = np.array([t.x, t.y, t.z])
            cam_q = np.array([r.x, r.y, r.z, r.w])

            # Transform camera_link → base_link
            offset = quaternion_rotate_point(
                cam_q[0], cam_q[1], cam_q[2], cam_q[3],
                self._cam_to_base_t)
            est_pos = cam_pos + offset

            with self.lock:
                if not self._navigating:
                    return
                if self._latest_gt_pos is None:
                    return

                err = est_pos - self._latest_gt_pos
                self._loc_errors_x.append(err[0])
                self._loc_errors_y.append(err[1])
                self._loc_errors_z.append(err[2])

    # ─── /depth/metrics callback ─────────────────────────────────────────
    def _depth_metrics_cb(self, msg: String):
        with self.lock:
            if not self._navigating:
                return
        try:
            m = json.loads(msg.data)
            with self.lock:
                if 'abs_rel' in m:
                    self._depth_abs_rel_list.append(m['abs_rel'])
                if 'rmse' in m:
                    self._depth_rmse_list.append(m['rmse'])
                if 'mae' in m:
                    self._depth_mae_list.append(m['mae'])
                if 'a1' in m:
                    self._depth_a1_list.append(m['a1'])
        except json.JSONDecodeError:
            pass

    # ─── Finalize & write CSV row ────────────────────────────────────────
    def _finalize_run(self, result: str):
        """Compute summary statistics and append a row to the CSV. Must be called with lock held."""
        end_time = self._now_sec()
        duration = end_time - self._start_time if self._start_time else 0.0

        n_pose = len(self._loc_errors_x)
        n_depth = len(self._depth_abs_rel_list)

        # Localization metrics
        if n_pose > 0:
            ex = np.array(self._loc_errors_x)
            ey = np.array(self._loc_errors_y)
            ez = np.array(self._loc_errors_z)
            mean_abs_x = float(np.mean(np.abs(ex)))
            mean_abs_y = float(np.mean(np.abs(ey)))
            mean_abs_z = float(np.mean(np.abs(ez)))
            rmse_x = float(np.sqrt(np.mean(ex**2)))
            rmse_y = float(np.sqrt(np.mean(ey**2)))
            rmse_z = float(np.sqrt(np.mean(ez**2)))
            rmse_3d = float(np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2))
        else:
            mean_abs_x = mean_abs_y = mean_abs_z = float('nan')
            rmse_x = rmse_y = rmse_z = rmse_3d = float('nan')

        # Depth metrics
        if n_depth > 0:
            d_abs_rel = float(np.mean(self._depth_abs_rel_list))
            d_rmse = float(np.mean(self._depth_rmse_list))
            d_mae = float(np.mean(self._depth_mae_list))
            d_a1 = float(np.mean(self._depth_a1_list))
        else:
            d_abs_rel = d_rmse = d_mae = d_a1 = float('nan')

        # Goal info
        gx, gy, gyaw = self._goal_pose if self._goal_pose else (float('nan'),)*3

        row = {
            'run_id': self._next_run_id,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'result': result,
            'goal_x': f'{gx:.4f}',
            'goal_y': f'{gy:.4f}',
            'goal_yaw_deg': f'{gyaw:.1f}',
            'duration_sec': f'{duration:.2f}',
            'distance_traveled_m': f'{self._distance_traveled:.4f}',
            'loc_mean_abs_err_x_m': f'{mean_abs_x:.6f}',
            'loc_mean_abs_err_y_m': f'{mean_abs_y:.6f}',
            'loc_mean_abs_err_z_m': f'{mean_abs_z:.6f}',
            'loc_rmse_x_m': f'{rmse_x:.6f}',
            'loc_rmse_y_m': f'{rmse_y:.6f}',
            'loc_rmse_z_m': f'{rmse_z:.6f}',
            'loc_rmse_3d_m': f'{rmse_3d:.6f}',
            'depth_mean_abs_rel': f'{d_abs_rel:.6f}',
            'depth_mean_rmse_m': f'{d_rmse:.6f}',
            'depth_mean_mae_m': f'{d_mae:.6f}',
            'depth_mean_a1': f'{d_a1:.6f}',
            'num_pose_samples': n_pose,
            'num_depth_samples': n_depth,
        }

        self._write_csv_row(row)
        self._log_summary(row, result)
        self._next_run_id += 1

    def _write_csv_row(self, row: dict):
        """Append one row to CSV, creating the file + header if needed."""
        file_exists = os.path.isfile(self.output_file)
        try:
            os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
            with open(self.output_file, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
                if not file_exists:
                    writer.writeheader()
                writer.writerow(row)
            self.get_logger().info(f'[Metrics] Row written → {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'[Metrics] Failed to write CSV: {e}')

    def _log_summary(self, row: dict, result: str):
        """Pretty-print a summary to the ROS log."""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Navigation Run #{row["run_id"]}  —  {result}')
        self.get_logger().info(f'  Goal         : ({row["goal_x"]}, {row["goal_y"]}) yaw={row["goal_yaw_deg"]}°')
        self.get_logger().info(f'  Duration     : {row["duration_sec"]} s')
        self.get_logger().info(f'  Distance     : {row["distance_traveled_m"]} m')
        self.get_logger().info(f'  Loc MAE(X,Y,Z): ({row["loc_mean_abs_err_x_m"]}, '
                               f'{row["loc_mean_abs_err_y_m"]}, {row["loc_mean_abs_err_z_m"]}) m')
        self.get_logger().info(f'  Loc RMSE 3D  : {row["loc_rmse_3d_m"]} m')
        self.get_logger().info(f'  Depth abs_rel: {row["depth_mean_abs_rel"]}')
        self.get_logger().info(f'  Depth RMSE   : {row["depth_mean_rmse_m"]} m')
        self.get_logger().info(f'  Depth MAE    : {row["depth_mean_mae_m"]} m')
        self.get_logger().info(f'  Samples      : pose={row["num_pose_samples"]}, '
                               f'depth={row["num_depth_samples"]}')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationMetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
