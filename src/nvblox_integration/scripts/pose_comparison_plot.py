#!/usr/bin/env python3
"""
Real-time Pose Comparison Plot

Compares ground truth base_link pose from /tf_gt with 
estimated base_link pose computed from SLAM camera pose + static TF.

Usage:
    ros2 run nvblox_integration pose_comparison_plot.py
"""

import sys
import threading
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for separate window
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def quaternion_to_yaw(qx, qy, qz, qw):
    """Extract yaw angle from quaternion."""
    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return yaw


def quaternion_multiply(q1, q2):
    """Multiply two quaternions (x, y, z, w format)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def quaternion_rotate_point(q, point):
    """Rotate a point by a quaternion."""
    qx, qy, qz, qw = q
    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ])
    return R @ point


class PoseComparisonNode(Node):
    def __init__(self):
        super().__init__('pose_comparison_plot')
        
        # Parameters
        self.declare_parameter('tf_gt_topic', '/tf_gt')
        self.declare_parameter('tf_gt_child_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('history_length', 500)
        self.declare_parameter('plot_update_rate', 10.0)  # Hz
        
        self.tf_gt_topic = self.get_parameter('tf_gt_topic').value
        self.tf_gt_child_frame = self.get_parameter('tf_gt_child_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.history_length = self.get_parameter('history_length').value
        
        # Data storage (thread-safe with lock)
        self.lock = threading.Lock()
        self.history_len = self.history_length
        
        # Ground truth data
        self.gt_times = deque(maxlen=self.history_len)
        self.gt_x = deque(maxlen=self.history_len)
        self.gt_y = deque(maxlen=self.history_len)
        self.gt_z = deque(maxlen=self.history_len)
        self.gt_yaw = deque(maxlen=self.history_len)
        
        # Estimated data (from SLAM)
        self.est_times = deque(maxlen=self.history_len)
        self.est_x = deque(maxlen=self.history_len)
        self.est_y = deque(maxlen=self.history_len)
        self.est_z = deque(maxlen=self.history_len)
        self.est_yaw = deque(maxlen=self.history_len)
        
        # Static TF: camera_link -> base_link (will be captured from /tf_static)
        self.static_tf_received = False
        self.cam_to_base_translation = np.array([0.0, 0.0, 0.0])
        self.cam_to_base_rotation = np.array([0.0, 0.0, 0.0, 1.0])  # xyzw
        
        # Error metrics
        self.errors_x = deque(maxlen=self.history_len)
        self.errors_y = deque(maxlen=self.history_len)
        self.errors_z = deque(maxlen=self.history_len)
        self.errors_yaw = deque(maxlen=self.history_len)
        
        # QoS for TF topics
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=100
        )
        
        static_tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=100
        )
        
        # Subscribers
        self.tf_gt_sub = self.create_subscription(
            TFMessage, self.tf_gt_topic, self.tf_gt_callback, tf_qos)
        
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, tf_qos)
        
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback, static_tf_qos)
        
        self.get_logger().info(f'Pose comparison plot started')
        self.get_logger().info(f'  GT topic: {self.tf_gt_topic} (child: {self.tf_gt_child_frame})')
        self.get_logger().info(f'  Camera frame: {self.camera_frame}')
        
        # Start time reference
        self.start_time = None
    
    def tf_gt_callback(self, msg: TFMessage):
        """Handle ground truth TF messages."""
        for transform in msg.transforms:
            if transform.child_frame_id == self.tf_gt_child_frame:
                t = transform.transform.translation
                r = transform.transform.rotation
                
                # Get time
                stamp = transform.header.stamp
                time_sec = stamp.sec + stamp.nanosec * 1e-9
                
                if self.start_time is None:
                    self.start_time = time_sec
                
                rel_time = time_sec - self.start_time
                yaw = quaternion_to_yaw(r.x, r.y, r.z, r.w)
                
                with self.lock:
                    self.gt_times.append(rel_time)
                    self.gt_x.append(t.x)
                    self.gt_y.append(t.y)
                    self.gt_z.append(t.z)
                    self.gt_yaw.append(np.degrees(yaw))
    
    def tf_static_callback(self, msg: TFMessage):
        """Capture static TF for camera_link -> base_link."""
        for transform in msg.transforms:
            if (transform.header.frame_id == self.camera_frame and 
                transform.child_frame_id == 'base_link'):
                t = transform.transform.translation
                r = transform.transform.rotation
                
                self.cam_to_base_translation = np.array([t.x, t.y, t.z])
                self.cam_to_base_rotation = np.array([r.x, r.y, r.z, r.w])
                self.static_tf_received = True
                
                self.get_logger().info(
                    f'Captured static TF {self.camera_frame} -> base_link: '
                    f't=[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]')
    
    def tf_callback(self, msg: TFMessage):
        """Handle SLAM TF messages (map -> camera_link)."""
        for transform in msg.transforms:
            if (transform.header.frame_id == 'map' and 
                transform.child_frame_id == self.camera_frame):
                
                if not self.static_tf_received:
                    return
                
                t = transform.transform.translation
                r = transform.transform.rotation
                
                # Camera pose in map frame
                cam_pos = np.array([t.x, t.y, t.z])
                cam_quat = np.array([r.x, r.y, r.z, r.w])
                
                # Apply static TF to get base_link pose
                # base_pos = cam_pos + R_cam * cam_to_base_translation
                rotated_offset = quaternion_rotate_point(cam_quat, self.cam_to_base_translation)
                base_pos = cam_pos + rotated_offset
                
                # base_quat = cam_quat * cam_to_base_rotation
                base_quat = quaternion_multiply(cam_quat, self.cam_to_base_rotation)
                
                # Get time
                stamp = transform.header.stamp
                time_sec = stamp.sec + stamp.nanosec * 1e-9
                
                if self.start_time is None:
                    self.start_time = time_sec
                
                rel_time = time_sec - self.start_time
                yaw = quaternion_to_yaw(base_quat[0], base_quat[1], base_quat[2], base_quat[3])
                
                with self.lock:
                    self.est_times.append(rel_time)
                    self.est_x.append(base_pos[0])
                    self.est_y.append(base_pos[1])
                    self.est_z.append(base_pos[2])
                    self.est_yaw.append(np.degrees(yaw))
                    
                    # Calculate error if we have GT data
                    if len(self.gt_x) > 0:
                        # Use latest GT for error calculation
                        self.errors_x.append(base_pos[0] - self.gt_x[-1])
                        self.errors_y.append(base_pos[1] - self.gt_y[-1])
                        self.errors_z.append(base_pos[2] - self.gt_z[-1])
                        
                        # Yaw error (handle wrap-around)
                        yaw_err = np.degrees(yaw) - self.gt_yaw[-1]
                        if yaw_err > 180:
                            yaw_err -= 360
                        elif yaw_err < -180:
                            yaw_err += 360
                        self.errors_yaw.append(yaw_err)
    
    def get_plot_data(self):
        """Get thread-safe copy of data for plotting."""
        with self.lock:
            return {
                'gt_times': list(self.gt_times),
                'gt_x': list(self.gt_x),
                'gt_y': list(self.gt_y),
                'gt_z': list(self.gt_z),
                'gt_yaw': list(self.gt_yaw),
                'est_times': list(self.est_times),
                'est_x': list(self.est_x),
                'est_y': list(self.est_y),
                'est_z': list(self.est_z),
                'est_yaw': list(self.est_yaw),
                'errors_x': list(self.errors_x),
                'errors_y': list(self.errors_y),
                'errors_z': list(self.errors_z),
                'errors_yaw': list(self.errors_yaw),
            }


def run_plot(node: PoseComparisonNode):
    """Run the matplotlib plot in the main thread."""
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('Pose Comparison: Ground Truth vs SLAM Estimate', fontsize=14)
    
    # Subplot layout:
    # [0,0] X position      [0,1] Y position
    # [1,0] Z position      [1,1] Yaw angle
    # [2,0] XY trajectory   [2,1] Error metrics
    
    ax_x, ax_y = axes[0]
    ax_z, ax_yaw = axes[1]
    ax_traj, ax_err = axes[2]
    
    # Initialize empty lines
    # Position plots
    line_gt_x, = ax_x.plot([], [], 'g-', label='GT', linewidth=1.5)
    line_est_x, = ax_x.plot([], [], 'r--', label='SLAM', linewidth=1.5)
    ax_x.set_ylabel('X (m)')
    ax_x.set_title('X Position')
    ax_x.legend(loc='upper right')
    ax_x.grid(True, alpha=0.3)
    
    line_gt_y, = ax_y.plot([], [], 'g-', label='GT', linewidth=1.5)
    line_est_y, = ax_y.plot([], [], 'r--', label='SLAM', linewidth=1.5)
    ax_y.set_ylabel('Y (m)')
    ax_y.set_title('Y Position')
    ax_y.legend(loc='upper right')
    ax_y.grid(True, alpha=0.3)
    
    line_gt_z, = ax_z.plot([], [], 'g-', label='GT', linewidth=1.5)
    line_est_z, = ax_z.plot([], [], 'r--', label='SLAM', linewidth=1.5)
    ax_z.set_ylabel('Z (m)')
    ax_z.set_xlabel('Time (s)')
    ax_z.set_title('Z Position')
    ax_z.legend(loc='upper right')
    ax_z.grid(True, alpha=0.3)
    
    line_gt_yaw, = ax_yaw.plot([], [], 'g-', label='GT', linewidth=1.5)
    line_est_yaw, = ax_yaw.plot([], [], 'r--', label='SLAM', linewidth=1.5)
    ax_yaw.set_ylabel('Yaw (deg)')
    ax_yaw.set_xlabel('Time (s)')
    ax_yaw.set_title('Yaw Angle')
    ax_yaw.legend(loc='upper right')
    ax_yaw.grid(True, alpha=0.3)
    
    # XY trajectory plot
    line_gt_traj, = ax_traj.plot([], [], 'g-', label='GT', linewidth=1.5)
    line_est_traj, = ax_traj.plot([], [], 'r--', label='SLAM', linewidth=1.5)
    point_gt, = ax_traj.plot([], [], 'go', markersize=8)
    point_est, = ax_traj.plot([], [], 'rs', markersize=8)
    ax_traj.set_xlabel('X (m)')
    ax_traj.set_ylabel('Y (m)')
    ax_traj.set_title('XY Trajectory')
    ax_traj.legend(loc='upper right')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_aspect('equal', adjustable='box')
    
    # Error metrics plot
    line_err_x, = ax_err.plot([], [], 'r-', label='X err', linewidth=1)
    line_err_y, = ax_err.plot([], [], 'g-', label='Y err', linewidth=1)
    line_err_z, = ax_err.plot([], [], 'b-', label='Z err', linewidth=1)
    ax_err.set_ylabel('Position Error (m)')
    ax_err.set_xlabel('Time (s)')
    ax_err.set_title('Position Error')
    ax_err.legend(loc='upper right')
    ax_err.grid(True, alpha=0.3)
    ax_err.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    
    # Text annotations for statistics
    stats_text = ax_err.text(0.02, 0.98, '', transform=ax_err.transAxes, 
                              fontsize=9, verticalalignment='top',
                              fontfamily='monospace',
                              bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    def update(frame):
        """Update function for animation."""
        data = node.get_plot_data()
        
        gt_times = data['gt_times']
        est_times = data['est_times']
        
        if len(gt_times) < 2 and len(est_times) < 2:
            return []
        
        # Update position time series
        if gt_times:
            line_gt_x.set_data(gt_times, data['gt_x'])
            line_gt_y.set_data(gt_times, data['gt_y'])
            line_gt_z.set_data(gt_times, data['gt_z'])
            line_gt_yaw.set_data(gt_times, data['gt_yaw'])
            line_gt_traj.set_data(data['gt_x'], data['gt_y'])
            point_gt.set_data([data['gt_x'][-1]], [data['gt_y'][-1]])
        
        if est_times:
            line_est_x.set_data(est_times, data['est_x'])
            line_est_y.set_data(est_times, data['est_y'])
            line_est_z.set_data(est_times, data['est_z'])
            line_est_yaw.set_data(est_times, data['est_yaw'])
            line_est_traj.set_data(data['est_x'], data['est_y'])
            point_est.set_data([data['est_x'][-1]], [data['est_y'][-1]])
        
        # Update error plot
        if data['errors_x']:
            err_times = est_times[-len(data['errors_x']):]
            line_err_x.set_data(err_times, data['errors_x'])
            line_err_y.set_data(err_times, data['errors_y'])
            line_err_z.set_data(err_times, data['errors_z'])
            
            # Calculate RMS errors
            rmse_x = np.sqrt(np.mean(np.array(data['errors_x'])**2))
            rmse_y = np.sqrt(np.mean(np.array(data['errors_y'])**2))
            rmse_z = np.sqrt(np.mean(np.array(data['errors_z'])**2))
            rmse_3d = np.sqrt(rmse_x**2 + rmse_y**2 + rmse_z**2)
            rmse_yaw = np.sqrt(np.mean(np.array(data['errors_yaw'])**2)) if data['errors_yaw'] else 0
            
            stats_str = (f'RMSE X:   {rmse_x:.3f} m\n'
                        f'RMSE Y:   {rmse_y:.3f} m\n'
                        f'RMSE Z:   {rmse_z:.3f} m\n'
                        f'RMSE 3D:  {rmse_3d:.3f} m\n'
                        f'RMSE Yaw: {rmse_yaw:.2f}°')
            stats_text.set_text(stats_str)
        
        # Auto-scale axes
        all_times = gt_times + est_times
        if all_times:
            t_min, t_max = min(all_times), max(all_times)
            t_margin = max(1.0, (t_max - t_min) * 0.05)
            
            for ax in [ax_x, ax_y, ax_z, ax_yaw, ax_err]:
                ax.set_xlim(t_min - t_margin, t_max + t_margin)
        
        # Auto-scale Y axes
        def auto_scale_y(ax, *data_lists):
            all_data = []
            for d in data_lists:
                all_data.extend(d)
            if all_data:
                y_min, y_max = min(all_data), max(all_data)
                margin = max(0.1, (y_max - y_min) * 0.1)
                ax.set_ylim(y_min - margin, y_max + margin)
        
        auto_scale_y(ax_x, data['gt_x'], data['est_x'])
        auto_scale_y(ax_y, data['gt_y'], data['est_y'])
        auto_scale_y(ax_z, data['gt_z'], data['est_z'])
        auto_scale_y(ax_yaw, data['gt_yaw'], data['est_yaw'])
        
        if data['errors_x']:
            all_errors = data['errors_x'] + data['errors_y'] + data['errors_z']
            auto_scale_y(ax_err, all_errors)
        
        # Auto-scale trajectory (equal aspect handles the rest)
        all_x = data['gt_x'] + data['est_x']
        all_y = data['gt_y'] + data['est_y']
        if all_x and all_y:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            # Calculate equal range for both axes
            x_range = x_max - x_min
            y_range = y_max - y_min
            max_range = max(x_range, y_range, 2.0)  # at least 2m range
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            margin = max_range * 0.6  # 20% margin on each side
            ax_traj.set_xlim(x_center - margin, x_center + margin)
            ax_traj.set_ylim(y_center - margin, y_center + margin)
        
        return [line_gt_x, line_est_x, line_gt_y, line_est_y, 
                line_gt_z, line_est_z, line_gt_yaw, line_est_yaw,
                line_gt_traj, line_est_traj, point_gt, point_est,
                line_err_x, line_err_y, line_err_z, stats_text]
    
    # Create animation
    ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    
    plt.show()


def main():
    rclpy.init()
    
    node = PoseComparisonNode()
    
    # Run ROS2 spin in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # Run matplotlib in main thread (required by some backends)
        run_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
