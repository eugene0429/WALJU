#!/usr/bin/env python3
"""
Ground Truth Point Cloud Publisher

Publishes simulation ground truth terrain as a PointCloud2 topic
for visual comparison with nvblox mapping output.

Usage:
    ros2 run nvblox_integration ground_truth_publisher.py
    ros2 run nvblox_integration ground_truth_publisher.py --ros-args -p pointcloud_file:=/path/to/file.ply
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import os
from pathlib import Path


class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')
        
        # Parameters
        self.declare_parameter('pointcloud_file', '/shared_data/dem/sim_pointcloud_latest.ply')
        self.declare_parameter('npy_file', '/shared_data/dem/sim_pointcloud_latest.npy')
        self.declare_parameter('use_ply', True)  # True for .ply, False for .npy
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)  # Hz (low rate since it's static)
        self.declare_parameter('point_color_r', 0)  # Green color for ground truth
        self.declare_parameter('point_color_g', 255)
        self.declare_parameter('point_color_b', 0)
        self.declare_parameter('alpha', 64)  # Semi-transparent
        self.declare_parameter('auto_reload', True)  # Reload if file changes
        self.declare_parameter('z_offset', 0.0)  # Vertical offset for alignment
        
        # Get parameters
        self.pointcloud_file = self.get_parameter('pointcloud_file').value
        self.npy_file = self.get_parameter('npy_file').value
        self.use_ply = self.get_parameter('use_ply').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.color_r = self.get_parameter('point_color_r').value
        self.color_g = self.get_parameter('point_color_g').value
        self.color_b = self.get_parameter('point_color_b').value
        self.alpha = self.get_parameter('alpha').value
        self.auto_reload = self.get_parameter('auto_reload').value
        self.z_offset = self.get_parameter('z_offset').value
        
        # Publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            '/ground_truth/pointcloud',
            10
        )
        
        # State
        self.points = None
        self.last_mtime = None
        self.pointcloud_msg = None
        
        # Load initial point cloud
        self.load_pointcloud()
        
        # Timer for publishing
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_callback)
        
        self.get_logger().info(f'Ground truth publisher started')
        self.get_logger().info(f'  File: {self.pointcloud_file if self.use_ply else self.npy_file}')
        self.get_logger().info(f'  Frame: {self.frame_id}')
        self.get_logger().info(f'  Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Color: RGB({self.color_r}, {self.color_g}, {self.color_b})')
    
    def load_pointcloud(self):
        """Load point cloud from file."""
        try:
            if self.use_ply:
                self.points = self.load_ply(self.pointcloud_file)
            else:
                self.points = self.load_npy(self.npy_file)
            
            if self.points is not None:
                # Apply Z offset if specified
                if self.z_offset != 0.0:
                    self.points[:, 2] += self.z_offset
                
                self.pointcloud_msg = self.create_pointcloud2_msg(self.points)
                self.get_logger().info(f'Loaded {len(self.points)} points')
                
                # Update mtime for auto-reload
                filepath = self.pointcloud_file if self.use_ply else self.npy_file
                if os.path.exists(filepath):
                    self.last_mtime = os.path.getmtime(filepath)
        except Exception as e:
            self.get_logger().error(f'Failed to load point cloud: {e}')
    
    def load_ply(self, filepath: str) -> np.ndarray:
        """Load PLY file and return Nx3 numpy array of points."""
        if not os.path.exists(filepath):
            self.get_logger().warn(f'PLY file not found: {filepath}')
            return None
        
        try:
            # Try using open3d if available
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(filepath)
            points = np.asarray(pcd.points)
            self.get_logger().info(f'Loaded PLY using Open3D: {points.shape}')
            return points.astype(np.float32)
        except ImportError:
            # Fallback to manual PLY parsing
            return self.parse_ply_manual(filepath)
    
    def parse_ply_manual(self, filepath: str) -> np.ndarray:
        """Manually parse PLY file without Open3D."""
        points = []
        header_ended = False
        vertex_count = 0
        is_binary = False
        
        with open(filepath, 'rb') as f:
            # Read header
            while True:
                line = f.readline().decode('ascii').strip()
                if line.startswith('element vertex'):
                    vertex_count = int(line.split()[-1])
                elif line.startswith('format binary'):
                    is_binary = True
                elif line == 'end_header':
                    break
            
            if is_binary:
                # Read binary data (assuming float32 x, y, z)
                for _ in range(vertex_count):
                    data = f.read(12)  # 3 floats * 4 bytes
                    if len(data) == 12:
                        x, y, z = struct.unpack('fff', data)
                        points.append([x, y, z])
            else:
                # Read ASCII data
                for _ in range(vertex_count):
                    line = f.readline().decode('ascii').strip()
                    if line:
                        parts = line.split()
                        if len(parts) >= 3:
                            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
        
        return np.array(points, dtype=np.float32) if points else None
    
    def load_npy(self, filepath: str) -> np.ndarray:
        """Load NPY file and return Nx3 numpy array of points."""
        if not os.path.exists(filepath):
            self.get_logger().warn(f'NPY file not found: {filepath}')
            return None
        
        data = np.load(filepath)
        
        # Handle different formats
        if data.ndim == 2 and data.shape[1] >= 3:
            return data[:, :3].astype(np.float32)
        else:
            self.get_logger().error(f'Unexpected NPY format: {data.shape}')
            return None
    
    def create_pointcloud2_msg(self, points: np.ndarray) -> PointCloud2:
        """Create PointCloud2 message with XYZRGB format."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = self.frame_id
        
        # Define fields (XYZRGB)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16  # 4 floats * 4 bytes
        msg.height = 1
        msg.width = len(points)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack RGB into float
        rgb_packed = struct.unpack('f', struct.pack('BBBB', 
            self.color_b, self.color_g, self.color_r, self.alpha))[0]
        
        # Create data buffer
        data = []
        for p in points:
            data.extend(struct.pack('ffff', p[0], p[1], p[2], rgb_packed))
        
        msg.data = bytes(data)
        return msg
    
    def check_file_changed(self) -> bool:
        """Check if source file has been modified."""
        filepath = self.pointcloud_file if self.use_ply else self.npy_file
        if os.path.exists(filepath):
            mtime = os.path.getmtime(filepath)
            if self.last_mtime is None or mtime > self.last_mtime:
                return True
        return False
    
    def publish_callback(self):
        """Timer callback to publish point cloud."""
        # Auto-reload if file changed
        if self.auto_reload and self.check_file_changed():
            self.get_logger().info('File changed, reloading...')
            self.load_pointcloud()
        
        if self.pointcloud_msg is not None:
            # Update timestamp
            self.pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.pointcloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
