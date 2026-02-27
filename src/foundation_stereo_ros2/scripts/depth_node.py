#!/usr/bin/env python3
"""
FoundationStereo Depth Node for ROS2

Subscribes to stereo image pairs, runs FoundationStereo inference,
and publishes depth map.

Subscribed Topics:
  /stereo/left/image_raw  (sensor_msgs/Image)
  /stereo/right/image_raw (sensor_msgs/Image)

Published Topics:
  /depth/image (sensor_msgs/Image, encoding: 32FC1, unit: meters)
  /depth/camera_info (sensor_msgs/CameraInfo) - for nvblox integration

Parameters:
  left_topic: Left image topic name
  right_topic: Right image topic name
  depth_topic: Output depth topic name
  model_path: Path to FoundationStereo model checkpoint
  intrinsic_file: Path to camera intrinsic file (K matrix + baseline)
  scale: Image downscale factor (default: 0.5)
  valid_iters: Number of refinement iterations (default: 32)
  use_hiera: Use hierarchical inference for high-res images (default: False)
"""

import os
import sys
import time
import threading
from collections import deque

import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

# FoundationStereo imports will be done after path is configured
OmegaConf = None
FoundationStereo = None
InputPadder = None


def set_seed(seed):
    """Set random seed for reproducibility"""
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def ensure_3ch(img):
    """Ensure image is 3-channel RGB"""
    if img.ndim == 2:
        return np.stack([img, img, img], axis=2)
    if img.shape[2] == 1:
        return np.concatenate([img, img, img], axis=2)
    if img.shape[2] == 4:
        return img[..., :3]
    return img


class FoundationStereoDepthNode(Node):
    def __init__(self):
        super().__init__('foundation_stereo_depth_node')
        
        # Declare parameters
        self.declare_parameter('foundation_stereo_path', 
            os.path.expanduser('~/WALJU/deps/FoundationStereo'))
        self.declare_parameter('left_topic', '/stereo/left/image_raw')
        self.declare_parameter('right_topic', '/stereo/right/image_raw')
        self.declare_parameter('depth_topic', '/depth/image')
        self.declare_parameter('model_path', '')
        self.declare_parameter('intrinsic_file', 
            os.path.expanduser('~/WALJU/data/LuSNAR/K.txt'))
        self.declare_parameter('scale', 0.5)
        self.declare_parameter('valid_iters', 32)
        self.declare_parameter('use_hiera', False)
        self.declare_parameter('z_far', 20.0)  # Max depth to clip (meters)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('viz_mode', 'cv2')  # 'cv2' for direct OpenCV window, 'topic' for ROS2 topic
        self.declare_parameter('skip_frames_during_processing', True)  # Skip frames or queue them
        self.declare_parameter('camera_frame_id', 'camera_link')  # Frame ID for depth/camera_info
        # Timestamp mode: 'original' keeps input image timestamp, 'current' uses current time
        # Use 'current' when there's significant processing delay causing TF lookup failures
        self.declare_parameter('timestamp_mode', 'current')
        # GT depth evaluation parameters
        self.declare_parameter('gt_depth_topic', '/depth/gt_image')  # GT depth topic (dataset mode)
        self.declare_parameter('sim_gt_depth_topic', '/front_camera/depth/depth')  # Sim GT depth topic
        self.declare_parameter('enable_gt_evaluation', True)  # Enable GT depth comparison
        self.declare_parameter('data_source', 'dataset')  # 'dataset' or 'sim' mode
        # Point cloud visualization parameters
        self.declare_parameter('enable_pc_visualization', False)  # Enable Open3D PC overlay visualization
        self.declare_parameter('pc_downsample_factor', 4)  # Downsample factor for PC visualization (1=full, 4=every 4th pixel)
        # Logging verbosity
        self.declare_parameter('verbose', False)  # Enable verbose logging (startup info, periodic stats)
        # Stereo sync parameters
        self.declare_parameter('sync_slop', 0.033)  # Max timestamp difference for stereo pair (33ms ~ 30fps)
        self.declare_parameter('sync_queue_size', 30)  # Sync buffer size
        self.declare_parameter('max_timestamp_diff_ms', 15.0)  # Warn if timestamp diff exceeds this (ms)
        self.declare_parameter('skip_mismatched_frames', True)  # Skip frames with large timestamp diff
        # Depth filtering parameters
        self.declare_parameter('enable_bilateral_filter', True)  # Apply bilateral filter to depth
        self.declare_parameter('bilateral_d', 5)  # Bilateral filter diameter (5-9 typical)
        self.declare_parameter('bilateral_sigma_color', 50.0)  # Bilateral filter sigma in color space
        self.declare_parameter('bilateral_sigma_space', 50.0)  # Bilateral filter sigma in coordinate space
        
        # Get parameters (use .value for ROS2 Jazzy compatibility)
        self.foundation_stereo_path = self.get_parameter('foundation_stereo_path').value
        self.left_topic = self.get_parameter('left_topic').value
        self.right_topic = self.get_parameter('right_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.model_path = self.get_parameter('model_path').value
        if not self.model_path:
            self.model_path = os.path.join(
                self.foundation_stereo_path, 'pretrained_models', '11-33-40', 'model_best_bp2.pth')
        self.intrinsic_file = self.get_parameter('intrinsic_file').value
        self.scale = self.get_parameter('scale').value
        self.valid_iters = self.get_parameter('valid_iters').value
        self.use_hiera = self.get_parameter('use_hiera').value
        self.z_far = self.get_parameter('z_far').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.viz_mode = self.get_parameter('viz_mode').value  # 'cv2' or 'topic'
        self.skip_frames_during_processing = self.get_parameter('skip_frames_during_processing').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.timestamp_mode = self.get_parameter('timestamp_mode').value  # 'original' or 'current'
        # GT depth evaluation parameters
        self.gt_depth_topic = self.get_parameter('gt_depth_topic').value
        self.sim_gt_depth_topic = self.get_parameter('sim_gt_depth_topic').value
        self.enable_gt_evaluation = self.get_parameter('enable_gt_evaluation').value
        self.data_source = self.get_parameter('data_source').value
        # Point cloud visualization parameters
        self.enable_pc_visualization = self.get_parameter('enable_pc_visualization').value
        self.pc_downsample_factor = self.get_parameter('pc_downsample_factor').value
        # Logging verbosity
        self.verbose = self.get_parameter('verbose').value
        # Stereo sync parameters
        self.sync_slop = self.get_parameter('sync_slop').value
        self.sync_queue_size = self.get_parameter('sync_queue_size').value
        self.max_timestamp_diff_ms = self.get_parameter('max_timestamp_diff_ms').value
        self.skip_mismatched_frames = self.get_parameter('skip_mismatched_frames').value
        # Depth filtering parameters
        self.enable_bilateral_filter = self.get_parameter('enable_bilateral_filter').value
        self.bilateral_d = self.get_parameter('bilateral_d').value
        self.bilateral_sigma_color = self.get_parameter('bilateral_sigma_color').value
        self.bilateral_sigma_space = self.get_parameter('bilateral_sigma_space').value
        
        # Add FoundationStereo to Python path and import
        self._setup_foundation_stereo_imports()
        
        if self.verbose:
            self.get_logger().info(f'FoundationStereo path: {self.foundation_stereo_path}')
            self.get_logger().info(f'Left topic: {self.left_topic}')
            self.get_logger().info(f'Right topic: {self.right_topic}')
            self.get_logger().info(f'Depth topic: {self.depth_topic}')
            self.get_logger().info(f'Model path: {self.model_path}')
            self.get_logger().info(f'Intrinsic file: {self.intrinsic_file}')
            self.get_logger().info(f'Scale: {self.scale}')
            self.get_logger().info(f'Valid iters: {self.valid_iters}')
            self.get_logger().info(f'Timestamp mode: {self.timestamp_mode}')
            self.get_logger().info(f'GT evaluation enabled: {self.enable_gt_evaluation}')
            self.get_logger().info(f'Data source: {self.data_source}')
            self.get_logger().info(f'PC visualization enabled: {self.enable_pc_visualization}')
        
        # Load camera intrinsics and baseline
        self.load_intrinsics()
        
        # Initialize model
        if self.verbose:
            self.get_logger().info('Loading FoundationStereo model...')
        self.load_model()
        if self.verbose:
            self.get_logger().info('Model loaded successfully!')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # QoS profile - use RELIABLE for both dataset and sim (raw image topics are RELIABLE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribers with message synchronization
        self.left_sub = Subscriber(self, Image, self.left_topic, qos_profile=qos)
        self.right_sub = Subscriber(self, Image, self.right_topic, qos_profile=qos)
        
        # ApproximateTimeSynchronizer - use tight slop to ensure matched stereo pairs
        # Large slop (e.g., 0.5s) can cause mismatched frames during fast rotation!
        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop  # Default 33ms - tight sync for accurate depth
        )
        self.sync.registerCallback(self._sync_callback)
        
        if self.verbose:
            self.get_logger().info(f'Stereo sync: slop={self.sync_slop}s, queue={self.sync_queue_size}')
            self.get_logger().info(f'Timestamp validation: max_diff={self.max_timestamp_diff_ms}ms, skip={self.skip_mismatched_frames}')
            self.get_logger().info(f'Bilateral filter: enabled={self.enable_bilateral_filter}, d={self.bilateral_d}')
        
        # GT depth subscription for evaluation
        self._gt_depth_lock = threading.Lock()
        self._latest_gt_depth = None
        self._gt_depth_available = False
        
        if self.enable_gt_evaluation:
            # Select GT depth topic based on data source
            gt_topic = self.gt_depth_topic if self.data_source == 'dataset' else self.sim_gt_depth_topic
            if self.verbose:
                self.get_logger().info(f'Subscribing to GT depth topic: {gt_topic}')
            
            # Use BEST_EFFORT for sim topics (typically from Isaac Sim)
            gt_qos = qos if self.data_source == 'dataset' else QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )
            self.gt_depth_sub = self.create_subscription(
                Image, gt_topic, self._gt_depth_callback, gt_qos)
        
        # Latest synchronized stereo pair (protected by lock)
        self._stereo_lock = threading.Lock()
        self._latest_stereo = None  # (left_msg, right_msg)
        
        # Processing flag
        self.is_processing = False
        
        # Timer to process latest stereo pair when ready
        # This decouples reception from processing, ensuring we always process latest
        self._process_timer = self.create_timer(0.01, self._process_latest_stereo)  # 100Hz check
        
        # Publisher - use lower queue depth for real-time performance
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 1)
        
        # CameraInfo publisher for nvblox integration
        self.camera_info_pub = self.create_publisher(CameraInfo, self.depth_topic.replace('/image', '/camera_info'), 1)
        
        # Color image publisher (re-publish with synchronized timestamp for nvblox)
        self.color_pub = self.create_publisher(Image, '/color/image', 1)
        self.color_info_pub = self.create_publisher(CameraInfo, '/color/camera_info', 1)
        
        # Depth metrics publisher (JSON String) for navigation_metrics_logger
        self.depth_metrics_pub = self.create_publisher(String, '/depth/metrics', 10)
        
        # Visualization publisher (colorized depth as BGR8 image) - use BEST_EFFORT QoS
        viz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.depth_viz_pub = self.create_publisher(Image, self.depth_topic + '/viz', viz_qos)
        
        # Visualization state - all modes use separate thread to avoid blocking inference
        self._viz_lock = threading.Lock()
        self._viz_running = True
        self._latest_viz_frame = None
        self._latest_rgb_frame = None  # Original RGB image for side-by-side viz
        self._latest_gt_depth_frame = None  # GT depth for error visualization
        self._latest_error_map = None  # Error map for visualization
        self._latest_metrics = None  # Depth metrics (abs_rel, etc.)
        self._latest_viz_info = None
        self._viz_thread = None
        
        if self.enable_visualization:
            # Both modes use a separate thread to avoid blocking ROS2 callbacks
            self._viz_thread = threading.Thread(target=self._visualization_thread_worker, daemon=True)
            self._viz_thread.start()
            if self.verbose:
                mode_str = 'OpenCV window' if self.viz_mode == 'cv2' else 'ROS2 topic'
                self.get_logger().info(f'Visualization mode: {mode_str} (separate thread)')
        
        # Point cloud visualization with Open3D (separate thread)
        self._pc_viz_lock = threading.Lock()
        self._pc_viz_running = True
        self._latest_pc_pred_depth = None
        self._latest_pc_gt_depth = None
        self._pc_viz_thread = None
        
        if self.enable_pc_visualization:
            try:
                import open3d as o3d
                self._o3d = o3d
                self._pc_viz_thread = threading.Thread(target=self._pc_visualization_thread_worker, daemon=True)
                self._pc_viz_thread.start()
                if self.verbose:
                    self.get_logger().info('Open3D point cloud visualization enabled (separate thread)')
            except ImportError:
                self.get_logger().warn('Open3D not installed. PC visualization disabled. Install with: pip install open3d')
                self.enable_pc_visualization = False
        
        # Pre-build CameraInfo message template (will be updated with scaled K)
        self.camera_info_msg = None  # Will be initialized after first frame
        
        # Statistics
        self.frame_count = 0
        self.total_inference_time = 0.0
        self._last_callback_time = None  # For measuring actual frame interval
        
        self.get_logger().info('FoundationStereo Depth Node initialized')

    def _setup_foundation_stereo_imports(self):
        """Add FoundationStereo to path and import required modules"""
        global OmegaConf, FoundationStereo, InputPadder
        
        if self.foundation_stereo_path not in sys.path:
            sys.path.insert(0, self.foundation_stereo_path)
        
        from omegaconf import OmegaConf as _OmegaConf
        from core.foundation_stereo import FoundationStereo as _FoundationStereo
        from core.utils.utils import InputPadder as _InputPadder
        
        OmegaConf = _OmegaConf
        FoundationStereo = _FoundationStereo
        InputPadder = _InputPadder

    def load_intrinsics(self):
        """Load camera intrinsic matrix and baseline from file"""
        try:
            with open(self.intrinsic_file, 'r') as f:
                lines = f.readlines()
                # First line: flattened 3x3 intrinsic matrix
                K_flat = list(map(float, lines[0].strip().split()))
                self.K = np.array(K_flat, dtype=np.float32).reshape(3, 3)
                # Second line: baseline in meters
                self.baseline = float(lines[1].strip())
                
            if self.verbose:
                self.get_logger().info(f'Camera intrinsics loaded:')
                self.get_logger().info(f'  K:\n{self.K}')
                self.get_logger().info(f'  Baseline: {self.baseline} m')
        except Exception as e:
            self.get_logger().error(f'Failed to load intrinsics: {e}')
            raise

    def _build_camera_info_msg(self, width, height, stamp):
        """Build CameraInfo message with scaled intrinsics for nvblox
        
        Intrinsic matrix K:
            [fx  0  cx]
            [0  fy  cy]
            [0   0   1]
        
        When scaling image by factor s:
            - fx, fy scale by s (focal length in pixels)
            - cx, cy scale by s (principal point in pixels)
        """
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame_id
        
        msg.width = width
        msg.height = height
        
        # Scale intrinsics for the actual output resolution
        # Original K is for full resolution, we need to scale for resized image
        # fx' = fx * scale, fy' = fy * scale
        # cx' = cx * scale, cy' = cy * scale
        fx = self.K[0, 0] * self.scale
        fy = self.K[1, 1] * self.scale
        cx = self.K[0, 2] * self.scale
        cy = self.K[1, 2] * self.scale
        
        # Intrinsic camera matrix (3x3 row-major)
        # nvblox uses: k[0]=fx, k[4]=fy, k[2]=cx, k[5]=cy
        msg.k = [
            float(fx),  0.0,        float(cx),
            0.0,        float(fy),  float(cy),
            0.0,        0.0,        1.0
        ]
        
        # Distortion model (assuming no distortion for rectified stereo)
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Rectification matrix (identity for rectified images)
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix (3x4) - for monocular/depth: P = [K | 0]
        # Format: [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]
        # Tx, Ty = 0 for left camera (reference camera)
        msg.p = [
            float(fx), 0.0,       float(cx), 0.0,
            0.0,       float(fy), float(cy), 0.0,
            0.0,       0.0,       1.0,       0.0
        ]
        
        # Binning (no binning)
        msg.binning_x = 0
        msg.binning_y = 0
        
        # ROI (full image)
        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi.height = 0
        msg.roi.width = 0
        msg.roi.do_rectify = False
        
        return msg

    def load_model(self):
        """Load FoundationStereo model"""
        set_seed(0)
        torch.autograd.set_grad_enabled(False)
        
        # Load config from model directory
        model_dir = os.path.dirname(self.model_path)
        cfg_path = os.path.join(model_dir, 'cfg.yaml')
        cfg = OmegaConf.load(cfg_path)
        
        if 'vit_size' not in cfg:
            cfg['vit_size'] = 'vitl'
        
        # Override with our parameters
        cfg['scale'] = self.scale
        cfg['valid_iters'] = self.valid_iters
        cfg['hiera'] = self.use_hiera
        
        # Initialize model
        self.model = FoundationStereo(cfg)
        
        # Load weights
        ckpt = torch.load(self.model_path, weights_only=False)
        self.get_logger().info(f"Checkpoint - global_step: {ckpt['global_step']}, epoch: {ckpt['epoch']}")
        self.model.load_state_dict(ckpt['model'])
        
        # Move to GPU and set eval mode
        self.model.cuda()
        self.model.eval()
        
        self.cfg = cfg

    def _sync_callback(self, left_msg: Image, right_msg: Image):
        """Store latest synchronized stereo pair with timestamp validation (non-blocking)"""
        # Calculate timestamp difference between left and right images
        left_stamp = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec * 1e-9
        right_stamp = right_msg.header.stamp.sec + right_msg.header.stamp.nanosec * 1e-9
        timestamp_diff_ms = abs(left_stamp - right_stamp) * 1000.0
        
        # Check for timestamp mismatch (indicates potential sync issue)
        if timestamp_diff_ms > self.max_timestamp_diff_ms:
            self.get_logger().warn(
                f'Stereo timestamp mismatch: {timestamp_diff_ms:.1f}ms '
                f'(left={left_stamp:.3f}, right={right_stamp:.3f}) - '
                f'may cause depth errors during fast motion!'
            )
            if self.skip_mismatched_frames:
                self.get_logger().warn('Skipping mismatched frame pair')
                return
        
        with self._stereo_lock:
            self._latest_stereo = (left_msg, right_msg)

    def _gt_depth_callback(self, msg: Image):
        """Store latest GT depth image (non-blocking)"""
        try:
            # Convert ROS message to numpy array
            if msg.encoding == '32FC1':
                gt_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            elif msg.encoding == 'mono16' or msg.encoding == '16UC1':
                # Isaac Sim depth is typically in millimeters as uint16
                gt_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
                gt_depth = gt_depth / 1000.0  # Convert mm to meters
            else:
                gt_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough').astype(np.float32)
            
            # PFM format from dataset is vertically flipped - correct it
            if self.data_source == 'dataset':
                gt_depth = np.flipud(gt_depth)
            
            with self._gt_depth_lock:
                self._latest_gt_depth = gt_depth
                self._gt_depth_available = True
        except Exception as e:
            self.get_logger().warn(f'Failed to process GT depth: {e}')

    def _compute_depth_metrics(self, pred_depth, gt_depth, valid_mask=None):
        """
        Compute depth evaluation metrics between predicted and GT depth.
        
        Returns:
            dict: Dictionary containing metrics:
                - abs_rel: Absolute relative error
                - sq_rel: Squared relative error  
                - rmse: Root mean squared error
                - rmse_log: RMSE of log depths
                - a1, a2, a3: Threshold accuracies (delta < 1.25^i)
        """
        if valid_mask is None:
            # Valid pixels: both depths are positive, finite, and within z_far range
            valid_mask = (gt_depth > 0.1) & (pred_depth > 0.1) & \
                        np.isfinite(gt_depth) & np.isfinite(pred_depth) & \
                        (gt_depth < self.z_far) & (pred_depth < self.z_far)
        
        if np.sum(valid_mask) == 0:
            return None
        
        pred = pred_depth[valid_mask]
        gt = gt_depth[valid_mask]
        
        # Compute metrics
        thresh = np.maximum((gt / pred), (pred / gt))
        a1 = (thresh < 1.25).mean()
        a2 = (thresh < 1.25 ** 2).mean()
        a3 = (thresh < 1.25 ** 3).mean()
        
        abs_rel = np.mean(np.abs(gt - pred) / gt)
        sq_rel = np.mean(((gt - pred) ** 2) / gt)
        mae = np.mean(np.abs(gt - pred))  # Mean Absolute Error in meters
        
        rmse = np.sqrt(np.mean((gt - pred) ** 2))
        rmse_log = np.sqrt(np.mean((np.log(gt) - np.log(pred)) ** 2))
        
        return {
            'abs_rel': abs_rel,
            'sq_rel': sq_rel,
            'mae': mae,
            'rmse': rmse,
            'rmse_log': rmse_log,
            'a1': a1,
            'a2': a2,
            'a3': a3,
            'valid_ratio': np.sum(valid_mask) / valid_mask.size
        }

    def _compute_error_map(self, pred_depth, gt_depth):
        """
        Compute per-pixel error map between predicted and GT depth.
        
        Returns:
            error_map: Relative error map (same size as input)
            valid_mask: Mask of valid pixels
        """
        valid_mask = (gt_depth > 0.1) & (pred_depth > 0.1) & \
                    np.isfinite(gt_depth) & np.isfinite(pred_depth) & \
                    (gt_depth < self.z_far) & (pred_depth < self.z_far)
        
        error_map = np.zeros_like(pred_depth)
        error_map[valid_mask] = np.abs(pred_depth[valid_mask] - gt_depth[valid_mask]) / gt_depth[valid_mask]
        
        return error_map, valid_mask

    def _process_latest_stereo(self):
        """Timer callback: process latest stereo pair if not busy"""
        if self.is_processing:
            return
        
        # Get latest stereo pair atomically
        stereo_pair = None
        with self._stereo_lock:
            if self._latest_stereo is not None:
                stereo_pair = self._latest_stereo
                self._latest_stereo = None  # Consume it
        
        if stereo_pair is not None:
            left_msg, right_msg = stereo_pair
            self._do_inference(left_msg, right_msg)

    def _do_inference(self, left_msg: Image, right_msg: Image):
        """Process synchronized stereo image pair"""
        # Measure actual callback interval (wall-clock time between processed frames)
        callback_start = time.time()
        if self._last_callback_time is not None:
            callback_interval = callback_start - self._last_callback_time
        else:
            callback_interval = 0.0
        
        self._last_callback_time = callback_start
        self.is_processing = True
        try:
            t0 = time.time()
            
            # Convert ROS images to numpy arrays
            # Handle different encodings
            if left_msg.encoding == 'mono8':
                left_img = self.bridge.imgmsg_to_cv2(left_msg, 'mono8')
                left_img = cv2.cvtColor(left_img, cv2.COLOR_GRAY2RGB)
            elif left_msg.encoding == 'mono16':
                left_img = self.bridge.imgmsg_to_cv2(left_msg, 'mono16')
                left_img = (left_img / 256).astype(np.uint8)
                left_img = cv2.cvtColor(left_img, cv2.COLOR_GRAY2RGB)
            else:
                left_img = self.bridge.imgmsg_to_cv2(left_msg, 'rgb8')
            
            if right_msg.encoding == 'mono8':
                right_img = self.bridge.imgmsg_to_cv2(right_msg, 'mono8')
                right_img = cv2.cvtColor(right_img, cv2.COLOR_GRAY2RGB)
            elif right_msg.encoding == 'mono16':
                right_img = self.bridge.imgmsg_to_cv2(right_msg, 'mono16')
                right_img = (right_img / 256).astype(np.uint8)
                right_img = cv2.cvtColor(right_img, cv2.COLOR_GRAY2RGB)
            else:
                right_img = self.bridge.imgmsg_to_cv2(right_msg, 'rgb8')
            
            t1 = time.time()
            preprocess_time = t1 - t0
            
            # Resize images
            if self.scale != 1.0:
                left_img = cv2.resize(left_img, None, fx=self.scale, fy=self.scale)
                right_img = cv2.resize(right_img, None, fx=self.scale, fy=self.scale)
            
            # Ensure 3 channels
            left_img = ensure_3ch(left_img)
            right_img = ensure_3ch(right_img)
            
            H, W = left_img.shape[:2]
            
            # Convert to tensors
            img0 = torch.as_tensor(left_img).cuda().float()[None].permute(0, 3, 1, 2)
            img1 = torch.as_tensor(right_img).cuda().float()[None].permute(0, 3, 1, 2)
            
            # Pad to be divisible by 32
            padder = InputPadder(img0.shape, divis_by=32, force_square=False)
            img0_padded, img1_padded = padder.pad(img0, img1)
            
            t2 = time.time()
            
            # Run inference
            with torch.cuda.amp.autocast(True):
                if not self.use_hiera:
                    disp = self.model.forward(img0_padded, img1_padded, 
                                             iters=self.valid_iters, test_mode=True)
                else:
                    disp = self.model.run_hierachical(img0_padded, img1_padded,
                                                      iters=self.valid_iters, 
                                                      test_mode=True, small_ratio=0.5)
            
            torch.cuda.synchronize()
            t3 = time.time()
            model_time = t3 - t2
            
            # Unpad and convert to numpy
            disp = padder.unpad(disp.float())
            disp = disp.data.cpu().numpy().reshape(H, W)
            
            t4 = time.time()
            gpu_to_cpu_time = t4 - t3
            
            # Convert disparity to depth
            # depth = focal_length * baseline / disparity
            # Use scaled focal length for the resized image
            focal_length = self.K[0, 0] * self.scale  # fx scaled for output resolution
            
            # Avoid division by zero
            disp_safe = np.where(disp > 0.1, disp, 0.1)
            depth = (focal_length * self.baseline) / disp_safe
            
            # Clip invalid depths and apply z_far
            depth = np.where(disp > 0.1, depth, 0.0)
            depth = np.where(depth <= self.z_far, depth, 0.0)  # Clip far depths
            depth = depth.astype(np.float32)
            
            # Apply bilateral filter for noise reduction (edge-preserving smoothing)
            # This helps reduce sporadic depth spikes from noisy input images
            if self.enable_bilateral_filter:
                # Bilateral filter preserves edges while smoothing flat regions
                # d: diameter of pixel neighborhood (5-9 typical)
                # sigmaColor: larger = more colors mixed
                # sigmaSpace: larger = farther pixels influence
                valid_mask = depth > 0
                depth_filtered = cv2.bilateralFilter(
                    depth, 
                    d=self.bilateral_d,
                    sigmaColor=self.bilateral_sigma_color,
                    sigmaSpace=self.bilateral_sigma_space
                )
                # Only apply filter to valid depth regions
                depth = np.where(valid_mask, depth_filtered, depth)
            
            # Determine timestamp based on mode
            # 'current': Use current time - ensures TF is available (recommended for real-time)
            # 'original': Use input image timestamp - for offline/rosbag processing
            if self.timestamp_mode == 'current':
                output_stamp = self.get_clock().now().to_msg()
            else:
                output_stamp = left_msg.header.stamp
            
            # Convert to ROS message
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
            depth_msg.header.stamp = output_stamp
            depth_msg.header.frame_id = self.camera_frame_id
            
            # Build and publish CameraInfo (same timestamp as depth)
            camera_info_msg = self._build_camera_info_msg(W, H, output_stamp)
            self.camera_info_pub.publish(camera_info_msg)
            
            # Publish depth
            self.depth_pub.publish(depth_msg)
            
            # Publish color image with same timestamp (for nvblox color integration)
            # Resize left image to match depth dimensions and convert to RGB
            color_resized = cv2.resize(left_img, (W, H))
            if len(color_resized.shape) == 2:
                color_resized = cv2.cvtColor(color_resized, cv2.COLOR_GRAY2RGB)
            elif color_resized.shape[2] == 4:
                color_resized = cv2.cvtColor(color_resized, cv2.COLOR_RGBA2RGB)
            # Note: left_img is already RGB from ensure_3ch, publish as rgb8
            color_msg = self.bridge.cv2_to_imgmsg(color_resized, encoding='rgb8')
            color_msg.header.stamp = output_stamp
            color_msg.header.frame_id = self.camera_frame_id
            self.color_pub.publish(color_msg)
            self.color_info_pub.publish(camera_info_msg)  # Same camera info as depth
            
            t5 = time.time()
            postprocess_time = t5 - t4
            total_callback_time = t5 - t0
            
            # Statistics
            self.frame_count += 1
            self.total_inference_time += total_callback_time
            avg_time = self.total_inference_time / self.frame_count
            
            # GT depth evaluation
            gt_depth_resized = None
            error_map = None
            metrics = None
            
            if self.enable_gt_evaluation:
                with self._gt_depth_lock:
                    if self._latest_gt_depth is not None and self._gt_depth_available:
                        gt_depth_resized = cv2.resize(self._latest_gt_depth, (W, H), 
                                                      interpolation=cv2.INTER_NEAREST)
                
                if gt_depth_resized is not None:
                    # Compute metrics
                    metrics = self._compute_depth_metrics(depth, gt_depth_resized)
                    error_map, _ = self._compute_error_map(depth, gt_depth_resized)
                    
                    # Publish depth metrics as JSON for navigation_metrics_logger
                    if metrics is not None:
                        import json
                        metrics_msg = String()
                        metrics_msg.data = json.dumps({
                            k: float(v) for k, v in metrics.items()
                        })
                        self.depth_metrics_pub.publish(metrics_msg)
                    
                    # Metrics logging controlled by verbose flag (every 30 frames = ~10s at 3fps)
                    if self.verbose and metrics is not None and self.frame_count % 30 == 0:
                        self.get_logger().info(
                            f'Metrics - abs_rel: {metrics["abs_rel"]:.4f}, '
                            f'a1: {metrics["a1"]:.3f}, rmse: {metrics["rmse"]:.3f}m'
                        )
            
            # Visualization - just store frame, separate thread handles display
            if self.enable_visualization:
                self._update_viz_frame(depth, total_callback_time, left_img, 
                                      gt_depth_resized, error_map, metrics)
            
            # Point cloud visualization update
            if self.enable_pc_visualization and gt_depth_resized is not None:
                self._update_pc_viz_frame(depth, gt_depth_resized)
            
            # Frame stats logging controlled by verbose flag (every 30 frames)
            if self.verbose and self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Frame {self.frame_count}: inference={total_callback_time:.3f}s, '
                    f'avg={avg_time:.3f}s, fps={1.0/avg_time:.1f}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing stereo pair: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.is_processing = False

    def _update_viz_frame(self, depth, inference_time, rgb_image, 
                         gt_depth=None, error_map=None, metrics=None):
        """Store latest frame for visualization thread (non-blocking)"""
        with self._viz_lock:
            self._latest_viz_frame = depth.copy()
            self._latest_rgb_frame = rgb_image.copy()  # Store RGB for side-by-side viz
            self._latest_gt_depth_frame = gt_depth.copy() if gt_depth is not None else None
            self._latest_error_map = error_map.copy() if error_map is not None else None
            self._latest_metrics = metrics.copy() if metrics is not None else None
            self._latest_viz_info = (inference_time, self.frame_count)

    def _update_pc_viz_frame(self, pred_depth, gt_depth):
        """Store latest depth frames for PC visualization thread (non-blocking)"""
        with self._pc_viz_lock:
            self._latest_pc_pred_depth = pred_depth.copy()
            self._latest_pc_gt_depth = gt_depth.copy()

    def _depth_to_pointcloud(self, depth, color, downsample=1):
        """
        Convert depth map to Open3D point cloud.
        
        Args:
            depth: HxW depth map in meters
            color: RGB color tuple (0-1 range) for all points
            downsample: Skip every N pixels (1=full resolution)
        
        Returns:
            Open3D PointCloud object
        """
        o3d = self._o3d
        H, W = depth.shape
        
        # Scaled intrinsics for the depth map resolution
        fx = self.K[0, 0] * self.scale
        fy = self.K[1, 1] * self.scale
        cx = self.K[0, 2] * self.scale
        cy = self.K[1, 2] * self.scale
        
        # Create pixel coordinate grid (downsampled)
        u = np.arange(0, W, downsample)
        v = np.arange(0, H, downsample)
        u, v = np.meshgrid(u, v)
        
        # Get downsampled depth
        depth_ds = depth[::downsample, ::downsample]
        
        # Valid depth mask
        valid = (depth_ds > 0.1) & (depth_ds < self.z_far) & np.isfinite(depth_ds)
        
        if np.sum(valid) == 0:
            # Return empty point cloud
            pcd = o3d.geometry.PointCloud()
            return pcd
        
        # Back-project to 3D (camera coordinates: X-right, Y-down, Z-forward)
        z = depth_ds[valid]
        x = (u[valid] - cx) * z / fx
        y = (v[valid] - cy) * z / fy
        
        points = np.stack([x, y, z], axis=-1)
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Set uniform color for all points
        colors = np.tile(np.array(color), (len(points), 1))
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd

    def _pc_visualization_thread_worker(self):
        """
        Open3D point cloud visualization thread.
        Displays predicted depth (red/orange) and GT depth (blue/cyan) overlaid.
        """
        o3d = self._o3d
        
        # Create visualizer with legend in window title
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='Point Cloud: Predicted (Orange) vs GT (Cyan)', width=1280, height=720)
        
        # Set render options
        opt = vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.1])  # Dark gray background
        opt.point_size = 2.0
        
        # Initialize empty point clouds
        pcd_pred = o3d.geometry.PointCloud()
        pcd_gt = o3d.geometry.PointCloud()
        
        # Add to visualizer
        vis.add_geometry(pcd_pred)
        vis.add_geometry(pcd_gt)
        
        # Add coordinate frame for reference
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        vis.add_geometry(coord_frame)
        
        # Set initial viewpoint (looking along Z axis)
        ctr = vis.get_view_control()
        ctr.set_front([0, 0, -1])
        ctr.set_up([0, -1, 0])
        ctr.set_lookat([0, 0, 5])
        ctr.set_zoom(0.5)
        
        first_frame = True
        
        while self._pc_viz_running:
            pred_depth = None
            gt_depth = None
            
            with self._pc_viz_lock:
                if self._latest_pc_pred_depth is not None and self._latest_pc_gt_depth is not None:
                    pred_depth = self._latest_pc_pred_depth.copy()
                    gt_depth = self._latest_pc_gt_depth.copy()
                    self._latest_pc_pred_depth = None  # Consume
                    self._latest_pc_gt_depth = None
            
            if pred_depth is not None and gt_depth is not None:
                # Convert depth maps to point clouds
                # Predicted: Orange/Red color
                new_pcd_pred = self._depth_to_pointcloud(
                    pred_depth, color=[1.0, 0.4, 0.0], downsample=self.pc_downsample_factor)
                # GT: Cyan/Blue color
                new_pcd_gt = self._depth_to_pointcloud(
                    gt_depth, color=[0.0, 0.8, 1.0], downsample=self.pc_downsample_factor)
                
                # Update point cloud data
                pcd_pred.points = new_pcd_pred.points
                pcd_pred.colors = new_pcd_pred.colors
                pcd_gt.points = new_pcd_gt.points
                pcd_gt.colors = new_pcd_gt.colors
                
                # Update geometry in visualizer
                vis.update_geometry(pcd_pred)
                vis.update_geometry(pcd_gt)
                
                # Reset viewpoint on first frame
                if first_frame:
                    vis.reset_view_point(True)
                    first_frame = False
            
            # Process Open3D events (non-blocking)
            vis.poll_events()
            vis.update_renderer()
            
            # Small sleep to prevent CPU spinning
            time.sleep(0.033)  # ~30Hz update
        
        # Cleanup
        vis.destroy_window()

    def _visualization_thread_worker(self):
        """Unified visualization thread - handles both cv2 and topic modes.
        Runs completely separate from ROS2 executor to avoid blocking inference."""
        last_frame_id = -1
        
        while self._viz_running:
            frame_data = None
            rgb_frame = None
            gt_depth_frame = None
            error_map = None
            metrics = None
            
            with self._viz_lock:
                if self._latest_viz_frame is not None and self._latest_viz_info is not None:
                    _, frame_id = self._latest_viz_info
                    if frame_id != last_frame_id:
                        frame_data = (self._latest_viz_frame.copy(), self._latest_viz_info)
                        rgb_frame = self._latest_rgb_frame.copy() if self._latest_rgb_frame is not None else None
                        gt_depth_frame = self._latest_gt_depth_frame.copy() if self._latest_gt_depth_frame is not None else None
                        error_map = self._latest_error_map.copy() if self._latest_error_map is not None else None
                        metrics = self._latest_metrics.copy() if self._latest_metrics is not None else None
                        last_frame_id = frame_id
            
            if frame_data is not None:
                depth, (inference_time, frame_count) = frame_data
                
                # Normalize depth to 0-255 range for visualization
                depth_viz = np.clip(depth, 0, self.z_far)
                depth_viz = (depth_viz / self.z_far * 255).astype(np.uint8)
                
                # Apply colormap
                depth_colored = cv2.applyColorMap(depth_viz, cv2.COLORMAP_INFERNO)
                
                # Add text info on depth image
                fps = 1.0 / inference_time if inference_time > 0 else 0
                cv2.putText(depth_colored, f'Pred Depth', (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(depth_colored, f'FPS: {fps:.1f}', (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(depth_colored, f'z_far: {self.z_far:.1f}m', (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Create visualization panels
                panels = []
                
                # Panel 1: Original RGB image
                if rgb_frame is not None:
                    rgb_bgr = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                    cv2.putText(rgb_bgr, 'Input', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(rgb_bgr, f'Frame: {frame_count}', (10, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    panels.append(rgb_bgr)
                
                # Panel 2: Predicted depth
                panels.append(depth_colored)
                
                # Panel 3 & 4: GT depth and error map (if available)
                if gt_depth_frame is not None:
                    # GT depth visualization (only show depths within z_far range)
                    gt_valid_mask = (gt_depth_frame > 0.1) & (gt_depth_frame < self.z_far)
                    gt_viz = np.zeros_like(gt_depth_frame)
                    gt_viz[gt_valid_mask] = gt_depth_frame[gt_valid_mask]
                    gt_viz = (gt_viz / self.z_far * 255).astype(np.uint8)
                    gt_colored = cv2.applyColorMap(gt_viz, cv2.COLORMAP_INFERNO)
                    cv2.putText(gt_colored, 'GT Depth', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(gt_colored, f'z_far: {self.z_far:.1f}m', (10, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    panels.append(gt_colored)
                    
                    # Error map visualization (only show valid pixels within z_far range)
                    if error_map is not None:
                        # Create valid mask: only pixels where both pred and gt are within z_far
                        error_valid_mask = (gt_depth_frame > 0.1) & (gt_depth_frame < self.z_far) & \
                                          (depth > 0.1) & (depth < self.z_far)
                        # Error is relative error (0-1 typically, clip to 0.5 max)
                        error_viz = np.zeros_like(error_map)
                        error_viz[error_valid_mask] = np.clip(error_map[error_valid_mask], 0, 0.5)
                        error_viz = (error_viz / 0.5 * 255).astype(np.uint8)
                        # Use JET colormap: blue=low error, red=high error
                        error_colored = cv2.applyColorMap(error_viz, cv2.COLORMAP_JET)
                        # Mask out invalid pixels to black (JET colormap shows 0 as blue, not black)
                        error_colored[~error_valid_mask] = [0, 0, 0]
                        cv2.putText(error_colored, 'Error (0-50%)', (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        
                        # Display metrics on error map
                        if metrics is not None:
                            cv2.putText(error_colored, f'abs_rel: {metrics["abs_rel"]:.4f}', (10, 55),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            cv2.putText(error_colored, f'MAE: {metrics["mae"]:.3f}m', (10, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            cv2.putText(error_colored, f'RMSE: {metrics["rmse"]:.3f}m', (10, 105),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            cv2.putText(error_colored, f'a1: {metrics["a1"]:.3f}', (10, 130),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        panels.append(error_colored)
                
                # Combine panels in 2x2 grid if we have 4 panels, otherwise horizontal
                if len(panels) == 4:
                    # 2x2 grid: [Input | Pred] / [GT | Error]
                    top_row = np.hstack([panels[0], panels[1]])
                    bottom_row = np.hstack([panels[2], panels[3]])
                    combined = np.vstack([top_row, bottom_row])
                elif len(panels) == 3:
                    # 3 panels: [Input | Pred | GT]
                    combined = np.hstack(panels)
                elif len(panels) == 2:
                    combined = np.hstack(panels)
                else:
                    combined = depth_colored
                
                if self.viz_mode == 'cv2':
                    cv2.imshow('Stereo Depth', combined)
                else:
                    # Topic mode - publish to ROS2
                    try:
                        viz_msg = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
                        viz_msg.header.stamp = self.get_clock().now().to_msg()
                        viz_msg.header.frame_id = self.camera_frame_id
                        self.depth_viz_pub.publish(viz_msg)
                    except:
                        pass
            
            # For cv2 mode, always call waitKey to keep GUI alive
            if self.viz_mode == 'cv2':
                key = cv2.waitKey(16)  # ~60Hz GUI update
                if key == 27:  # ESC
                    cv2.destroyAllWindows()
                    break
            else:
                time.sleep(0.016)  # ~60Hz for topic mode

    def destroy_node(self):
        """Clean shutdown"""
        self._viz_running = False
        self._pc_viz_running = False
        if self._viz_thread is not None and self._viz_thread.is_alive():
            self._viz_thread.join(timeout=1.0)
        if self._pc_viz_thread is not None and self._pc_viz_thread.is_alive():
            self._pc_viz_thread.join(timeout=1.0)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = FoundationStereoDepthNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down. Processed {node.frame_count} frames, '
            f'avg time: {node.total_inference_time/max(1,node.frame_count):.3f}s'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
