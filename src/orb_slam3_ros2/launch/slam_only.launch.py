"""
SLAM Only Launch File

Launches only the stereo SLAM node for use with external image sources
like Isaac Sim or a real stereo camera.

Usage:
    ros2 launch orb_slam3_ros2 slam_only.launch.py

For Isaac Sim integration:
    ros2 launch orb_slam3_ros2 slam_only.launch.py \
        left_topic:=/front_stereo_camera/left/image_raw \
        right_topic:=/front_stereo_camera/right/image_raw
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('orb_slam3_ros2')
    config_file = os.path.join(pkg_share, 'config', 'stereo_lusnr.yaml')
    
    # Declare launch arguments
    left_topic_arg = DeclareLaunchArgument(
        'left_topic',
        default_value='/stereo/left/image_raw',
        description='Left camera image topic'
    )
    
    right_topic_arg = DeclareLaunchArgument(
        'right_topic',
        default_value='/stereo/right/image_raw',
        description='Right camera image topic'
    )
    
    vocabulary_path_arg = DeclareLaunchArgument(
        'vocabulary_path',
        default_value=os.path.expanduser('~/WALJU/deps/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
        description='Path to ORB vocabulary file'
    )
    
    settings_path_arg = DeclareLaunchArgument(
        'settings_path',
        default_value=os.path.expanduser('~/WALJU/deps/ORB_SLAM3/Examples/Stereo/Custom.yaml'),
        description='Path to camera settings YAML'
    )
    
    camera_tilt_arg = DeclareLaunchArgument(
        'camera_tilt_deg',
        default_value='70.077',
        description='Camera tilt angle in degrees'
    )
    
    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='true',
        description='Enable ORB-SLAM3 Pangolin viewer'
    )
    
    gt_file_arg = DeclareLaunchArgument(
        'gt_file_path',
        default_value='',
        description='Path to ground truth file (optional)'
    )
    
    output_file_arg = DeclareLaunchArgument(
        'output_trajectory_file',
        default_value='',
        description='Path to output trajectory file (optional)'
    )
    
    # SLAM Node
    slam_node = Node(
        package='orb_slam3_ros2',
        executable='stereo_slam',
        name='stereo_slam_node',
        output='screen',
        parameters=[{
            'vocabulary_path': LaunchConfiguration('vocabulary_path'),
            'settings_path': LaunchConfiguration('settings_path'),
            'left_topic': LaunchConfiguration('left_topic'),
            'right_topic': LaunchConfiguration('right_topic'),
            'camera_tilt_deg': LaunchConfiguration('camera_tilt_deg'),
            'enable_viewer': LaunchConfiguration('enable_viewer'),
            'gt_file_path': LaunchConfiguration('gt_file_path'),
            'output_trajectory_file': LaunchConfiguration('output_trajectory_file'),
            'output_frame_id': 'map',
            'camera_frame_id': 'camera_link',
        }],
    )
    
    return LaunchDescription([
        left_topic_arg,
        right_topic_arg,
        vocabulary_path_arg,
        settings_path_arg,
        camera_tilt_arg,
        enable_viewer_arg,
        gt_file_arg,
        output_file_arg,
        slam_node,
    ])
