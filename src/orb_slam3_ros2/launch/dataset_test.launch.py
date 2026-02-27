"""
Dataset Test Launch File

Launches both the dataset publisher and stereo SLAM node for testing
with LuSNAR dataset before Isaac Sim integration.

Usage:
    ros2 launch orb_slam3_ros2 dataset_test.launch.py

With custom parameters:
    ros2 launch orb_slam3_ros2 dataset_test.launch.py \
        dataset_path:=/path/to/dataset \
        publish_rate:=20.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('orb_slam3_ros2')
    config_file = os.path.join(pkg_share, 'config', 'stereo_lusnr.yaml')
    
    # Declare launch arguments
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value=os.path.expanduser('~/WALJU/data/LuSNAR/Moon_2'),
        description='Path to dataset root directory'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Image publishing rate in Hz'
    )
    
    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer',
        default_value='true',
        description='Enable ORB-SLAM3 Pangolin viewer'
    )
    
    # SLAM Node (start first)
    slam_node = Node(
        package='orb_slam3_ros2',
        executable='stereo_slam',
        name='stereo_slam_node',
        output='screen',
        parameters=[config_file, {
            'enable_viewer': LaunchConfiguration('enable_viewer'),
        }],
    )
    
    # Dataset Publisher Node (start after SLAM is ready)
    # Using TimerAction to delay start by 5 seconds
    dataset_node = TimerAction(
        period=5.0,  # Wait 5 seconds for SLAM to initialize
        actions=[
            Node(
                package='orb_slam3_ros2',
                executable='dataset_publisher',
                name='dataset_publisher',
                output='screen',
                parameters=[config_file, {
                    'dataset_path': LaunchConfiguration('dataset_path'),
                    'publish_rate': LaunchConfiguration('publish_rate'),
                    'left_subdir': 'image0/color',
                    'right_subdir': 'image1/color',
                }],
            )
        ]
    )
    
    return LaunchDescription([
        dataset_path_arg,
        publish_rate_arg,
        enable_viewer_arg,
        slam_node,
        dataset_node,
    ])
