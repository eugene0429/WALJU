#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Full Pipeline Launch File

Launches the complete pipeline:
1. Dataset Publisher (stereo images)
2. ORB-SLAM3 (stereo SLAM -> pose)
3. FoundationStereo (stereo -> depth)
4. nvblox (depth + pose -> TSDF/ESDF)
5. RViz2 (visualization)

Usage:
    ros2 launch nvblox_integration full_pipeline.launch.py \
        dataset_path:=/path/to/dataset
    
    # Without RViz
    ros2 launch nvblox_integration full_pipeline.launch.py run_rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    orb_slam3_pkg = get_package_share_directory('orb_slam3_ros2')
    foundation_stereo_pkg = get_package_share_directory('foundation_stereo_ros2')
    nvblox_integration_pkg = get_package_share_directory('nvblox_integration')
    
    # RViz config path
    rviz_config = os.path.join(nvblox_integration_pkg, 'config', 'custom_example.rviz')
    
    # ============================================
    # Launch Arguments
    # ============================================
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value=os.path.expanduser('~/WALJU/data/LuSNAR/Moon_2'),
        description='Path to dataset directory'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='2.0',
        description='Dataset publish rate (Hz)'
    )
    
    vocabulary_path_arg = DeclareLaunchArgument(
        'vocabulary_path',
        default_value=os.path.expanduser('~/WALJU/deps/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
        description='Path to ORB vocabulary file'
    )
    
    slam_settings_arg = DeclareLaunchArgument(
        'slam_settings',
        default_value=os.path.expanduser('~/WALJU/deps/ORB_SLAM3/Examples/Stereo/LuSNAR.yaml'),
        description='Path to SLAM settings YAML'
    )
    
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.05',
        description='nvblox voxel size (meters)'
    )
    
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz',
        default_value='true',
        description='Whether to launch RViz2 for visualization'
    )
    
    # ============================================
    # Node Definitions
    # ============================================
    
    # 0. Static TF: camera_link -> base_link (identity transform)
    # nvblox requires base_link for map clearing
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'base_link'],
        output='screen'
    )
    
    # 1. Dataset Publisher
    dataset_publisher_node = Node(
        package='orb_slam3_ros2',
        executable='dataset_publisher',
        name='dataset_publisher',
        output='screen',
        parameters=[{
            'dataset_path': LaunchConfiguration('dataset_path'),
            'left_subdir': 'image0/color',
            'right_subdir': 'image1/color',
            'publish_rate': LaunchConfiguration('publish_rate'),
            'loop': False,
        }],
    )
    
    # 2. ORB-SLAM3 Node
    slam_node = Node(
        package='orb_slam3_ros2',
        executable='stereo_slam',
        name='stereo_slam_node',
        output='screen',
        parameters=[{
            'vocabulary_path': LaunchConfiguration('vocabulary_path'),
            'settings_path': LaunchConfiguration('slam_settings'),
            'enable_viewer': True,
            'camera_tilt_deg': 70.077,
        }],
    )
    
    # 3. FoundationStereo Depth Node (delayed start to allow SLAM initialization)
    depth_node = TimerAction(
        period=3.0,  # Wait 3 seconds for SLAM to initialize
        actions=[
            Node(
                package='foundation_stereo_ros2',
                executable='depth_node',
                name='foundation_stereo_depth_node',
                output='screen',
                parameters=[{
                    'scale': 0.5,
                    'valid_iters': 32,
                    'z_far': 20.0,
                    'camera_frame_id': 'camera_link',
                    'enable_visualization': True,
                }],
            )
        ]
    )
    
    # 4. nvblox Node (delayed start to allow depth node initialization)
    nvblox_launch = TimerAction(
        period=10.0,  # Wait 10 seconds for depth node to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nvblox_integration_pkg, 'launch', 'nvblox.launch.py')
                ),
                launch_arguments={
                    'voxel_size': LaunchConfiguration('voxel_size'),
                }.items()
            )
        ]
    )
    
    # 5. RViz2 for visualization (delayed start)
    rviz_node = TimerAction(
        period=12.0,  # Wait 12 seconds for nvblox to initialize
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                condition=IfCondition(LaunchConfiguration('run_rviz'))
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        dataset_path_arg,
        publish_rate_arg,
        vocabulary_path_arg,
        slam_settings_arg,
        voxel_size_arg,
        run_rviz_arg,
        
        # Nodes (in order)
        static_tf_node,      # Static TF first
        dataset_publisher_node,
        slam_node,
        depth_node,
        nvblox_launch,
        rviz_node,
    ])
