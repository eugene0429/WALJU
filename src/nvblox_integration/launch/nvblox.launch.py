#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
nvblox Launch File for Custom SLAM + Depth Pipeline

This launch file runs nvblox_node with remappings for:
- ORB-SLAM3 pose (/slam/pose)
- FoundationStereo depth (/depth/image, /depth/camera_info)

Usage:
    ros2 launch nvblox_integration nvblox.launch.py
    ros2 launch nvblox_integration nvblox.launch.py voxel_size:=0.1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('nvblox_integration')
    
    # Declare launch arguments
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.05',
        description='TSDF voxel size in meters'
    )
    
    global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='map',
        description='Global frame ID'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Camera frame ID'
    )
    
    max_integration_distance_arg = DeclareLaunchArgument(
        'max_integration_distance',
        default_value='20.0',
        description='Maximum depth integration distance in meters'
    )
    
    esdf_slice_height_arg = DeclareLaunchArgument(
        'esdf_slice_height',
        default_value='0.0',
        description='Height of the 2D ESDF slice visualization'
    )
    
    esdf_slice_min_height_arg = DeclareLaunchArgument(
        'esdf_slice_min_height',
        default_value='0.0',
        description='Minimum height for ESDF obstacle consideration'
    )
    
    esdf_slice_max_height_arg = DeclareLaunchArgument(
        'esdf_slice_max_height',
        default_value='1.0',
        description='Maximum height for ESDF obstacle consideration'
    )
    
    use_tf_transforms_arg = DeclareLaunchArgument(
        'use_tf_transforms',
        default_value='true',
        description='Use TF for pose instead of pose topic'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time from /clock topic'
    )

    # Config file path
    nvblox_config = os.path.join(pkg_dir, 'config', 'nvblox_custom.yaml')
    
    # nvblox node with remappings
    nvblox_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        output='screen',
        parameters=[
            nvblox_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'global_frame': LaunchConfiguration('global_frame'),
                'pose_frame': LaunchConfiguration('camera_frame'),
                'use_tf_transforms': True,  # Use TF from SLAM
                'use_depth': True,
                'use_color': True,  # Enable color for textured mesh
                'use_lidar': False,
                'num_cameras': 1,
                'map_clearing_frame_id': LaunchConfiguration('camera_frame'),
                'static_mapper.projective_integrator_max_integration_distance_m': 
                    LaunchConfiguration('max_integration_distance'),
                'static_mapper.esdf_slice_height': LaunchConfiguration('esdf_slice_height'),
                'static_mapper.esdf_slice_min_height': LaunchConfiguration('esdf_slice_min_height'),
                'static_mapper.esdf_slice_max_height': LaunchConfiguration('esdf_slice_max_height'),
            }
        ],
        remappings=[
            # Depth input
            ('camera_0/depth/image', '/depth/image'),
            ('camera_0/depth/camera_info', '/depth/camera_info'),
            # Color input (synchronized with depth from depth_node)
            ('camera_0/color/image', '/color/image'),
            ('camera_0/color/camera_info', '/color/camera_info'),
        ],
    )
    
    return LaunchDescription([
        voxel_size_arg,
        global_frame_arg,
        camera_frame_arg,
        max_integration_distance_arg,
        esdf_slice_height_arg,
        esdf_slice_min_height_arg,
        esdf_slice_max_height_arg,
        use_tf_transforms_arg,
        use_sim_time_arg,
        nvblox_node,
    ])
