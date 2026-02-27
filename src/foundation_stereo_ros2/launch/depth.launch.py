#!/usr/bin/env python3
"""
Launch file for FoundationStereo depth node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('foundation_stereo_ros2')
    
    # Default config file
    default_config = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the config file'
    )
    
    left_topic_arg = DeclareLaunchArgument(
        'left_topic',
        default_value='/stereo/left/image_raw',
        description='Left image topic'
    )
    
    right_topic_arg = DeclareLaunchArgument(
        'right_topic',
        default_value='/stereo/right/image_raw',
        description='Right image topic'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/depth/image',
        description='Output depth topic'
    )
    
    scale_arg = DeclareLaunchArgument(
        'scale',
        default_value='0.5',
        description='Image scale factor'
    )
    
    # Node
    depth_node = Node(
        package='foundation_stereo_ros2',
        executable='depth_node.py',
        name='foundation_stereo_depth_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'left_topic': LaunchConfiguration('left_topic'),
                'right_topic': LaunchConfiguration('right_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'scale': LaunchConfiguration('scale'),
            }
        ],
    )
    
    return LaunchDescription([
        config_file_arg,
        left_topic_arg,
        right_topic_arg,
        depth_topic_arg,
        scale_arg,
        depth_node,
    ])
