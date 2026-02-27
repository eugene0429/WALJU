"""
Full Navigation Pipeline Launch File

Launches the complete navigation stack:
1. Nav2 Navigation Stack
2. Visualization tools (optional)

Prerequisites:
- Isaac Sim running with camera/depth output
- FoundationStereo depth estimation running
- ORB-SLAM3 pose estimation running
- nvblox 3D mapping (optional, for mesh visualization)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    nvblox_integration_share = get_package_share_directory('nvblox_integration')
    
    # ===== Launch Arguments =====
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='Launch Nav2 navigation stack'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz visualization'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/depth/image',
        description='Depth image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/depth/camera_info',
        description='Camera info topic'
    )
    
    # ===== Nav2 Launch =====
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nvblox_integration_share, 'launch', 'nav2_bringup.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_nav2')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
        }.items()
    )
    
    # ===== RViz Visualization =====
    rviz_config_file = os.path.join(
        nvblox_integration_share, 'config', 'nav2_visualization.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen',
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        launch_nav2_arg,
        launch_rviz_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        
        # Launch components
        nav2_launch,
        rviz_node,
    ])
