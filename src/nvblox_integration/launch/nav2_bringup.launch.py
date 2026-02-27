"""
Nav2 Bringup Launch File

Launches Nav2 navigation stack with elevation map-based costmap.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    nvblox_integration_share = get_package_share_directory('nvblox_integration')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start Nav2 lifecycle nodes'
    )
    
    # Nav2 parameters
    nav2_params_file = os.path.join(
        nvblox_integration_share, 'config', 'nav2', 'nav2_params.yaml'
    )
    
    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to Nav2 params file'
    )
    
    # Include Nav2 bringup launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        autostart_arg,
        nav2_params_arg,
        nav2_launch,
    ])
