#!/usr/bin/env python3
# =============================================================================
# NVBlox + Nav2 Navigation Launch (LiDAR 없음, collision_monitor 비활성화)
# velocity_smoother가 cmd_vel을 직접 출력
# 
# Supports both sim mode and dataset mode:
#   - use_sim_time:=True  -> uses sim_nav2.yaml (for Isaac Sim)
#   - use_sim_time:=False -> uses dataset_nav2.yaml (for dataset playback)
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('nvblox_integration')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Default config file (sim mode)
    default_params_file = os.path.join(pkg_dir, 'config', 'sim_nav2.yaml')

    # Lifecycle manager에서 관리할 노드들 (collision_monitor 제외)
    # ★ velocity_smoother는 initial_rotation_controller와 충돌하므로 제외
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        # 'velocity_smoother'  # 비활성화: initial_rotation_controller가 cmd_vel 직접 관리
    ]

    # Remap velocity_smoother output: cmd_vel_smoothed -> cmd_vel
    # collision_monitor가 없으므로 velocity_smoother가 직접 cmd_vel 출력
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    velocity_smoother_remappings = remappings + [('cmd_vel_smoothed', 'cmd_vel')]
    
    # ★ controller_server의 cmd_vel 출력을 /nav2_cmd_vel로 remap
    # initial_rotation_controller가 이를 구독하고 /cmd_vel로 패스스루
    controller_server_remappings = remappings + [('cmd_vel', '/nav2_cmd_vel')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation (Isaac Sim) clock if true. False for dataset mode.'),

        DeclareLaunchArgument(
            'autostart', default_value='True',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file. Use dataset_nav2.yaml for dataset mode.'),

        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Whether to respawn if a node crashes'),

        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='log level'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=controller_server_remappings),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        # ★ velocity_smoother 비활성화: initial_rotation_controller가 cmd_vel 직접 관리
        # controller_server → /nav2_cmd_vel → initial_rotation_controller → /cmd_vel → 로봇
        # Node(
        #     package='nav2_velocity_smoother',
        #     executable='velocity_smoother',
        #     name='velocity_smoother',
        #     output='screen',
        #     respawn=use_respawn,
        #     respawn_delay=2.0,
        #     parameters=[configured_params],
        #     arguments=['--ros-args', '--log-level', log_level],
        #     # cmd_vel_smoothed -> cmd_vel remap (collision_monitor bypass)
        #     remappings=velocity_smoother_remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ])
