#!/usr/bin/env python3
# Launch file for Nav2 with GPS localization (no AMCL)
# Refer this: https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution 
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('rover_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    # Paths to config files
    params_file = LaunchConfiguration('params_file')
    
    # Declare the argument so it can be passed from the terminal
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_gps_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    # Remap cmd_vel to diff_drive_controller topic
    remappings = [
        ('/cmd_vel', '/diff_drive_controller/cmd_vel'),
    ]
    full_params_path = PathJoinSubstitution([
        FindPackageShare('rover_description'),
        'config',
        LaunchConfiguration('params_file')
    ])
    
    # Nav2 bringup with remappings
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': full_params_path,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )
    
    # Create a group action to apply remappings to all Nav2 nodes
    nav2_with_remappings = GroupAction([
        PushRosNamespace(''),
        nav2_bringup,
    ])
    
    # CMD_VEL relay to convert Twist to TwistStamped for diff_drive_controller
    cmd_vel_relay = Node(
        package='rover_description',
        executable='cmd_vel_relay.py',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_params_file,
        declare_autostart,
        cmd_vel_relay,
        nav2_with_remappings,
        rviz_node,
    ])
