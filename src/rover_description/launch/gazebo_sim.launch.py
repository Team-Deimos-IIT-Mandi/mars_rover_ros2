#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get paths for packages and files
    pkg_path = get_package_share_directory('rover_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    urdf_path = os.path.join(pkg_path, 'urdf', 'rover.urdf')
    world_file = "empty.sdf"
    
    # Load the robot description from the xacro file
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    # Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items()
    )

    # Spawn the robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )
    
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mars_rover',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn_robot, 
    ])
