#!/usr/bin/env python3
"""
Complete launch file for Mars Rover simulation with Foxglove visualization
This launches:
- Gazebo simulation
- Robot controllers
- Foxglove Bridge for visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directories
    pkg_rover_description = get_package_share_directory('rover_description')
    
    # Paths to other launch files
    gazebo_launch_path = os.path.join(pkg_rover_description, 'launch', 'gazebo_sim.launch.py')
    
    # Check if gazebo_sim.launch.py exists
    use_gazebo = os.path.exists(gazebo_launch_path)
    
    # URDF path
    urdf_path = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf')
    
    # Read URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Port for Foxglove Bridge'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Foxglove Bridge
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_compression': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Info message
    info_msg = LogInfo(
        msg=[
            '\n',
            '='*70, '\n',
            '  Mars Rover Simulation + Foxglove Visualization\n',
            '='*70, '\n',
            '  Foxglove Bridge: ws://localhost:', LaunchConfiguration('port'), '\n',
            '  Open Foxglove Studio and connect to visualize the rover!\n',
            '='*70, '\n'
        ]
    )
    
    launch_description = LaunchDescription([
        # Arguments
        port_arg,
        use_sim_time_arg,
        
        # Foxglove Bridge
        foxglove_bridge_node,
        
        # Info
        info_msg
    ])
    
    # Add Gazebo launch if it exists
    if use_gazebo:
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        )
        launch_description.add_action(gazebo_launch)
    
    return launch_description
