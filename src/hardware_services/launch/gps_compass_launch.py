#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('hardware_services')
    
    # GPS device port
    gps_port = LaunchConfiguration('gps_port', default='/dev/ttyUSB0')
    
    # GPS node (using nmea_navsat_driver)
    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'gps_config.yaml')
        ],
        remappings=[
            ('/fix', '/gps/fix'),
            ('/vel', '/gps/vel'),
            ('/time_reference', '/gps/time_reference')
        ]
    )
    
    # Compass node (IST8310)
    compass_node = Node(
        package='hardware_services',
        executable='ist8310_compass_node.py',
        name='compass_driver',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'compass_config.yaml')
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'gps_port',
            default_value='/dev/ttyUSB0',
            description='GPS serial port'
        ),
        gps_node,
        compass_node
    ])
