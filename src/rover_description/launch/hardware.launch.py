#!/usr/bin/env python3
"""
Hardware Launch File for Mars Rover on Jetson Nano
This launch file starts all hardware components including:
- Robot state publisher
- Hardware controllers (CAN-based motor control)
- Sensor drivers (cameras, IMU, GPS, LiDAR)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('rover_description').find('rover_description')
    
    # File paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf')
    controllers_config = os.path.join(pkg_share, 'config', 'controllers.yaml')
    sensors_config = os.path.join(pkg_share, 'config', 'sensors.yaml')
    cameras_launch = os.path.join(pkg_share, 'launch', 'cameras.launch.py')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set to false for hardware)'
    )
    
    # Robot State Publisher - publishes robot TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Hardware Controller Manager - interfaces with CAN-based motor controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_config],
        output='both',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )
    
    # Joint State Broadcaster - publishes joint states to /joint_states
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Differential Drive Controller - handles robot motion control
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager',
                   '--param-file', controllers_config],
        output='screen'
    )
    
    # ============ SENSOR NODES ============
    
    # Cameras - Launch all 3 USB cameras using cameras.launch.py
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cameras_launch),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )
    
    # GPS Node - Using nmea_navsat_driver
    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver',
        parameters=[sensors_config],
        remappings=[
            ('/fix', '/gps/fix'),
        ],
        output='screen'
    )
    
    # IMU Node - Adjust package/executable based on your IMU
    # Examples:
    # - For MPU6050: package='mpu6050_driver'
    # - For BNO055: package='ros_imu_bno055'
    # - For generic serial IMU: package='imu_serial_driver'
    #
    # Uncomment and adjust based on your IMU:
    # imu_node = Node(
    #     package='YOUR_IMU_PACKAGE',
    #     executable='YOUR_IMU_EXECUTABLE',
    #     name='imu_driver',
    #     parameters=[sensors_config],
    #     remappings=[
    #         ('/imu', '/imu/data'),
    #     ],
    #     output='screen'
    # )
    
    # LiDAR Node - Adjust based on your LiDAR model
    # For RPLidar:
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar',
        parameters=[sensors_config],
        output='screen'
    )
    # For YDLidar, use:
    # lidar_node = Node(
    #     package='ydlidar_ros2_driver',
    #     executable='ydlidar_ros2_driver_node',
    #     name='ydlidar',
    #     parameters=[sensors_config],
    #     output='screen'
    # )
    
    # ============ LAUNCH DESCRIPTION ============
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        
        # Robot description and controllers
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        
        # Sensors
        cameras,
        gps_node,
        # imu_node,  # Uncomment when IMU driver is configured
        lidar_node,
    ])
