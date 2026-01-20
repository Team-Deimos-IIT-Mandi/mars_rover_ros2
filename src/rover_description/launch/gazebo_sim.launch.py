#!/usr/bin/env python3
import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def resolve_package_path(urdf_content, pkg_path):
    """Replace package:// URIs and hardcoded paths with absolute paths"""
    mesh_path = os.path.join(pkg_path, 'meshes')
    config_path = os.path.join(pkg_path, 'config', 'controllers.yaml')
    
    # Replace package://rover_description/meshes
    urdf_content = urdf_content.replace(
        'package://rover_description/meshes',
        f'file://{mesh_path}'
    )
    
    # Regex to replace the hardcoded controllers.yaml path
    # Matches <parameters>.../config/controllers.yaml</parameters>
    pattern = r'<parameters>.*controllers\.yaml</parameters>'
    # check for alternatives to fix the fixed controllers yaml path in urdf 
    replacement = f'<parameters>{config_path}</parameters>'
    urdf_content = re.sub(pattern, replacement, urdf_content)
    
    return urdf_content

def generate_launch_description():
    pkg_path = get_package_share_directory('rover_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    urdf_path = os.path.join(pkg_path, 'urdf', 'rover.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
    
    # Process the URDF to fix paths
    robot_description = resolve_package_path(robot_description, pkg_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items()
    )

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

    cameras_lidar_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/camera_1/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera_2/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ],
    output='screen'
    )
    
    imu_gps_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
       '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
       '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
    ],
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

    # Foxglove Bridge - for web-based visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_compression': True,
            'use_sim_time': True,
            'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe', 
                           'services', 'connectionGraph', 'assets'],
            'asset_uri_allowlist': ['^package://.*'],
        }]
    )

    return LaunchDescription([
        gz_sim,
        clock_bridge,
        cameras_lidar_bridge,
        imu_gps_bridge,
        robot_state_publisher,
        spawn_robot,
        foxglove_bridge,
    ])