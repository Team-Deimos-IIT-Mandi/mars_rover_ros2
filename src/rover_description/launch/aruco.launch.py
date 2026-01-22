import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Setup paths
    pkg_share = get_package_share_directory('rover_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'aruco_marker.xacro')
    aruco_namespace = 'aruco_marker_1'

    # Launch arguments
    args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (set to false for hardware)'
        ),
        DeclareLaunchArgument('x', default_value='4.0'),
        DeclareLaunchArgument('y', default_value='2.0'),
        DeclareLaunchArgument('z', default_value='0.35'), 
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='1.0'),
        DeclareLaunchArgument('model_name', default_value='arucoMarker1'),
        DeclareLaunchArgument('model_file', default_value='arucoMarker0.dae')
    ]

    robot_description_content = Command([
        'xacro ', xacro_file, 
        ' model_file:=', LaunchConfiguration('model_file')
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=aruco_namespace,
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': LaunchConfiguration('use_sim_time'), 'frame_prefix': f'{aruco_namespace}/'}]
    )

    # NOTE: spawn_aruco node removed - it's Gazebo-specific and not needed for hardware
    # If you need Gazebo simulation, use a separate simulation-specific launch file

    return LaunchDescription(args + [
        robot_state_publisher_node,
    ])




   

    