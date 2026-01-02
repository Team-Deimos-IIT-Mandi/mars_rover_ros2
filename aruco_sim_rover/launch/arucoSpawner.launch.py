import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path Setup
    pkg_name = 'aruco_sim_rover'
    pkg_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'aruco_marker.xacro')

    # 2. Declare Launch Arguments (equivalent to <arg> in ROS 1)
    x_pose = LaunchConfiguration('x', default='4.0')
    y_pose = LaunchConfiguration('y', default='2.0')
    z_pose = LaunchConfiguration('z', default='0.0')
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='1.0')
    model_name = LaunchConfiguration('model_name', default='arucoMarker1')
    model_file_arg = LaunchConfiguration('model_file', default='arucoMarker0.dae')

    # 3. Process Xacro (equivalent to <param command="...">)
    # We pass the 'model_file' argument directly into the xacro command
    robot_description_content = Command([
        'xacro ', xacro_file, 
        ' model_file:=', model_file_arg
    ])

    # 4. Define the Spawn Node (equivalent to gazebo_ros spawn_model)
    # In Gazebo Harmonic, we use ros_gz_sim/create
    spawn_aruco = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', model_name,
            '-string', robot_description_content,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ]
    )

    return LaunchDescription([
        # Declare arguments so they can be overridden via command line
        DeclareLaunchArgument('x', default_value='4.0'),
        DeclareLaunchArgument('y', default_value='2.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='1.0'),
        DeclareLaunchArgument('model_name', default_value='arucoMarker1'),
        DeclareLaunchArgument('model_file', default_value='arucoMarker0.dae'),
        
        spawn_aruco
    ])