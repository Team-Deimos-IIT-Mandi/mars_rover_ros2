import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Setup paths
    pkg_share = get_package_share_directory('rover_description')
    gz_plus_controller_path = os.path.join(pkg_share, 'launch', 'gz_plus_control.launch.py')
    
    # Path to your mallet URDF
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'rockpick.urdf')
    model_namespace = 'rock_pick'

    # Read the URDF file content directly
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_plus_controller_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    args = [
        DeclareLaunchArgument('x', default_value='3.5'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.5'), 
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='1.0'),
        DeclareLaunchArgument('model_name', default_value='rockpick')
    ]

    # Node to publish the robot state to TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=model_namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_desc, # Passing the string directly
            'use_sim_time': True, 
            'frame_prefix': f'{model_namespace}/'
        }]
    )

    # Node to spawn the entity in Gazebo Harmonic
    spawn_rockpick = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', LaunchConfiguration('model_name'),
            '-topic', f'/{model_namespace}/robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen'
    )

    return LaunchDescription(args + [
        gazebo_sim,
        robot_state_publisher_node,
        spawn_rockpick,
    ])