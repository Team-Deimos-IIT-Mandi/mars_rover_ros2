import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Path Setup
    # Replace 'rover_description' with your actual package name if different
    pkg_share = get_package_share_directory('rover_description')
    navigation_path = os.path.join(pkg_share,'launch','gps_navigation.launch.py')
    
    # 2. Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_gps_params.yaml'),
        description='Path to the nav2 parameters file'
    )

    full_params_path = PathJoinSubstitution([
        FindPackageShare('rover_description'),
        'config',
        LaunchConfiguration('params_file')
    ])

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': full_params_path
        }.items()
    )

    # 4. Your Spiral Generator Node
    spiral_node = Node(
        package='rover_description',      # Your package name
        executable='spiral_search_alone.py',  # The name you gave in setup.py/CMake
        name='spiral_waypoint_generator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_params_file,
        navigation_launch,
        spiral_node
    ])