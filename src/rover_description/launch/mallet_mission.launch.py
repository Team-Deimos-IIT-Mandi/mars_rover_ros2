from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = get_package_share_directory('rover_description')
    mallet_launch_path = os.path.join(pkg_share, 'launch', 'mallet.launch.py')
    navigation_path = os.path.join(pkg_share,'launch','gps_navigation.launch.py')

    lat_arg = DeclareLaunchArgument('lat', default_value='0.0')
    lon_arg = DeclareLaunchArgument('lon', default_value='0.0')
    target_lat_val = LaunchConfiguration('lat')
    target_lon_val = LaunchConfiguration('lon')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    mallet_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mallet_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
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

    initializer_node = Node(
        package='rover_description',
        executable='mission_initializer.py',
        name='mission_initializer',
        parameters=[{
            'use_sim_time': True,
            'target_lat': target_lat_val,
            'target_lon': target_lon_val,
        }],
        output='screen',
        emulate_tty=True 
    )

    spiral_node = Node(
        package='rover_description',
        executable='spiral_search_od.py', 
        name='spiral_waypoint_generator',
        parameters=[{
            'use_sim_time': True,
        }],
        output='screen'
    )

    # ArUco Nodes
    camera_1 = Node(package='rover_description', 
                    executable='mallet_detection.py', 
                    name='mallet_front', 
                    parameters=[{
                        'use_sim_time': True,
                    }],output='screen')
    camera_2 = Node(package='rover_description', 
                    executable='mallet_detection1.py', 
                    name='mallot_left', 
                    parameters=[{
                        'use_sim_time': True,
                    }],output='screen')
    camera_3 = Node(package='rover_description', 
                    executable='mallet_detection2.py', 
                    name='mallet_right', 
                    parameters=[{
                        'use_sim_time': True,
                    }],output='screen')

    return LaunchDescription([
        mallet_gz_sim,
        declare_params_file,
        navigation_launch,
        initializer_node,
        spiral_node,
        camera_1,
        camera_2,
        camera_3
    ])