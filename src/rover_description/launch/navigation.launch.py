import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('rover_description')
    gazebo_launch_path = os.path.join(pkg_share, 'launch', 'gazebo_sim.launch.py')
    controllers_launch_path = os.path.join(pkg_share, 'launch', 'controllers.launch.py')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_yaml_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config_dir = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')
    navsat_config_file = os.path.join(get_package_share_directory('rover_description'), 'config', 'gps.yaml')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controllers_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        
        gazebo_sim,
        controllers,

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': True}],
            remappings=[
                ('/odometry/filtered', '/odometry/local')
            ]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_waypoint',
            output='screen',
            parameters=[navsat_config_file, {'use_sim_time': True}],
            remappings=[
                ('/gps/fix', '/gps/input'),
                ('/odometry/filtered', '/odometry/filtered/global'),
                ('/odometry/gps', '/gps_point'),
                ('/gps/filtered', '/waypoint/gps')
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_file,
                'params_file': nav2_params_file,
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])