from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch rosbridge_server for web GUI communication
    This enables WebSocket connection for real-time process tracking
    """
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0.0,
                'max_message_size': 10000000,
                'unregister_timeout': 10.0
            }],
            output='screen'
        )
    ])
