from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # rosbridge WebSocket server — exposes ROS2 topics to browser via JSON/WebSocket
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',
                'unregister_timeout': 10.0,
            }],
            output='screen',
        ),

        # web_video_server — serves camera topics as MJPEG streams over HTTP
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'port': 8080,
                'address': '0.0.0.0',
                'default_stream_type': 'mjpeg',
            }],
            output='screen',
        ),
    ])
