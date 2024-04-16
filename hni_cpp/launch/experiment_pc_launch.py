from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hni_py', executable='face_track_server', name='face_track_server_node',
            output='screen',
        ),
        Node(
            package='hni_cpp', executable='chat_action_server', name='chat_action_server',
            prefix=['xterm -e'],
        ),
    ])