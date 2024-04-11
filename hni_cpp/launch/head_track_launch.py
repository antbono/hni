from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node'
        ),
        Node(
            package='hni_cpp',
            executable='head_track_action_server',
            name='head_track_action_server_node'
        ),
        Node(
            package='hni_cpp',
            executable='head_track_action_client',
            name='head_track_action_client_node'
        )
    ])