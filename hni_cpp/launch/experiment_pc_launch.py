from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hni_py', executable='yolo_video_track_server', name='yolo_face_track_server_node',
            output='screen',
            remappings=[
                ('video_obj_tracker/_action/feedback', 'video_face_tracker/_action/feedback'),
                ('video_obj_tracker/_action/status', 'video_face_tracker/_action/status'),
                ('video_obj_tracker/_action/cancel_goal', 'video_face_tracker/_action/cancel_goal'),
                ('video_obj_tracker/_action/get_result', 'video_face_tracker/_action/get_result'),
                ('video_obj_tracker/_action/send_goal', 'video_face_tracker/_action/send_goal'),
            ],

        ),
        Node(
            package='hni_cpp', executable='chat_action_server', name='chat_action_server',
            prefix=['xterm -e'],
        ),
    ])