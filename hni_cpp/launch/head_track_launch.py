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
            executable='head_track_action_client',
            name='head_track_action_client_node'
        ),
        Node(
            package='hni_cpp',
            executable='head_track_action_server',
            name='head_track_action_server_node',
            remappings=[
                ('video_obj_tracker/_action/feedback', 'video_face_tracker/_action/feedback'),
                ('video_obj_tracker/_action/status', 'video_face_tracker/_action/status'),
                ('video_obj_tracker/_action/cancel_goal', 'video_face_tracker/_action/cancel_goal'),
                ('video_obj_tracker/_action/get_result', 'video_face_tracker/_action/get_result'),
                ('video_obj_tracker/_action/send_goal', 'video_face_tracker/_action/send_goal'),
            ],
        ),
        Node(
            package='nao_pos_server',
            executable='nao_pos_action_client',
            name='nao_pos_action_client_head',
            remappings=[
                ('action_req', 'action_req_head'),
                ('nao_pos_action/_action/feedback', 'nao_pos_action_head/_action/feedback'),
                ('nao_pos_action/_action/status', 'nao_pos_action_head/_action/status'),
                ('nao_pos_action/_action/cancel_goal', 'nao_pos_action_head/_action/cancel_goal'),
                ('nao_pos_action/_action/get_result', 'nao_pos_action_head/_action/get_result'),
                ('nao_pos_action/_action/send_goal', 'nao_pos_action_head/_action/send_goal'),
            ],
        ),
        Node(
            package='nao_pos_server',
            executable='nao_pos_action_server',
            name='nao_pos_action_server_head',
            remappings=[
                ('nao_pos_action/_action/feedback', 'nao_pos_action_head/_action/feedback'),
                ('nao_pos_action/_action/status', 'nao_pos_action_head/_action/status'),
                ('nao_pos_action/_action/cancel_goal', 'nao_pos_action_head/_action/cancel_goal'),
                ('nao_pos_action/_action/get_result', 'nao_pos_action_head/_action/get_result'),
                ('nao_pos_action/_action/send_goal', 'nao_pos_action_head/_action/send_goal'),
            ],
        ),
    ])