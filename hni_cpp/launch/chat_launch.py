from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='nao_lola_client',
        #    executable='nao_lola_client',
        #    name='lola_node',
        #    output='screen'
        #),
        Node(
            package='sound_play',
            executable='soundplay_node.py',
            name='soundplay_node',
            output='screen'
        ),
        Node(
            package='hni_py',
            executable='gstt_service.py',
            name='gstt_service_node',
            output='screen'
        ),
        Node(
            package='hni_py',
            executable='chat_service.py',
            name='chat_service_node',
            output='screen'
        ),
        Node(
            package='hni_py',
            executable='gtts_service.py',
            name='gtts_service_node',
            output='screen'
        ),
        Node(
            package='hni_cpp',
            executable='joints_play_action_server',
            name='joints_play_action_server',
            output='screen'
        ),
        Node(
            package='nao_led_server',
            executable='led_action_server',
            name='led_action_server',
            output='screen'
        ),
        #Node(
        #    package='hni_cpp',
        #    executable='chat_action_server',
        #    name='chat_action_server',
        #    output='screen',
        #),
        Node(
            package='hni_cpp',
            executable='chat_action_client',
            name='chat_action_client',
            output='screen'
        ),
    ])