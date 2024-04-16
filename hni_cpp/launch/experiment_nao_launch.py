from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nao_lola_client',
            executable='nao_lola_client',
            name='lola_node',
            output='screen',
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("hni_cpp"), '/launch', '/chat_launch.py'])
            ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("hni_cpp"), '/launch', '/head_track_launch.py'])
            ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("nao_pos_server"), '/launch', '/swing_launch.py'])
            ),
    ])