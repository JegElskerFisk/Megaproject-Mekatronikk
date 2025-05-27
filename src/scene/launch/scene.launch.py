from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scene',
            executable='interactive_scene',
            name='interactive_scene',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='scene',
            executable='table_scene',
            name='table_scene',
            output='screen',
            emulate_tty=True,
        )
    ])