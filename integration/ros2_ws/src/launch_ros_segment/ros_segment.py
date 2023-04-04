from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='light_pos',
            executable='light_pos',
        ),
        Node(
            package='controller',
            executable='controller',
        )
    ])