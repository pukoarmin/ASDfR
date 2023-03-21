# Launch file for assignment 2.3
# ************************************************
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='timing',
            executable='Seq04',
        ),
        Node(
            package='timing',
            executable='Loop04',
        )
    ])