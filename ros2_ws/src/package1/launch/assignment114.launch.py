from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package1',
            executable='cam2image',
        ),
        Node(
            package='package1',
            executable='showimage',
        ),
        Node(
            package='package1',
            executable='lightracker'
        )
    ])