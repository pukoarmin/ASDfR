
# Launch file for assignment 1.2.1
# ************************************************

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unit_test',
            executable='set_point_pub',
        ),
        Node(
            package='jiwy_simulator',
            executable='jiwy_simulator',
        ),
           Node(
            package='package1',
            executable='cam2image',
            remappings=[
                ('/image', '/webcam_input'),
            ]

        ),
        Node(
            package='package1',
            executable='showimage',
            remappings=[
                ('/image', '/moving_camera_output'),
            ]
        )
    ])
