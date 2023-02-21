import os
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing_ros


def generate_test_description():
    launch_description = LaunchDescription()
    publisher_node_parameters = {
        'reliability': 'reliable',
        'show_camera': False,
        'burger_mode': True,
        'frequency': 5.0,
    }
    subscriber_node_parameters = {
        'reliability': 'reliable',
        'show_image': False,
    }

    launch_description.add_action(SetEnvironmentVariable('OSPL_VERBOSITY', '8'))  # 8 = OS_NONE
    # bare minimum formatting for console output matching
    launch_description.add_action(
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}'))
    launch_description.add_action(
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'))

    # Launch the process that will receive the images.
    # This is the process that gets to decide when to tear the launcher down.
    # It will exit when the regex for receiving images is matched.
    showimage_executable = '/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/showimage'
    showimage_name = 'test_showimage'

    showimage_node = Node(
        executable=showimage_executable,
        name=showimage_name,
        parameters=[subscriber_node_parameters],
        output='screen'
    )
    launch_description.add_action(showimage_node)

    # Launch the process that will publish the images.
    # This process will be exited when the launcher is torn down.
    cam2image_executable = '/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/cam2image'
    cam2image_name = 'test_cam2image'

    cam2image_node = Node(
        executable=cam2image_executable,
        name=cam2image_name,
        parameters=[publisher_node_parameters],
        output='screen'
    )
    launch_description.add_action(cam2image_node)

    launch_description.add_action(
        launch_testing.actions.ReadyToTest()
    )

    return launch_description, locals()


class TestExecutablesDemo(unittest.TestCase):

    def test_reliable_qos(self, proc_output, showimage_node, cam2image_node):
        """Test QoS settings for both processes work as expected."""
        output_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_rmw_implementation='rmw_fastrtps_cpp'
        )
        proc_output.assertWaitFor(
            expected_output=launch_testing.tools.expected_output_from_file(
                path='/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/test/showimage'
            ), process=showimage_node, output_filter=output_filter, timeout=10
        )
        proc_output.assertWaitFor(
            expected_output=launch_testing.tools.expected_output_from_file(
                path='/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/test/cam2image'
            ), process=cam2image_node, output_filter=output_filter, timeout=10
        )
