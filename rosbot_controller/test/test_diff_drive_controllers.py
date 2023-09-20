# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2023 Intel Corporation. All Rights Reserved.
# Copyright 2023 Husarion
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch_pytest
import pytest
import rclpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from controllers_test_node import ControllersTestNode


@launch_pytest.fixture
def generate_test_description():
    rosbot_controller = get_package_share_directory("rosbot_controller")
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_controller,
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim": "False",
            "mecanum": "False",
            "use_gpu": "False",
        }.items(),
    )

    return LaunchDescription([bringup_launch])


@pytest.mark.launch(fixture=generate_test_description)
def test_controllers_startup_fail():
    rclpy.init()
    try:
        node = ControllersTestNode("test_controllers_bringup")
        node.create_test_subscribers_and_publishers()

        node.start_node_thread()
        msgs_received_flag = node.joint_state_msg_event.wait(timeout=10.0)
        assert (
            not msgs_received_flag
        ), "Received JointStates message, check joint_state_broadcaster!"
        msgs_received_flag = node.odom_msg_event.wait(timeout=10.0)
        assert (
            not msgs_received_flag
        ), "Received Odom message, check rosbot_base_controller!"
        msgs_received_flag = node.joint_state_msg_event.wait(timeout=10.0)
        assert not msgs_received_flag, "Received Imu message, check imu_broadcaster!"
    finally:
        rclpy.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_controllers_startup_success():
    rclpy.init()
    try:
        node = ControllersTestNode("test_controllers_bringup")
        node.create_test_subscribers_and_publishers()
        node.start_publishing_fake_hardware()

        node.start_node_thread()
        msgs_received_flag = node.joint_state_msg_event.wait(timeout=10.0)
        assert (
            msgs_received_flag
        ), "Did not receive JointStates message, check joint_state_broadcaster!"
        msgs_received_flag = node.odom_msg_event.wait(timeout=10.0)
        assert (
            msgs_received_flag
        ), "Did not receive Odom message, check rosbot_base_controller!"
        msgs_received_flag = node.joint_state_msg_event.wait(timeout=10.0)
        assert msgs_received_flag, "Did not receive Imu message, check imu_broadcaster!"
    finally:
        rclpy.shutdown()
