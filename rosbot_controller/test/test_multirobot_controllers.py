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
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from test_utils import ControllersTestNode

robot_names = ["rosbot1", "rosbot2"]


@launch_pytest.fixture
def generate_test_description():
    rosbot_controller = get_package_share_directory("rosbot_controller")
    bringup_rosbot1_launch = IncludeLaunchDescription(
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
            "mecanum": "True",
            "use_gpu": "False",
            "namespace": robot_names[0],
        }.items(),
    )

    bringup_rosbot2_launch = IncludeLaunchDescription(
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
            "namespace": robot_names[1],
        }.items(),
    )

    delayed_spawn_robot2 = TimerAction(period=10.0, actions=[bringup_rosbot2_launch])

    return LaunchDescription([bringup_rosbot1_launch, delayed_spawn_robot2])


@pytest.mark.launch(fixture=generate_test_description)
def test_multirobot_controllers_startup_success():
    for robot_name in robot_names:
        rclpy.init()
        try:
            node = ControllersTestNode(f"test_{robot_name}_controllers", namespace=robot_name)
            node.create_test_subscribers_and_publishers()
            node.start_publishing_fake_hardware()

            node.start_node_thread()
            msgs_received_flag = node.joint_state_msg_event.wait(timeout=10.0)
            assert msgs_received_flag, (
                f"Expected JointStates message but it was not received. Check {robot_name}/"
                "joint_state_broadcaster!"
            )
            msgs_received_flag = node.odom_msg_event.wait(timeout=10.0)
            assert msgs_received_flag, (
                f"Expected Odom message but it was not received. Check {robot_name}/"
                "rosbot_base_controller!"
            )
            msgs_received_flag = node.imu_msg_event.wait(timeout=10.0)
            assert (
                msgs_received_flag
            ), f"Expected Imu message but it was not received. Check {robot_name}/imu_broadcaster!"
        finally:
            rclpy.shutdown()
