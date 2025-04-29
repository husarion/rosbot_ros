# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2023 Intel Corporation. All Rights Reserved.
# Copyright 2024 Husarion sp. z o.o.
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
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from test_utils import BringupTestNode, readings_data_test


@launch_pytest.fixture
def generate_test_description():
    rosbot_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "rosbot_bringup",
            "bringup.launch.py",
            "microros:=False",
            "namespace:=robot1",
            "robot_model:=rosbot",
        ],
        output="screen",
    )

    rosbot_xl_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "rosbot_bringup",
            "bringup.launch.py",
            "microros:=False",
            "namespace:=robot2",
            "robot_model:=rosbot_xl",
        ],
        output="screen",
    )

    return LaunchDescription([rosbot_launch, rosbot_xl_launch])


@pytest.mark.launch(fixture=generate_test_description)
def test_multirobot_bringup():
    robot_names = ["robot1", "robot2"]

    for robot_name in robot_names:
        rclpy.init()
        try:
            node = BringupTestNode("test_multirobot_bringup", namespace=robot_name)
            node.start_publishing_fake_hardware()
            node.start_node_thread()

            readings_data_test(node, robot_name)

        finally:
            rclpy.shutdown()
