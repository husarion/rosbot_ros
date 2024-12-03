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

from threading import Thread

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from test_ign_kill_utils import kill_ign_linux_processes
from test_utils import SimulationTestNode, mecanum_test


@launch_pytest.fixture
def generate_test_description():
    rosbot_gazebo = FindPackageShare("rosbot_gazebo")
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_gazebo,
                    "launch",
                    "simulation.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_headless_mode": "True",
            "gz_world": PathJoinSubstitution(
                [FindPackageShare("husarion_gz_worlds"), "worlds", "empty_with_plugins.sdf"]
            ),
            "healthcheck": "False",
            "mecanum": "True",
            "namespace": "rosbot",
        }.items(),
    )

    return LaunchDescription(
        [
            simulation_launch,
            KeepAliveProc(),
            # Tell launch to start the test
            ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=generate_test_description)
def test_namespaced_mecanum_simulation():
    rclpy.init()
    try:
        node = SimulationTestNode("test_namespaced_mecanum_simulation", namespace="rosbot")
        Thread(target=lambda node: rclpy.spin(node), args=(node,)).start()

        mecanum_test(node)

    finally:
        # The pytest cannot kill properly the Gazebo Ignition's tasks what blocks launching
        # several tests in a row.
        kill_ign_linux_processes()
        rclpy.shutdown()
