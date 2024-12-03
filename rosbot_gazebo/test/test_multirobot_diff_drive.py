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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from rclpy.executors import SingleThreadedExecutor
from test_ign_kill_utils import kill_ign_linux_processes
from test_utils import SimulationTestNode, diff_test


@launch_pytest.fixture
def generate_test_description():
    gz_world_path = (
        get_package_share_directory("husarion_gz_worlds") + "/worlds/empty_with_plugins.sdf"
    )
    simulation_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "rosbot_gazebo",
            "simulation.launch.py",
            "gz_headless_mode:=True",
            f"gz_world:={gz_world_path}",
            "robots:=robot1={y: -4.0}; robot2={y: 0.0};",
            "healthcheck:=False",
        ],
        output="screen",
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
def test_multirobot_diff_drive_simulation():
    robots = ["robot1", "robot2"]
    rclpy.init()
    try:
        nodes = {}
        executor = SingleThreadedExecutor()

        for node_namespace in robots:
            node = SimulationTestNode(
                "test_multirobot_diff_drive_simulation", namespace=node_namespace
            )
            nodes[node_namespace] = node
            executor.add_node(node)

        Thread(target=lambda executor: executor.spin(), args=(executor,)).start()

        for node_namespace in robots:
            node = nodes[node_namespace]
            diff_test(node, node_namespace)
            executor.remove_node(node)
            node.destroy_node()

    finally:
        # The pytest cannot kill properly the Gazebo Ignition's tasks what blocks launching
        # several tests in a row.
        executor.shutdown()
        kill_ign_linux_processes()
        rclpy.shutdown()
