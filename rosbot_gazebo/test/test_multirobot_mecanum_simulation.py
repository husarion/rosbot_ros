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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from test_utils import SimulationTestNode
from test_ign_kill_utils import kill_ign_linux_processes


@launch_pytest.fixture
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    # IncludeLaunchDescription does not work with robots argument
    simulation_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "rosbot_gazebo",
            "simulation.launch.py",
            (
                f'world:={get_package_share_directory("husarion_office_gz")}'
                "/worlds/empty_with_plugins.sdf"
            ),
            "robots:=robot1={y: -4.0}; robot2={y: 0.0}; robot3={y: 4.0};",
            "headless:=True",
            "mecanum:=True",
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
def test_multirobot_mecanum_simulation():
    robot_names = ["robot1", "robot2", "robot3"]
    rclpy.init()
    try:
        nodes = {}
        executor = MultiThreadedExecutor(num_threads=len(robot_names))

        for robot_name in robot_names:
            node = SimulationTestNode("test_simulation", namespace=robot_name)
            node.create_test_subscribers_and_publishers()
            nodes[robot_name] = node
            executor.add_node(node)

        ros_spin_thread = Thread(target=lambda executor: executor.spin(), args=(executor,))
        ros_spin_thread.start()

        for robot_name in robot_names:
            node = nodes[robot_name]
            # 0.9 m/s and 3.0 rad/s are controller's limits defined in
            #   rosbot_controller/config/mecanum_drive_controller.yaml
            node.set_destination_speed(0.9, 0.0, 0.0)

        for robot_name in robot_names:
            node = nodes[robot_name]
            assert node.vel_stabilization_time_event.wait(timeout=120.0), (
                f"{robot_name}: The simulation is running slowly or has crashed! The time elapsed"
                " since setting the target speed is:"
                f" {(node.current_time - node.goal_received_time):.1f}."
            )
            assert node.controller_odom_flag, (
                f"{robot_name}: does not move properly in x direction. Check"
                f" rosbot_base_controller! Twist: {node.twist}"
            )
            assert node.ekf_odom_flag, (
                f"{robot_name}: does not move properly in x direction. Check ekf_filter_node!"
                f" Twist: {node.twist}"
            )

        for robot_name in robot_names:
            node = nodes[robot_name]
            # 0.9 m/s and 3.0 rad/s are controller's limits defined in
            #   rosbot_controller/config/mecanum_drive_controller.yaml
            node.set_destination_speed(0.0, 0.9, 0.0)

        for robot_name in robot_names:
            node = nodes[robot_name]
            assert node.vel_stabilization_time_event.wait(timeout=120.0), (
                f"{robot_name}: The simulation is running slowly or has crashed! The time elapsed"
                " since setting the target speed is:"
                f" {(node.current_time - node.goal_received_time):.1f}."
            )
            assert node.controller_odom_flag, (
                f"{robot_name} does not move properly in y direction. Check"
                f" rosbot_base_controller! Twist: {node.twist}"
            )
            assert node.ekf_odom_flag, (
                f"{robot_name} does not move properly in y direction. Check ekf_filter_node!"
                f" Twist: {node.twist}"
            )

        for robot_name in robot_names:
            node = nodes[robot_name]
            node.set_destination_speed(0.0, 0.0, 3.0)

        for robot_name in robot_names:
            node = nodes[robot_name]
            assert node.vel_stabilization_time_event.wait(timeout=120.0), (
                f"{robot_name}: The simulation is running slowly or has crashed! The time elapsed"
                " since setting the target speed is:"
                f" {(node.current_time - node.goal_received_time):.1f}."
            )
            assert node.controller_odom_flag, (
                f"{robot_name} does not rotate properly. Check rosbot_base_controller! Twist:"
                f" {node.twist}"
            )
            assert (
                node.ekf_odom_flag
            ), f"{robot_name} does not rotate properly. Check ekf_filter_node! Twist: {node.twist}"

            flag = node.scan_event.wait(timeout=20.0)
            assert flag, f"{robot_name}'s lidar does not work properly!"

            for i in range(len(node.RANGE_SENSORS_TOPICS)):
                flag = node.ranges_events[i].wait(timeout=20.0)
                assert flag, (
                    f"{robot_name}:'s range sensor {node.RANGE_SENSORS_TOPICS[i]} does not work"
                    " properly!"
                )

            flag = node.camera_color_event.wait(timeout=20.0)
            assert flag, f"{robot_name}:'s camera color image does not work properly!"

            flag = node.camera_points_event.wait(timeout=20.0)
            assert flag, f"{robot_name}:'s camera point cloud does not work properly!"

            node.destroy_node()

    finally:
        rclpy.shutdown()

        # The pytest cannot kill properly the Gazebo Ignition's tasks what blocks launching
        # several tests in a row.
        kill_ign_linux_processes()
