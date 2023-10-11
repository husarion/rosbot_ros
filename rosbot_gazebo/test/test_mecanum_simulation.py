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

from simulation_test_node import SimulationTestNode
from kill_ign import kill_ign_linux_processes


@launch_pytest.fixture
def generate_test_description():
    rosbot_gazebo = get_package_share_directory("rosbot_gazebo")
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
            "mecanum": "True",
            "headless": "True",
        }.items(),
    )

    return LaunchDescription([simulation_launch])


@pytest.mark.launch(fixture=generate_test_description)
def test_mecanum_simulation():
    rclpy.init()
    try:
        node = SimulationTestNode("test_bringup")
        node.create_test_subscribers_and_publishers()
        node.start_node_thread()

        msgs_received_flag = node.odom_tf_event.wait(timeout=60.0)
        assert (
            msgs_received_flag
        ), "Expected odom to base_link tf but it was not received. Check robot_localization!"

        # 0.9 m/s and 3.0 rad/s are controller's limits defined in
        #   rosbot_controller/config/mecanum_drive_controller.yaml
        node.set_destination_speed(0.9, 0.0, 0.0)
        msgs_received_flag = node.goal_x_event.wait(timeout=60.0)
        assert msgs_received_flag, "ROSbot does not move properly in x direction!"

        node.set_destination_speed(0.0, 0.9, 0.0)
        msgs_received_flag = node.goal_y_event.wait(timeout=60.0)
        assert msgs_received_flag, "ROSbot does not move properly in y direction!"

        node.set_destination_speed(0.0, 0.0, 3.0)
        msgs_received_flag = node.goal_yaw_event.wait(timeout=60.0)
        assert msgs_received_flag, "ROSbot does not rotate properly!"

        msgs_received_flag = node.scan_event.wait(timeout=60.0)
        assert msgs_received_flag, "ROSbot's lidar does not work properly!"

    finally:
        # The pytest cannot kill properly the Gazebo Ignition's tasks what blocks launching
        # several tests in a row.
        kill_ign_linux_processes()
        rclpy.shutdown()
