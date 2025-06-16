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

from itertools import product
from threading import Thread

import launch_pytest
import pytest
import rclpy
from gz_kill_process import kill_ign_linux_processes
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from test_utils import (
    SimulationTestNode,
    sensors_readings_test,
    speed_test,
    wait_for_initialization,
)


@launch_pytest.fixture
def generate_test_description(request):
    mecanum, namespace, robot_model = request.param
    print(
        f"""
Running test with
    mecanum={mecanum}
    namespace={namespace}
    robot_model={robot_model}
    """
    )
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
            "mecanum": mecanum,
            "namespace": namespace,
            "robot_model": robot_model,
        }.items(),
    )

    return (
        LaunchDescription(
            [
                simulation_launch,
                KeepAliveProc(),
                ReadyToTest(),
            ]
        ),
        mecanum,
        namespace,
        robot_model,
    )


mecanum_options = ["True", "False"]
namespace_options = ["", "test_ns"]
robot_model_options = ["rosbot", "rosbot_xl"]
test_params = list(product(mecanum_options, namespace_options, robot_model_options))


@pytest.mark.parametrize("generate_test_description", test_params, indirect=True)
@pytest.mark.launch(fixture=generate_test_description)
def test_simulation(generate_test_description):
    _, mecanum, namespace, robot_model = generate_test_description

    sensor_to_test = {
        "rosbot": [
            # "camera_color",
            # "camera_points",
            # "scan_filtered", # FIXME: Remove add param for tf_prefix in husarion_components_description
        ],  # Not simulated "range_fl","range_fr", "range_rl", "range_rr"
        "rosbot_xl": [],
    }

    rclpy.init()
    try:
        node = SimulationTestNode("test_simulation", namespace=namespace)
        Thread(target=lambda node: rclpy.spin(node), args=(node,)).start()

        wait_for_initialization(node)
        speed_test(node, "Test velocity in x direction", v_x=0.7)
        if mecanum == "True":
            speed_test(node, "Test velocity in y direction", v_y=0.7)
        speed_test(node, "Test velocity in yaw", v_yaw=3.0)
        sensors_readings_test(node, sensor_to_test[robot_model])

    finally:
        # The pytest cannot kill properly the Gazebo's tasks, which blocks launching
        # several tests in a row.
        kill_ign_linux_processes()
        rclpy.shutdown()
