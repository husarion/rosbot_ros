# Copyright 2020 ros2_control Development Team
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

import os
import subprocess

from launch import LaunchDescription
from launch.actions import EmitEvent, OpaqueFunction, TimerAction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def check_controller_status(context):
    healthcheck = LaunchConfiguration("healthcheck").perform(context)
    if healthcheck.lower() == "false":
        with open("/var/tmp/rosbot_status.txt", "w") as status_file:
            status_file.write("healthy")
        return
    else:
        with open("/var/tmp/rosbot_status.txt", "w") as status_file:
            status_file.write("unhealthy")

    use_sim = LaunchConfiguration("use_sim", default="False").perform(context)
    nodes_to_check = [
        "/controller_manager",
        "/ekf_filter_node",
        "/imu_broadcaster",
        "/joint_state_broadcaster",
        "/laser_scan_box_filter",
        "/robot_state_publisher",
        "/rosbot_base_controller",
        "/scan_to_scan_filter_chain",
    ]
    if use_sim.lower() == "true":
        additional_nodes = [
            "/gz_ros2_control",
            "/ros_gz_bridge",
        ]
    else:
        additional_nodes = [
            "/imu_sensor_node",
            # "/rosbot_ros2_firmware", not visible via USB port
            "/rosbot_system_node",
        ]
    nodes_to_check.extend(additional_nodes)

    namespace = LaunchConfiguration("namespace", default="").perform(context)
    ns = "/" + namespace if namespace else ""
    nodes_to_check = [ns + node for node in nodes_to_check]

    def get_missing_nodes():
        env = os.environ.copy()
        env["ROS_LOCALHOST_ONLY"] = "1"

        result = subprocess.run(
            ["ros2", "node", "list"],
            stdout=subprocess.PIPE,
            text=True,
            env=env,
        )
        active_nodes = result.stdout.splitlines()
        return [node for node in nodes_to_check if node not in active_nodes]

    missing_nodes = get_missing_nodes()
    green_color = "\033[92m"
    red_color = "\033[91m"
    reset_color = "\033[0m"

    if missing_nodes:
        print(
            f"{red_color}Error: some nodes are missing: {missing_nodes}. Emitting shutdown...{reset_color}"
        )
        return [EmitEvent(event=Shutdown())]
    else:
        print(f"{green_color}All systems are up and running!{reset_color}")
        with open("/var/tmp/rosbot_status.txt", "w") as status_file:
            status_file.write("healthy")


def generate_launch_description():
    check_controller = TimerAction(
        period=15.0,
        actions=[OpaqueFunction(function=check_controller_status)],
    )

    return LaunchDescription([check_controller])
