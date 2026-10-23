# Copyright 2026 Husarion sp. z o.o.
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

"""Regression guard: under a namespace joy2twist must publish on `/<ns>/manual/cmd_vel`,
not the lowest-priority `/<ns>/cmd_vel` twist_mux input.
"""

import time

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc

NAMESPACE = "test_ns"
TELEOP_NODE = "joy2twist"


@launch_pytest.fixture
def joy_launch_description():
    joy_launch = GroupAction(
        [
            PushRosNamespace(NAMESPACE),
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare("rosbot_joy"), "launch", "joy.yaml"])
            ),
        ]
    )
    return LaunchDescription([joy_launch, KeepAliveProc(), ReadyToTest()])


def _teleop_publishes_on(node, topic):
    return any(
        info.node_name == TELEOP_NODE and info.node_namespace == f"/{NAMESPACE}"
        for info in node.get_publishers_info_by_topic(topic)
    )


@pytest.mark.launch(fixture=joy_launch_description)
def test_joy2twist_publishes_on_namespaced_manual_cmd_vel():
    rclpy.init()
    try:
        node = rclpy.create_node("test_namespace_remap")
        manual_topic = f"/{NAMESPACE}/manual/cmd_vel"

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not _teleop_publishes_on(node, manual_topic):
            time.sleep(0.2)

        assert _teleop_publishes_on(
            node, manual_topic
        ), f"{TELEOP_NODE} not publishing on {manual_topic}"
        assert not _teleop_publishes_on(
            node, f"/{NAMESPACE}/cmd_vel"
        ), f"{TELEOP_NODE} leaked onto /{NAMESPACE}/cmd_vel (cmd_vel remap not applied)"
    finally:
        rclpy.shutdown()
