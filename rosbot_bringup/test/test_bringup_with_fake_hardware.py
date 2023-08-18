# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from pathlib import Path
import sys
from threading import Event
from threading import Thread

import launch
import launch_pytest
import launch_ros

import pytest

import rclpy
from rclpy.node import Node

from launch.substitutions import  PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution
)

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry


@launch_pytest.fixture
def generate_test_description():
    rosbot_bringup = get_package_share_directory("rosbot_bringup")
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_bringup,
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim": "True",
            "mecanum": "False",
            "use_gpu": "False"
        }.items(),
    )

    return launch.LaunchDescription([
        # bringup_launch
    ])


@pytest.mark.launch(fixture=generate_test_description)
def test_controllers_startup():
    rclpy.init()
    try:
        node = TestControllersBringup('test_controllers_bringup')
        node.create_test_subscribers()

        node.start_node_thread()
        msgs_received_flag = node.joint_state_msg_event.wait(timeout=5.0)
        assert msgs_received_flag, 'Did not receive JointStates message, check joint_state_broadcaster!'
        msgs_received_flag = node.odom_msg_event.wait(timeout=5.0)
        assert msgs_received_flag, 'Did not receive Odom message, check rosbot_base_controller!'
        msgs_received_flag = node.joint_state_msg_event.wait(timeout=5.0)
        assert msgs_received_flag, 'Did not receive Imu message, check imu_broadcaster!'
    finally:
        rclpy.shutdown()

class TestControllersBringup(Node):
    __test__ = False

    def __init__(self, name='test_node'):
        super().__init__(name)
        self.joint_state_msg_event = Event()
        self.odom_msg_event = Event()
        self.imu_msg_event = Event()

    def create_test_subscribers(self):
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/rosbot_base_controller/odom',
            self.odometry_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_broadcaster/imu',
            self.joint_states_callback,
            10
        )

    def start_node_thread(self):
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def joint_states_callback(self, data):
        self.joint_state_msg_event.set()

    def odometry_callback(self, data):
        self.odom_msg_event.set()

    def imu_callback(self, data):
        self.imu_msg_event.set()