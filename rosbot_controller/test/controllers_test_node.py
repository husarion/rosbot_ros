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
import rclpy

from threading import Event
from threading import Thread

from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry


class ControllersTestNode(Node):
    ROSBOT_HARDWARE_PUBLISHERS_RATE = 10.0

    __test__ = False

    def __init__(self, name="test_node"):
        super().__init__(name)
        self.joint_state_msg_event = Event()
        self.odom_msg_event = Event()
        self.imu_msg_event = Event()

    def create_test_subscribers_and_publishers(self):
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/rosbot_base_controller/odom", self.odometry_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, "/imu_broadcaster/imu", self.joint_states_callback, 10
        )

        self.imu_publisher = self.create_publisher(Imu, "_imu/data_raw", 10)

        self.joint_states_publisher = self.create_publisher(
            JointState, "_motors_response", 10
        )

        self.timer = None

    def start_node_thread(self):
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()

    def joint_states_callback(self, data):
        self.joint_state_msg_event.set()

    def odometry_callback(self, data):
        self.odom_msg_event.set()

    def imu_callback(self, data):
        self.imu_msg_event.set()

    def start_publishing_fake_hardware(self):
        self.timer = self.create_timer(
            1.0 / self.ROSBOT_HARDWARE_PUBLISHERS_RATE,
            self.publish_fake_hardware_messages,
        )

    def publish_fake_hardware_messages(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            "fl_wheel_joint",
            "fr_wheel_joint",
            "rl_wheel_joint",
            "rr_wheel_joint",
        ]
        joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]
        joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0]

        self.imu_publisher.publish(imu_msg)
        self.joint_states_publisher.publish(joint_state_msg)
