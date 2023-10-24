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
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class BringupTestNode(Node):
    ROSBOT_HARDWARE_PUBLISHERS_RATE = 10.0

    __test__ = False

    def __init__(self, name="test_node"):
        super().__init__(name)
        self.odom_tf_event = Event()

    def create_test_subscribers_and_publishers(self):
        self.imu_publisher = self.create_publisher(Imu, "_imu/data_raw", 10)

        self.joint_states_publisher = self.create_publisher(JointState, "_motors_response", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = None

    def lookup_transform_odom(self):
        try:
            self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            self.odom_tf_event.set()
        except TransformException as ex:
            self.get_logger().error(f"Could not transform odom to base_link: {ex}")

    def start_node_thread(self):
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def start_publishing_fake_hardware(self):
        self.timer = self.create_timer(
            1.0 / self.ROSBOT_HARDWARE_PUBLISHERS_RATE,
            self.timer_callback,
        )

    def timer_callback(self):
        self.publish_fake_hardware_messages()
        self.lookup_transform_odom()

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
