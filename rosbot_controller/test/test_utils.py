# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from threading import Event, Thread

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


class ControllersTestNode(Node):
    ROSBOT_HARDWARE_PUBLISHERS_RATE = 10.0

    __test__ = False

    def __init__(self, name="test_node", namespace=None):
        super().__init__(name, namespace=namespace)
        self.joint_state_msg_event = Event()
        self.odom_msg_event = Event()
        self.imu_msg_event = Event()

    def create_test_subscribers_and_publishers(self):
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_states_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "rosbot_base_controller/odom", self.odometry_callback, 10
        )

        self.imu_sub = self.create_subscription(Imu, "imu_broadcaster/imu", self.imu_callback, 10)

        # TODO: @delihus namespaces have not been implemented in microros yet
        self.imu_pub = self.create_publisher(Imu, "/_imu/data_raw", 10)

        self.joint_pub = self.create_publisher(JointState, "/_motors_response", 10)

        self.timer = None

    def start_node_thread(self):
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
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

    # TODO: @delihus namespaces have not been implemented in microros yet
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

        self.imu_pub.publish(imu_msg)
        self.joint_pub.publish(joint_state_msg)


def controller_readings_test(node, robot_name="ROSbot"):
    msgs_received_flag = node.joint_state_msg_event.wait(10.0)
    assert msgs_received_flag, (
        f"{robot_name}: Expected JointStates message but it was not received. Check "
        "joint_state_broadcaster!"
    )
    msgs_received_flag = node.odom_msg_event.wait(10.0)
    assert msgs_received_flag, (
        f"{robot_name}: Expected Odom message but it was not received. Check "
        "rosbot_base_controller!"
    )
    msgs_received_flag = node.imu_msg_event.wait(10.0)
    assert (
        msgs_received_flag
    ), f"{robot_name}: Expected Imu message but it was not received. Check imu_broadcaster!"
