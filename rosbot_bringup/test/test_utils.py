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

import math
import random
import time
from threading import Event, Thread

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan


class BringupTestNode(Node):
    ROSBOT_HARDWARE_PUBLISHERS_RATE = 10.0

    __test__ = False

    def __init__(self, name="test_node", namespace=None):
        super().__init__(name, namespace=namespace)
        self.joint_state_msg_event = Event()
        self.controller_odom_msg_event = Event()
        self.imu_msg_event = Event()
        self.ekf_odom_msg_event = Event()
        self.scan_filter_event = Event()

        self.ros_spin_thread = None
        self.timer = None

    def create_test_subscribers_and_publishers(self):
        self.imu_pub = self.create_publisher(Imu, "/_imu/data_raw", 10)
        self.joint_pub = self.create_publisher(JointState, "/_motors_response", 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)

        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_states_callback, 10
        )
        self.controller_odom_sub = self.create_subscription(
            Odometry, "rosbot_base_controller/odom", self.controller_odometry_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "imu_broadcaster/imu", self.imu_callback, 10)
        self.ekf_odom_sub = self.create_subscription(
            Odometry, "odometry/filtered", self.ekf_odometry_callback, 10
        )
        self.scan_filtered_sub = self.create_subscription(
            LaserScan, "scan_filtered", self.filtered_scan_callback, 10
        )

    def start_node_thread(self):
        if not self.ros_spin_thread:
            self.ros_spin_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
            self.ros_spin_thread.start()

    def start_publishing_fake_hardware(self):
        if not self.timer:
            self.timer = self.create_timer(
                1.0 / self.ROSBOT_HARDWARE_PUBLISHERS_RATE,
                self.timer_callback,
            )

    def timer_callback(self):
        self.publish_fake_hardware_messages()
        self.publish_scan()

    def joint_states_callback(self, msg: JointState):
        self.joint_state_msg_event.set()

    def controller_odometry_callback(self, msg: Odometry):
        self.controller_odom_msg_event.set()

    def imu_callback(self, msg: Imu):
        self.imu_msg_event.set()

    def ekf_odometry_callback(self, msg: Odometry):
        self.ekf_odom_msg_event.set()

    def filtered_scan_callback(self, msg: LaserScan):
        if len(msg.ranges) > 0:
            self.scan_filter_event.set()

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

    def publish_scan(self):
        msg = LaserScan()
        msg.header.frame_id = "laser"
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 0.05
        msg.time_increment = 0.1
        msg.scan_time = 0.1
        msg.range_min = 0.0
        msg.range_max = 10.0

        # fill ranges from 0.0m to 1.0m
        msg.ranges = [random.random() for _ in range(int(msg.angle_max / msg.angle_increment))]
        self.scan_pub.publish(msg)


def wait_for_all_events(events, timeout):
    start_time = time.time()
    while time.time() - start_time < timeout:
        if all(event.is_set() for event in events):
            return True, []  # All events have been set
        time.sleep(0.1)  # Short interval between checks

    # Identify which events were not set
    not_set_events = [i for i, event in enumerate(events) if not event.is_set()]
    return False, not_set_events


def readings_data_test(node, robot_name="ROSbot"):
    events = [
        node.joint_state_msg_event,
        node.controller_odom_msg_event,
        node.imu_msg_event,
        node.ekf_odom_msg_event,
    ]

    event_names = [
        "JointStates",
        "Controller Odometry",
        "IMU",
        "EKF Odometry",
    ]

    msgs_received_flag, not_set_indices = wait_for_all_events(events, timeout=20.0)

    if not msgs_received_flag:
        not_set_event_names = [event_names[i] for i in not_set_indices]
        missing_events = ", ".join(not_set_event_names)
        raise AssertionError(
            f"{robot_name}: Not all expected messages were received. Missing: {missing_events}."
        )

    print(f"{robot_name}: All messages received successfully.")
