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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SimulationTestNode(Node):
    __test__ = False
    # The inaccuracies in measurement uncertainties and wheel slippage
    # cause the rosbot_xl_base_controller to determine inaccurate odometry.
    ACCURACY = 0.10  # 10% accuracy

    def __init__(self, name="test_node"):
        super().__init__(name)
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_yaw = 0.0

        self.controller_odom_event = Event()
        self.ekf_odom_event = Event()
        self.odom_tf_event = Event()
        self.scan_event = Event()

    def clear_odom_events(self):
        self.controller_odom_event.clear()
        self.ekf_odom_event.clear()

    def set_destination_speed(self, v_x, v_y, v_yaw):
        self.clear_odom_events()
        self.v_x = v_x
        self.v_y = v_y
        self.v_yaw = v_yaw

    def create_test_subscribers_and_publishers(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.controller_odom_sub = self.create_subscription(
            Odometry, "/rosbot_base_controller/odom", self.controller_callback, 10
        )

        self.ekf_odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.ekf_callback, 10
        )

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = None

    def start_node_thread(self):
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def is_twist_ok(self, twist: Twist):
        x_ok, y_ok, yaw_ok = False, False, False

        def are_close_to_zero(value1, value2, eps=0.05):
            return abs(value1) < eps and abs(value2) < eps

        def are_close_to_each_other(true_value, dest_value, tolerance=self.ACCURACY):
            acceptable_range = true_value * (tolerance / 100)
            return abs(true_value - dest_value) <= acceptable_range

        if (are_close_to_zero(self.v_x, twist.linear.x)) or are_close_to_each_other(
            twist.linear.x, self.v_x
        ):
            x_ok = True

        if are_close_to_zero(self.v_y, twist.linear.y) or are_close_to_each_other(
            twist.linear.y, self.v_y
        ):
            y_ok = True

        if are_close_to_zero(self.v_yaw, twist.angular.z) or are_close_to_each_other(
            twist.angular.z, self.v_yaw
        ):
            yaw_ok = True

        return x_ok and y_ok and yaw_ok

    def controller_callback(self, data: Odometry):
        if self.is_twist_ok(data.twist.twist):
            self.controller_odom_event.set()

    def ekf_callback(self, data: Odometry):
        if self.is_twist_ok(data.twist.twist):
            self.ekf_odom_event.set()

    def lookup_transform_odom(self):
        try:
            self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
            self.odom_tf_event.set()
        except TransformException as ex:
            self.get_logger().error(f"Could not transform odom to base_link: {ex}")

    def timer_callback(self):
        self.publish_cmd_vel_messages()
        self.lookup_transform_odom()

    def scan_callback(self, data: LaserScan):
        if data.ranges:
            self.scan_event.set()

    def publish_cmd_vel_messages(self):
        twist_msg = Twist()

        twist_msg.linear.x = self.v_x
        twist_msg.linear.y = self.v_y
        twist_msg.angular.z = self.v_yaw

        self.cmd_vel_publisher.publish(twist_msg)
