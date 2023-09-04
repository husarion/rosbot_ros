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
import tf_transformations


class SimulationTestNode(Node):
    __test__ = False

    def __init__(
        self,
        name="test_node",
        goal_x_distance=0.0,
        goal_y_distance=0.0,
        goal_theta_angle=0.0,
    ):
        super().__init__(name)
        self.goal_x_distance = goal_x_distance
        self.goal_y_distance = goal_y_distance
        self.goal_theta_angle = goal_theta_angle

        self.goal_x_event = Event()
        self.goal_y_event = Event()
        self.goal_theta_event = Event()

    def create_test_subscribers_and_publishers(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_callback, 10
        )
        self.timer = None

    def start_node_thread(self):
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()
        self.timer = self.create_timer(1.0 / 10.0, self.publish_cmd_vel_messages)

    def odometry_callback(self, data: Odometry):
        pose = data.pose.pose
        q = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        r, p, y = tf_transformations.euler_from_quaternion(q)
        print(pose.position.x)
        if pose.position.x > self.goal_x_distance:
            self.goal_x_event.set()

        if pose.position.y > self.goal_y_distance:
            self.goal_y_event.set()

        if y > self.goal_x_distance:
            self.goal_theta_event.set()

    def publish_cmd_vel_messages(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.goal_x_distance / 2.0
        twist_msg.linear.y = self.goal_y_distance / 2.0
        twist_msg.angular.z = self.goal_theta_angle / 1.0

        self.cmd_vel_publisher.publish(twist_msg)
