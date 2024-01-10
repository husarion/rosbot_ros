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
from sensor_msgs.msg import LaserScan, Image, PointCloud2


class SimulationTestNode(Node):
    __test__ = False
    # The inaccuracies in measurement uncertainties and wheel slippage
    # cause the rosbot_base_controller to determine inaccurate odometry.
    ACCURACY = 0.10  # 10% accuracy

    RANGE_SENSORS_TOPICS = ["range/fl", "range/fr", "range/rl", "range/rr"]
    RANGE_SENSORS_FRAMES = ["fl_range", "fr_range", "rl_range", "rr_range"]

    def __init__(self, name="test_node", namespace=None):
        super().__init__(name, namespace=namespace)

        # Use simulation time to correct run on slow machine
        use_sim_time = rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True
        )
        self.set_parameters([use_sim_time])

        self.VELOCITY_STABILIZATION_DELAY = 3
        self.current_time = 1e-9 * self.get_clock().now().nanoseconds
        self.goal_received_time = 1e-9 * self.get_clock().now().nanoseconds
        self.vel_stabilization_time_event = Event()

        self.v_x = 0.0
        self.v_y = 0.0
        self.v_yaw = 0.0

        self.controller_odom_flag = False
        self.ekf_odom_flag = False
        self.odom_tf_event = Event()
        self.scan_event = Event()
        self.ranges_events = [Event() for _ in range(len(self.RANGE_SENSORS_TOPICS))]
        self.camera_color_event = Event()
        self.camera_points_event = Event()
        self.ros_node_spin_event = Event()

    def clear_odom_flag(self):
        self.controller_odom_flag = False
        self.ekf_odom_flag = False

    def set_destination_speed(self, v_x, v_y, v_yaw):
        self.clear_odom_flag()
        self.v_x = v_x
        self.v_y = v_y
        self.v_yaw = v_yaw
        self.goal_received_time = 1e-9 * self.get_clock().now().nanoseconds
        self.vel_stabilization_time_event.clear()

    def create_test_subscribers_and_publishers(self):
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.controller_odom_sub = self.create_subscription(
            Odometry, "rosbot_base_controller/odom", self.controller_callback, 10
        )

        self.ekf_odom_sub = self.create_subscription(
            Odometry, "odometry/filtered", self.ekf_callback, 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )

        self.range_subs = []
        for range_topic_name in self.RANGE_SENSORS_TOPICS:
            sub = self.create_subscription(
                LaserScan, range_topic_name, self.ranges_callback, 10
            )
            self.range_subs.append(sub)

        self.camera_color_sub = self.create_subscription(
            Image, "camera/image", self.camera_image_callback, 10
        )

        self.camera_points_sub = self.create_subscription(
            PointCloud2, "camera/points", self.camera_points_callback, 10
        )

        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def start_node_thread(self):
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()

    def is_twist_ok(self, twist: Twist):
        def are_close_to_each_other(
            current_value, dest_value, tolerance=self.ACCURACY, eps=0.01
        ):
            acceptable_range = dest_value * tolerance
            return abs(current_value - dest_value) <= acceptable_range + eps

        x_ok = are_close_to_each_other(twist.linear.x, self.v_x)
        y_ok = are_close_to_each_other(twist.linear.y, self.v_y)
        yaw_ok = are_close_to_each_other(twist.angular.z, self.v_yaw)

        return x_ok and y_ok and yaw_ok

    def controller_callback(self, data: Odometry):
        self.get_logger().debug(f"Received twist from controller: {data.twist.twist}")
        self.controller_odom_flag = self.is_twist_ok(data.twist.twist)

    def ekf_callback(self, data: Odometry):
        self.get_logger().debug(f"Received twist filtered: {data.twist.twist}")

        self.odom_tf_event.set()
        self.ekf_odom_flag = self.is_twist_ok(data.twist.twist)

    def timer_callback(self):
        self.publish_cmd_vel_messages()

        self.current_time = 1e-9 * self.get_clock().now().nanoseconds
        if (
            self.current_time
            > self.goal_received_time + self.VELOCITY_STABILIZATION_DELAY
        ):
            self.vel_stabilization_time_event.set()

    def scan_callback(self, data: LaserScan):
        self.get_logger().debug(f"Received scan length: {len(data.ranges)}")
        if data.ranges:
            self.scan_event.set()

    def ranges_callback(self, data: LaserScan):
        index = self.RANGE_SENSORS_FRAMES.index(data.header.frame_id)
        if len(data.ranges) == 1:
            self.ranges_events[index].set()

    def camera_image_callback(self, data: Image):
        if data.data:
            self.camera_color_event.set()

    def camera_points_callback(self, data: PointCloud2):
        if data.data:
            self.camera_points_event.set()

    def publish_cmd_vel_messages(self):
        twist_msg = Twist()

        twist_msg.linear.x = self.v_x
        twist_msg.linear.y = self.v_y
        twist_msg.angular.z = self.v_yaw

        self.get_logger().debug(f"Publishing twist: {twist_msg}")
        self.cmd_vel_publisher.publish(twist_msg)
