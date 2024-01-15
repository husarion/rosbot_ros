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
        use_sim_time = rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([use_sim_time])

        self.VELOCITY_STABILIZATION_DELAY = 3
        self.current_time = 1e-9 * self.get_clock().now().nanoseconds
        self.goal_received_time = 1e-9 * self.get_clock().now().nanoseconds
        self.vel_stabilization_time_event = Event()

        self.v_x = 0.0
        self.v_y = 0.0
        self.v_yaw = 0.0
        self.twist = None  # Debug info

        self.controller_odom_flag = False
        self.ekf_odom_flag = False
        self.odom_tf_event = Event()
        self.scan_event = Event()
        self.ranges_events = [Event() for _ in range(len(self.RANGE_SENSORS_TOPICS))]
        self.camera_color_event = Event()
        self.camera_points_event = Event()

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

        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)

        self.range_subs = []
        for range_topic_name in self.RANGE_SENSORS_TOPICS:
            sub = self.create_subscription(LaserScan, range_topic_name, self.ranges_callback, 10)
            self.range_subs.append(sub)

        self.camera_color_sub = self.create_subscription(
            Image, "camera/image", self.camera_image_callback, 10
        )

        self.camera_points_sub = self.create_subscription(
            PointCloud2, "camera/points", self.camera_points_callback, 10
        )

        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)

    def start_node_thread(self):
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def is_twist_ok(self, twist: Twist):
        def are_close_to_each_other(current_value, dest_value, tolerance=self.ACCURACY, eps=0.01):
            acceptable_range = dest_value * tolerance
            return abs(current_value - dest_value) <= acceptable_range + eps

        x_ok = are_close_to_each_other(twist.linear.x, self.v_x)
        y_ok = are_close_to_each_other(twist.linear.y, self.v_y)
        yaw_ok = are_close_to_each_other(twist.angular.z, self.v_yaw)

        return x_ok and y_ok and yaw_ok

    def controller_callback(self, data: Odometry):
        self.get_logger().debug(f"Received twist from controller: {data.twist.twist}")
        self.controller_odom_flag = self.is_twist_ok(data.twist.twist)
        self.twist = data.twist.twist

    def ekf_callback(self, data: Odometry):
        self.get_logger().debug(f"Received twist filtered: {data.twist.twist}")

        self.odom_tf_event.set()
        self.ekf_odom_flag = self.is_twist_ok(data.twist.twist)

    def timer_callback(self):
        self.publish_cmd_vel_messages()

        self.current_time = 1e-9 * self.get_clock().now().nanoseconds
        if self.current_time > self.goal_received_time + self.VELOCITY_STABILIZATION_DELAY:
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


def x_speed_test(node, v_x=0.0, v_y=0.0, v_yaw=0.0, robot_name="ROSbot"):
    node.set_destination_speed(v_x, v_y, v_yaw)

    assert node.vel_stabilization_time_event.wait(timeout=120.0), (
        f"{robot_name}: The simulation is running slowly or has crashed! The time elapsed"
        " since setting the target speed is:"
        f" {(node.current_time - node.goal_received_time):.1f}."
    )
    assert node.controller_odom_flag, (
        f"{robot_name}: does not move properly in x direction. Check"
        f" rosbot_base_controller! Twist: {node.twist}"
    )
    assert node.ekf_odom_flag, (
        f"{robot_name}: does not move properly in x direction. Check ekf_filter_node!"
        f" Twist: {node.twist}"
    )


def y_speed_test(node, v_x=0.0, v_y=0.0, v_yaw=0.0, robot_name="ROSbot"):
    node.set_destination_speed(v_x, v_y, v_yaw)

    assert node.vel_stabilization_time_event.wait(timeout=120.0), (
        f"{robot_name}: The simulation is running slowly or has crashed! The time elapsed"
        " since setting the target speed is:"
        f" {(node.current_time - node.goal_received_time):.1f}."
    )
    assert node.controller_odom_flag, (
        f"{robot_name} does not move properly in y direction. Check"
        f" rosbot_base_controller! Twist: {node.twist}"
    )
    assert node.ekf_odom_flag, (
        f"{robot_name} does not move properly in y direction. Check ekf_filter_node!"
        f" Twist: {node.twist}"
    )


def yaw_speed_test(node, v_x=0.0, v_y=0.0, v_yaw=0.0, robot_name="ROSbot"):
    node.set_destination_speed(v_x, v_y, v_yaw)

    assert node.vel_stabilization_time_event.wait(timeout=120.0), (
        f"{robot_name}: The simulation is running slowly or has crashed! The time elapsed"
        " since setting the target speed is:"
        f" {(node.current_time - node.goal_received_time):.1f}."
    )
    assert (
        node.controller_odom_flag
    ), f"{robot_name} does not rotate properly. Check rosbot_base_controller! Twist: {node.twist}"
    assert (
        node.ekf_odom_flag
    ), f"{robot_name} does not rotate properly. Check ekf_filter_node! Twist: {node.twist}"


def sensors_readings_test(node, robot_name="ROSbot"):
    flag = node.scan_event.wait(timeout=20.0)
    assert flag, f"{robot_name}'s lidar does not work properly!"

    for i in range(len(node.RANGE_SENSORS_TOPICS)):
        flag = node.ranges_events[i].wait(timeout=20.0)
        assert (
            flag
        ), f"{robot_name}'s range sensor {node.RANGE_SENSORS_TOPICS[i]} does not work properly!"

    flag = node.camera_color_event.wait(timeout=20.0)
    assert flag, f"{robot_name}'s camera color image does not work properly!"

    flag = node.camera_points_event.wait(timeout=20.0)
    assert flag, f"{robot_name}'s camera point cloud does not work properly!"


def tf_test(node, robot_name="ROSbot"):
    flag = node.odom_tf_event.wait(timeout=20.0)
    assert flag, (
        f"{robot_name}: expected odom to base_link tf but it was not received. Check"
        " robot_localization!"
    )


def diff_test(node, robot_name="ROSbot"):
    sensors_readings_test(node, robot_name)
    # 0.9 m/s and 3.0 rad/s are controller's limits defined in
    # rosbot_controller/config/mecanum_drive_controller.yaml
    x_speed_test(node, v_x=0.9, robot_name=robot_name)
    yaw_speed_test(node, v_yaw=3.0, robot_name=robot_name)


def mecanum_test(node, robot_name="ROSbot"):
    sensors_readings_test(node, robot_name)
    # 0.9 m/s and 3.0 rad/s are controller's limits defined in
    # rosbot_controller/config/mecanum_drive_controller.yaml
    x_speed_test(node, v_x=0.9, robot_name=robot_name)
    y_speed_test(node, v_y=0.9, robot_name=robot_name)
    yaw_speed_test(node, v_yaw=3.0, robot_name=robot_name)
