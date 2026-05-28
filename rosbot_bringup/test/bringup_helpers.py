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

import time
from threading import Event, Thread

import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


def make_bringup_launch_description(**launch_arguments):
    """Build a LaunchDescription that launches ``rosbot_bringup/bringup.yaml``.

    ``launch_arguments`` are forwarded as launch-argument overrides; tests
    typically pass ``hardware_bridge='False'`` to keep the test offline. The
    returned description bundles ``KeepAliveProc`` + ``ReadyToTest`` so
    launch_pytest fixtures stay one-liners.
    """
    bringup_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("rosbot_bringup"), "launch", "bringup.yaml"]),
        launch_arguments=launch_arguments.items(),
    )
    return LaunchDescription([bringup_launch, KeepAliveProc(), ReadyToTest()])


class BringupTestNode(Node):
    ROSBOT_HARDWARE_PUBLISHERS_RATE = 10.0

    __test__ = False

    def __init__(self, name="test_node", namespace=None):
        super().__init__(
            name,
            namespace=namespace,
            cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
        )

        self.joint_state_msg_event = Event()
        self.controller_odom_msg_event = Event()
        self.imu_msg_event = Event()
        self.ekf_odom_msg_event = Event()

        self.ros_spin_thread = None
        self.timer = None

        self.create_test_subscribers_and_publishers()

    def create_test_subscribers_and_publishers(self):
        self.imu_pub = self.create_publisher(Imu, "_imu/data", 10)
        self.joint_pub = self.create_publisher(JointState, "_motors/feedback", 10)

        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_states_callback, 10
        )
        self.controller_odom_sub = self.create_subscription(
            Odometry, "odometry/wheels", self.controller_odometry_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)
        self.ekf_odom_sub = self.create_subscription(
            Odometry, "odometry/filtered", self.ekf_odometry_callback, 10
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

    def joint_states_callback(self, msg: JointState):
        self.joint_state_msg_event.set()

    def controller_odometry_callback(self, msg: Odometry):
        self.controller_odom_msg_event.set()

    def imu_callback(self, msg: Imu):
        self.imu_msg_event.set()

    def ekf_odometry_callback(self, msg: Odometry):
        self.ekf_odom_msg_event.set()

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


def wait_for_all_events(events, timeout):
    start_time = time.time()
    while time.time() - start_time < timeout:
        if all(event.is_set() for event in events):
            return True, []
        time.sleep(0.1)

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

    # 30s instead of 20s leaves a margin on slow CI runners; an 8-second
    # bringup with no margin trips the timeout on the first GC pause.
    msgs_received_flag, not_set_indices = wait_for_all_events(events, timeout=30.0)

    if not msgs_received_flag:
        not_set_event_names = [event_names[i] for i in not_set_indices]
        missing_events = ", ".join(not_set_event_names)
        raise AssertionError(
            f"{robot_name}: Not all expected messages were received. Missing: {missing_events}."
        )

    print(f"{robot_name}: All messages received successfully.")
