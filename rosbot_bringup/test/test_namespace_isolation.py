# Copyright 2026 Husarion sp. z o.o.
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

"""Regression guard for the namespace-audit (Variant B insurance).

Brings the rosbot_xl bringup up with ``microros:=False`` and a fixed
``namespace='test_ns'``, waits for the stack to stabilise, and then asserts:

* No topic / service / node leaks onto the global root ``/`` outside the
  explicit whitelist (TF + parameter_events + rosout). A new upstream
  global must be added to the whitelist on purpose — the test fail message
  points at the leaking name so the reviewer sees what changed.
* A list of critical topics / services exists under ``/test_ns/`` (the
  positive side: not just "nothing leaks" but "the robot's published
  contract is reachable under the prefix").

Scope notes:
* ``microros:=False`` keeps this test CI-runnable (no real driver, no
  micro-ROS agent — base-only hardware faked via :class:`BringupTestNode`).
* ``configuration='manipulation'`` is NOT covered here: bringup.yaml does
  not propagate the arg, and offline manipulation would need the
  ``manipulator_controller`` spawner to talk to a dynamixel bus that is
  not present in CI. Manipulation namespacing is exercised by the HW
  retest sequence in Phase 6 of the namespace-audit plan.
"""

import time

import launch_pytest
import pytest
import rclpy
from bringup_helpers import BringupTestNode
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc

NAMESPACE = "test_ns"
ROBOT_MODEL = "rosbot_xl"

# Topics that legitimately stay on `/` after a namespaced bringup. Adding to
# this set is a deliberate API decision — reviewers should challenge any new
# entry.
ALLOWED_GLOBAL_TOPICS = {
    "/tf",
    "/tf_static",
    "/parameter_events",
    "/rosout",
}

# Services that legitimately stay on `/`. Each running node still exposes its
# own private parameter services under its node namespace, but with bringup
# under /test_ns/ every per-node parameter service should be /test_ns/<node>/...
# — i.e. nothing at the global root after the fix.
ALLOWED_GLOBAL_SERVICES: set[str] = set()

# Critical topics that MUST be present under /<ns>/ after bringup. Limited to
# the base-only stack because ``microros:=False`` + no real HW means the
# manipulator controllers do not spawn in offline mode.
REQUIRED_NS_TOPICS = {
    f"/{NAMESPACE}/joint_states",
    f"/{NAMESPACE}/odometry/filtered",
    f"/{NAMESPACE}/odometry/wheels",
    f"/{NAMESPACE}/imu/data",
    f"/{NAMESPACE}/scan_filtered",
    f"/{NAMESPACE}/cmd_vel",
    f"/{NAMESPACE}/diagnostics",
}

# Critical services that MUST be present under /<ns>/. controller_manager
# services land under /<ns>/ via push_ros_namespace on HW (gz_ros2_control
# uses the URDF <remapping> path covered by Phase 5 sim test). EKF wystawia
# `set_pose` jako relative — z push_ros_namespace ląduje pod /<ns>/set_pose
# (NIE /<ns>/ekf_node/set_pose; HW retest 2026-05-21).
REQUIRED_NS_SERVICES = {
    f"/{NAMESPACE}/controller_manager/list_controllers",
    f"/{NAMESPACE}/controller_manager/switch_controller",
    f"/{NAMESPACE}/set_pose",
}


@launch_pytest.fixture
def generate_test_description():
    rosbot_bringup = FindPackageShare("rosbot_bringup")
    bringup_launch = IncludeLaunchDescription(
        PathJoinSubstitution([rosbot_bringup, "launch", "bringup.yaml"]),
        launch_arguments={
            "microros": "False",
            "namespace": NAMESPACE,
            "robot_model": ROBOT_MODEL,
        }.items(),
    )

    return LaunchDescription(
        [
            bringup_launch,
            KeepAliveProc(),
            ReadyToTest(),
        ]
    )


def _classify(names, ns_prefix, allowed_globals):
    leaks = []
    namespaced = []
    for name in names:
        if name.startswith(ns_prefix):
            namespaced.append(name)
        elif name in allowed_globals:
            continue
        else:
            leaks.append(name)
    return leaks, namespaced


@pytest.mark.launch(fixture=generate_test_description)
def test_namespace_isolation():
    rclpy.init()
    try:
        node = BringupTestNode("test_namespace_isolation", namespace=NAMESPACE)
        node.start_publishing_fake_hardware()
        node.start_node_thread()

        # 15s leaves room for controller spawners + EKF stabilisation on slow
        # CI runners; test_bringup.py uses 30s for the heavier readings loop,
        # we just need name discovery rather than data flow.
        time.sleep(15.0)

        topic_names = [name for name, _ in node.get_topic_names_and_types()]
        service_names = [name for name, _ in node.get_service_names_and_types()]

        ns_prefix = f"/{NAMESPACE}/"

        topic_leaks, ns_topics = _classify(topic_names, ns_prefix, ALLOWED_GLOBAL_TOPICS)
        service_leaks, ns_services = _classify(service_names, ns_prefix, ALLOWED_GLOBAL_SERVICES)

        assert not topic_leaks, (
            "Topic leaks to /: "
            + ", ".join(sorted(topic_leaks))
            + ". Either namespace the source (see namespace-audit plan) or "
            "add the name to ALLOWED_GLOBAL_TOPICS with reviewer sign-off."
        )

        assert not service_leaks, (
            "Service leaks to /: "
            + ", ".join(sorted(service_leaks))
            + ". Either namespace the source (see namespace-audit plan) or "
            "add the name to ALLOWED_GLOBAL_SERVICES with reviewer sign-off."
        )

        ns_topic_set = set(ns_topics)
        missing_topics = REQUIRED_NS_TOPICS - ns_topic_set
        assert not missing_topics, (
            "Required topics missing under /"
            + NAMESPACE
            + "/: "
            + ", ".join(sorted(missing_topics))
        )

        ns_service_set = set(ns_services)
        missing_services = REQUIRED_NS_SERVICES - ns_service_set
        assert not missing_services, (
            "Required services missing under /"
            + NAMESPACE
            + "/: "
            + ", ".join(sorted(missing_services))
        )
    finally:
        rclpy.shutdown()
