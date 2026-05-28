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

"""Regression guard: no topic/service leaks to `/` outside the global allowlist
after namespaced bringup, and required topics/services present under `/<ns>/`.
"""

import time

import launch_pytest
import pytest
import rclpy
from bringup_helpers import BringupTestNode, make_bringup_launch_description

NAMESPACE = "test_ns"
ROBOT_MODEL = "rosbot_xl"

# Adding entries here is a deliberate API decision — challenge in review.
ALLOWED_GLOBAL_TOPICS = {
    "/tf",
    "/tf_static",
    "/parameter_events",
    "/rosout",
}

ALLOWED_GLOBAL_SERVICES: set[str] = set()

# Base-only stack — `hardware_bridge:=False` skips manipulator spawners.
# `scan` comes from the hardware/sim lidar bridge (disabled here), so it is
# intentionally absent offline and not listed.
REQUIRED_NS_TOPICS = {
    f"/{NAMESPACE}/joint_states",
    f"/{NAMESPACE}/odometry/filtered",
    f"/{NAMESPACE}/odometry/wheels",
    f"/{NAMESPACE}/imu/data",
    f"/{NAMESPACE}/cmd_vel",
    f"/{NAMESPACE}/diagnostics",
}

# EKF's `set_pose` lands at `/<ns>/set_pose` (NOT `/<ns>/ekf_node/set_pose`).
REQUIRED_NS_SERVICES = {
    f"/{NAMESPACE}/controller_manager/list_controllers",
    f"/{NAMESPACE}/controller_manager/switch_controller",
    f"/{NAMESPACE}/set_pose",
}


@launch_pytest.fixture
def generate_test_description():
    return make_bringup_launch_description(
        hardware_bridge="False",
        namespace=NAMESPACE,
        robot_model=ROBOT_MODEL,
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

        # 15s = controller spawners + EKF stabilisation (name discovery only).
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
