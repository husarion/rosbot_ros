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

"""Invariants for the joy_node / joy2twist (teleop_twist_joy) parameters."""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope="module")
def joy_config():
    path = os.path.join(get_package_share_directory("rosbot_joy"), "config", "config.yaml")
    assert os.path.isfile(path), f"Missing {path}"
    with open(path) as f:
        return yaml.safe_load(f)["/**"]


def test_publish_stamped_twist_enabled(joy_config):
    # ROS 2 jazzy drive controllers consume TwistStamped, not Twist.
    # Flipping this back to false silently breaks the cmd_vel path.
    params = joy_config["joy2twist"]["ros__parameters"]
    assert params["publish_stamped_twist"] is True


def test_enable_button_required(joy_config):
    # Safety invariant: without require_enable_button the robot drives whenever
    # the stick is moved, which is a hard NO for HW operation.
    params = joy_config["joy2twist"]["ros__parameters"]
    assert params["require_enable_button"] is True
