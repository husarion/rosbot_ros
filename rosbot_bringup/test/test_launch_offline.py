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

"""Offline schema tests for rosbot_bringup launch files.

The systemic launch_pytest tests in test_bringup.py / test_multirobot.py
require micro-ROS and the real robot, so they cannot run in CI. These
tests cover the schema that does not need a runtime: arg declarations,
include targets, and the snap-relevant config_dir convention.
"""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

ROBOT_MODELS = ["rosbot", "rosbot_xl"]


def _launch(name):
    path = os.path.join(get_package_share_directory("rosbot_bringup"), "launch", name)
    assert os.path.isfile(path), f"Missing {path}"
    with open(path) as f:
        return yaml.safe_load(f)


def _args(doc):
    return {item["arg"]["name"]: item["arg"] for item in doc["launch"] if "arg" in item}


def _includes(doc):
    return [item["include"] for item in doc["launch"] if "include" in item]


def test_bringup_yaml_dispatches_on_robot_model():
    doc = _launch("bringup.yaml")
    args = _args(doc)
    assert "robot_model" in args
    choices = {c["value"] for c in args["robot_model"].get("choice", [])}
    assert choices == {"rosbot", "rosbot_xl"}, f"Unexpected robot_model choices: {choices}"


@pytest.mark.parametrize("model", ROBOT_MODELS)
def test_per_model_launch_declares_required_args(model):
    doc = _launch(f"{model}.yaml")
    args = _args(doc)
    # config_dir (snap convention), namespace (multi-robot),
    # hardware_bridge (HW switch — renamed from `microros` when the MAVLink
    # backend landed, CLAUDE.md §9 2026-05-21), backend (microros|mavlink
    # picker), tf_namespace_bridge (multirobot TF), robot_model (sanity).
    required = {
        "config_dir",
        "namespace",
        "hardware_bridge",
        "backend",
        "tf_namespace_bridge",
        "robot_model",
    }
    missing = required - set(args)
    assert not missing, f"{model}.yaml missing args: {missing}"


@pytest.mark.parametrize("model", ROBOT_MODELS)
def test_per_model_launch_pulls_in_subsystems(model):
    """Each bringup file must include the controller, joy, localization and
    laser_filter subsystems — these are the contract advertised in
    ROS_API.md and the snap config_dir layout."""
    doc = _launch(f"{model}.yaml")
    include_files = " ".join(inc.get("file", "") for inc in _includes(doc))
    for needle in (
        "rosbot_controller",
        "rosbot_joy",
        "rosbot_localization",
        "rosbot_utils",  # laser_filter lives here
    ):
        assert needle in include_files, f"{model}.yaml does not include any launch from {needle}"


def test_rosbot_xl_has_led_strip_arg():
    """ROSbot XL-specific arg (CLAUDE.md §9 2025-04-21). The plain rosbot
    launch must not carry it (no LED strip)."""
    xl_args = _args(_launch("rosbot_xl.yaml"))
    rosbot_args = _args(_launch("rosbot.yaml"))
    assert "led_strip" in xl_args
    assert "led_strip" not in rosbot_args


def test_tf_namespace_bridge_default_is_pass_through():
    """frame_filters=['*'] = pass-through (no filtering). CLAUDE.md §9 2026-05-04."""
    path = os.path.join(
        get_package_share_directory("rosbot_bringup"), "config", "tf_namespace_bridge.yaml"
    )
    with open(path) as f:
        cfg = yaml.safe_load(f)
    filters = cfg["/**"]["tf_namespace_bridge"]["ros__parameters"]["frame_filters"]
    assert filters == ["*"], (
        f"Default frame_filters must be pass-through, got {filters!r}. "
        "Changing the default breaks namespaced TF for downstream consumers."
    )
