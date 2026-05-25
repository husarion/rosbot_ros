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

"""Offline schema tests for rosbot_gazebo launch + bridge configs."""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


def _share(*parts):
    return os.path.join(get_package_share_directory("rosbot_gazebo"), *parts)


def _launch(name):
    path = _share("launch", name)
    assert os.path.isfile(path), f"Missing {path}"
    with open(path) as f:
        return yaml.safe_load(f)


def _args(doc):
    return {item["arg"]["name"]: item["arg"] for item in doc["launch"] if "arg" in item}


def test_simulation_yaml_declares_config_dir():
    """Snap config_dir convention applies to simulation too."""
    args = _args(_launch("simulation.yaml"))
    assert "config_dir" in args


def test_spawn_robot_args_cover_pose_and_config():
    args = _args(_launch("spawn_robot.yaml"))
    required = {
        "robot_model",
        "namespace",
        "configuration",
        "arm_activate",
        "tf_namespace_bridge",
        "rviz",
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
    }
    missing = required - set(args)
    assert not missing, f"spawn_robot.yaml missing args: {missing}"


def test_spawn_robot_configuration_choices_match_rosbot_description():
    """configuration arg must match rosbot_description/config/rosbot_xl/*.yaml."""
    args = _args(_launch("spawn_robot.yaml"))
    spawn_choices = {c["value"] for c in args["configuration"].get("choice", [])}
    yaml_dir = os.path.join(
        get_package_share_directory("rosbot_description"), "config", "rosbot_xl"
    )
    yaml_choices = {os.path.splitext(f)[0] for f in os.listdir(yaml_dir) if f.endswith(".yaml")}
    assert spawn_choices == yaml_choices, (
        f"configuration choices ({spawn_choices}) drifted from "
        f"rosbot_description/config/rosbot_xl/ ({yaml_choices})"
    )


def test_spawn_robot_robot_model_choices():
    args = _args(_launch("spawn_robot.yaml"))
    choices = {c["value"] for c in args["robot_model"].get("choice", [])}
    assert choices == {"rosbot", "rosbot_xl"}


@pytest.mark.parametrize(
    "config_file,top_level_check",
    [
        ("gz_bridge.yaml", "/clock"),
        # <namespace>/ is sed-substituted by spawn_robot.yaml.
        ("rosbot_bridge.yaml", "<namespace>"),
    ],
)
def test_bridge_configs_parse(config_file, top_level_check):
    path = _share("config", config_file)
    with open(path) as f:
        data = yaml.safe_load(f)
    assert isinstance(data, list), f"{config_file}: expected a list of bridge entries"
    serialized = yaml.safe_dump(data)
    assert top_level_check in serialized, f"{config_file} no longer references {top_level_check!r}"


def test_rosbot_bridge_has_namespace_placeholder():
    """Losing <namespace>/ breaks the sed step in spawn_robot.yaml."""
    with open(_share("config", "rosbot_bridge.yaml")) as f:
        raw = f.read()
    assert "<namespace>/" in raw, (
        "rosbot_bridge.yaml lost <namespace>/ placeholder — sed in spawn_robot "
        "will produce malformed topic names for namespaced robots."
    )


def test_tf_namespace_bridge_default_is_pass_through():
    """frame_filters=['*'] = pass-through. CLAUDE.md §9 2026-05-04."""
    with open(_share("config", "tf_namespace_bridge.yaml")) as f:
        cfg = yaml.safe_load(f)
    filters = cfg["/**"]["tf_namespace_bridge"]["ros__parameters"]["frame_filters"]
    assert filters == ["*"]
