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

"""Offline schema tests (arg declarations, include targets, config_dir convention)."""

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


def _nodes(doc):
    return [item["node"] for item in doc["launch"] if "node" in item]


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
    # hardware_bridge (HW switch), tf_namespace_bridge (multirobot TF),
    # robot_model (sanity).
    required = {
        "config_dir",
        "namespace",
        "hardware_bridge",
        "tf_namespace_bridge",
        "robot_model",
        "asset_server",
    }
    missing = required - set(args)
    assert not missing, f"{model}.yaml missing args: {missing}"


@pytest.mark.parametrize("model", ROBOT_MODELS)
def test_per_model_launch_pulls_in_subsystems(model):
    """Each bringup must include controller / joy / localization (ROS_API.md contract)."""
    doc = _launch(f"{model}.yaml")
    include_files = " ".join(inc.get("file", "") for inc in _includes(doc))
    for needle in (
        "rosbot_controller",
        "rosbot_joy",
        "rosbot_localization",
    ):
        assert needle in include_files, f"{model}.yaml does not include any launch from {needle}"


@pytest.mark.parametrize("model", ROBOT_MODELS)
def test_per_model_launch_has_asset_server_node(model):
    """husarion_asset_server runs inside the driver launch, gated by the
    `asset_server` arg — no separate snap daemon (ARCHITECTURE.md)."""
    nodes = [
        n for n in _nodes(_launch(f"{model}.yaml")) if n.get("pkg") == "husarion_asset_server"
    ]
    assert len(nodes) == 1, f"{model}.yaml must have exactly one husarion_asset_server node"
    assert nodes[0].get("exec") == "asset_server"
    assert nodes[0].get("if") == "$(var asset_server)"


def test_rosbot_xl_has_led_strip_arg():
    """ROSbot XL-only arg (CLAUDE.md §9 2025-04-21)."""
    xl_args = _args(_launch("rosbot_xl.yaml"))
    rosbot_args = _args(_launch("rosbot.yaml"))
    assert "led_strip" in xl_args
    assert "led_strip" not in rosbot_args


def test_rosbot_xl_battery_alert_is_gated_and_xl_only():
    """The alert drives the XL's on-board speaker, so it must stay off rosbot.yaml
    and stay switchable — the snap maps driver.battery-alert onto this arg."""
    xl_doc = _launch("rosbot_xl.yaml")
    xl_args = _args(xl_doc)
    assert "battery_alert" in xl_args
    assert xl_args["battery_alert"]["default"] == "True"
    assert "battery_alert" not in _args(_launch("rosbot.yaml"))

    includes = [inc for inc in _includes(xl_doc) if "battery_alert.yaml" in inc.get("file", "")]
    assert len(includes) == 1, "rosbot_xl.yaml must include battery_alert.yaml exactly once"
    assert includes[0].get("if") == '$(eval \'"$(var battery_alert)" == "True"\')'


def test_battery_alert_launch_is_rosbot_xl_only():
    """battery_alert.yaml builds its config path from robot_model, and only
    rosbot_xl has a config directory (it is the model with the speaker). An empty
    default resolved to config//config.yaml, and offering `rosbot` as a choice
    would resolve to a config/rosbot/config.yaml that does not exist."""
    share = get_package_share_directory("rosbot_utils")
    with open(os.path.join(share, "launch", "battery_alert.yaml")) as f:
        args = _args(yaml.safe_load(f))

    assert args["robot_model"]["default"] == "rosbot_xl"
    choices = {c["value"] for c in args["robot_model"].get("choice", [])}
    assert choices == {"rosbot_xl"}, f"battery_alert must stay XL-only, got {choices}"

    for model in choices:
        config = os.path.join(share, "config", model, "config.yaml")
        assert os.path.isfile(config), f"Missing {config} for allowed robot_model {model}"


def test_tf_namespace_bridge_default_is_pass_through():
    """frame_filters=['*'] = pass-through (no filtering). CLAUDE.md §9 2026-05-04.

    Config now lives in rosbot_description (single owner — see CLAUDE.md §9
    2026-08-12, was duplicated identically across 3 packages).
    """
    path = os.path.join(
        get_package_share_directory("rosbot_description"), "config", "tf_namespace_bridge.yaml"
    )
    with open(path) as f:
        cfg = yaml.safe_load(f)
    filters = cfg["/**"]["tf_namespace_bridge"]["ros__parameters"]["frame_filters"]
    assert filters == ["*"], (
        f"Default frame_filters must be pass-through, got {filters!r}. "
        "Changing the default breaks namespaced TF for downstream consumers."
    )
