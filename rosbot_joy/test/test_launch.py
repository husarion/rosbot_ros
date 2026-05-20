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

"""Structural sanity for joy.yaml — config_dir convention, node identities, cmd_vel remap."""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope="module")
def launch_doc():
    path = os.path.join(get_package_share_directory("rosbot_joy"), "launch", "joy.yaml")
    assert os.path.isfile(path), f"Missing {path}"
    with open(path) as f:
        return yaml.safe_load(f)


def _args(doc):
    return {item["arg"]["name"]: item["arg"] for item in doc["launch"] if "arg" in item}


def _nodes(doc):
    return [item["node"] for item in doc["launch"] if "node" in item]


def test_config_dir_arg_declared(launch_doc):
    # CLAUDE.md §5: every config-bearing package must accept config_dir for the snap.
    assert "config_dir" in _args(launch_doc)


def test_joy_vel_defaults_to_cmd_vel(launch_doc):
    # Cross-package contract: drive controller listens on cmd_vel.
    args = _args(launch_doc)
    assert "joy_vel" in args
    assert args["joy_vel"]["default"] == "cmd_vel"


def test_starts_joy_node_and_teleop_twist_joy(launch_doc):
    nodes = _nodes(launch_doc)
    pkg_exec = {(n["pkg"], n["exec"]) for n in nodes}
    assert ("joy", "joy_node") in pkg_exec, "Missing joy/joy_node"
    assert ("teleop_twist_joy", "teleop_node") in pkg_exec, "Missing teleop_twist_joy/teleop_node"


def test_cmd_vel_remap_to_joy_vel_arg(launch_doc):
    teleop = next(n for n in _nodes(launch_doc) if n["pkg"] == "teleop_twist_joy")
    remaps = teleop.get("remap", [])
    assert any(
        r.get("from") == "/cmd_vel" and "joy_vel" in r.get("to", "") for r in remaps
    ), "teleop_twist_joy must remap /cmd_vel onto the joy_vel arg"
