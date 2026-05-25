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

"""Structural sanity for ekf.yaml — guards the config_dir convention and node identity."""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope="module")
def launch_doc():
    path = os.path.join(get_package_share_directory("rosbot_localization"), "launch", "ekf.yaml")
    assert os.path.isfile(path), f"Missing {path}"
    with open(path) as f:
        return yaml.safe_load(f)


def test_launch_yaml_top_level(launch_doc):
    assert "launch" in launch_doc, "Top-level key 'launch' missing"
    assert isinstance(launch_doc["launch"], list) and launch_doc["launch"], "Empty launch list"


def test_config_dir_arg_is_declared(launch_doc):
    # CLAUDE.md §5 "Configurations": every config-bearing package must accept config_dir.
    arg_names = [item["arg"]["name"] for item in launch_doc["launch"] if "arg" in item]
    assert "config_dir" in arg_names, "config_dir arg missing — breaks the snap config convention"


def test_starts_robot_localization_ekf_node(launch_doc):
    nodes = [item["node"] for item in launch_doc["launch"] if "node" in item]
    assert len(nodes) == 1, f"Expected exactly one node, got {len(nodes)}"
    node = nodes[0]
    assert node["pkg"] == "robot_localization"
    assert node["exec"] == "ekf_node"
