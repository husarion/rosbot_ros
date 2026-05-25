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

"""Schema sanity for the per-model laser_filter configs consumed by rosbot_bringup."""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


@pytest.mark.parametrize("robot_model", ["rosbot", "rosbot_xl"])
def test_config_yaml_parses(robot_model):
    path = os.path.join(
        get_package_share_directory("rosbot_utils"), "config", robot_model, "config.yaml"
    )
    assert os.path.isfile(path), f"Missing {path}"
    with open(path) as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict), f"{path}: top-level must be a mapping"
    assert "/**" in data, f"{path}: expected ROS 2 wildcard key '/**'"
    laser_filter = data["/**"].get("laser_filter")
    assert laser_filter is not None, f"{path}: missing laser_filter section"

    params = laser_filter["ros__parameters"]["filter1"]["params"]
    assert (
        params["box_frame"] == "base_link"
    ), f"{path}: laser_filter must crop relative to base_link, not odom/map"
    for axis in ("x", "y", "z"):
        assert params[f"max_{axis}"] > params[f"min_{axis}"], (
            f"{path}: filter1.params {axis}-range max must exceed min "
            f"(got max={params[f'max_{axis}']}, min={params[f'min_{axis}']})"
        )
