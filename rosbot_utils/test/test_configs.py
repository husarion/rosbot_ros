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

"""Schema sanity for the per-model configs consumed by rosbot_bringup."""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


@pytest.mark.parametrize("robot_model", ["rosbot_xl"])
def test_config_yaml_parses(robot_model):
    path = os.path.join(
        get_package_share_directory("rosbot_utils"), "config", robot_model, "config.yaml"
    )
    assert os.path.isfile(path), f"Missing {path}"
    with open(path) as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict), f"{path}: top-level must be a mapping"
    assert "/**" in data, f"{path}: expected ROS 2 wildcard key '/**'"
    battery_alert = data["/**"].get("battery_alert")
    assert battery_alert is not None, f"{path}: missing battery_alert section"

    params = battery_alert["ros__parameters"]
    assert 0.0 < params["percentage_threshold"] < 1.0, (
        f"{path}: battery_alert.percentage_threshold must be a fraction in (0, 1) "
        f"(got {params['percentage_threshold']})"
    )
