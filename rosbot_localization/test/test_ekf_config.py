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

"""Schema invariants for robot_localization's EKF param file."""

import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope="module")
def ekf_params():
    path = os.path.join(
        get_package_share_directory("rosbot_localization"), "config", "config.yaml"
    )
    assert os.path.isfile(path), f"Missing {path}"
    with open(path) as f:
        data = yaml.safe_load(f)
    assert "/**" in data and "ekf_node" in data["/**"], "Expected '/**.ekf_node' wildcard node"
    return data["/**"]["ekf_node"]["ros__parameters"]


def test_planar_two_d_mode(ekf_params):
    assert ekf_params["two_d_mode"] is True, "ROSbot is planar — two_d_mode must be enabled"


def test_world_frame_matches_odom_frame(ekf_params):
    # CLAUDE.md §8: EKF publishes odom -> base_link, world_frame must equal odom_frame.
    assert ekf_params["world_frame"] == ekf_params["odom_frame"]


def test_publish_tf_enabled(ekf_params):
    # Drive controller has enable_odom_tf=false; EKF must own the odom -> base_link broadcast.
    assert ekf_params["publish_tf"] is True


def test_fuses_wheel_odom_and_imu(ekf_params):
    # CLAUDE.md §8: EKF fuses odometry/wheels + imu/data into odometry/filtered.
    assert ekf_params["odom0"] == "odometry/wheels"
    assert ekf_params["imu0"] == "imu/data"


@pytest.mark.parametrize("key", ["odom0_config", "imu0_config"])
def test_sensor_config_shape(ekf_params, key):
    cfg = ekf_params[key]
    assert isinstance(cfg, list) and len(cfg) == 15, f"{key} must be a 15-bool mask"
    assert all(isinstance(v, bool) for v in cfg), f"{key} entries must be bools"


def test_process_noise_covariance_is_15x15(ekf_params):
    cov = ekf_params["process_noise_covariance"]
    assert len(cov) == 15 * 15, f"process_noise_covariance must be 15x15 (got {len(cov)})"
