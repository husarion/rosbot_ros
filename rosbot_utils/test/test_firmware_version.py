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

"""Guards CLAUDE.md hard rule #1: FIRMWARE_VERSION must match shipped .bin files."""

import os
import re

import pytest
from ament_index_python.packages import get_package_share_directory
from rosbot_utils.firmware_version import FIRMWARE_VERSION


def test_firmware_version_format():
    assert re.fullmatch(
        r"v\d+\.\d+\.\d+-[a-z]+", FIRMWARE_VERSION
    ), f"FIRMWARE_VERSION='{FIRMWARE_VERSION}' must match 'v<major>.<minor>.<patch>-<distro>'"


@pytest.mark.parametrize("robot_model", ["rosbot", "rosbot_xl"])
def test_firmware_binary_exists_for_version(robot_model):
    firmware_dir = os.path.join(get_package_share_directory("rosbot_utils"), "firmware")
    expected = os.path.join(firmware_dir, f"{robot_model}-{FIRMWARE_VERSION}.bin")
    assert os.path.isfile(expected), (
        f"Missing firmware binary for {robot_model} at {expected}. "
        f"Either bump FIRMWARE_VERSION back or add the matching .bin (see CLAUDE.md §6.1)."
    )
