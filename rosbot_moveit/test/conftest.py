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

"""Shared helpers for rosbot_moveit pytest suite.

`conftest.py` is auto-discovered by pytest; fixtures and helpers defined here
are available to every test_*.py in this directory without imports beyond
``from conftest import …``.
"""

import os
import xml.etree.ElementTree as ET
from functools import lru_cache

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


def share(rel_path: str) -> str:
    """Resolve ``rel_path`` against ``rosbot_moveit``'s install share dir."""
    return os.path.join(get_package_share_directory("rosbot_moveit"), rel_path)


@lru_cache(maxsize=None)
def load_yaml(rel_path: str):
    """Parse a YAML config from the rosbot_moveit share dir (cached)."""
    with open(share(rel_path)) as f:
        return yaml.safe_load(f)


@lru_cache(maxsize=None)
def srdf_root():
    """Return the parsed ``rosbot_xl.srdf`` root element (cached)."""
    return ET.parse(share("config/rosbot_xl.srdf")).getroot()


@pytest.fixture(scope="session")
def moveit_share():
    """Fixture form of :func:`share` for tests that prefer fixture injection."""
    return share
