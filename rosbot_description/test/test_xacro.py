# Copyright 2024 Husarion sp. z o.o.
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

"""xacro -> URDF for every supported (robot_model, mecanum, configuration) combo.

Beyond parsing, the resulting URDF is required to declare at least one link
and one joint, and the manipulator must show up if and only if the rosbot_xl
configuration starts with 'manipulation'.
"""

import os

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

# rosbot has no `configuration` arg; rosbot_xl uses one of these to pick the
# components yaml in rosbot_description/config/rosbot_xl/.
ROSBOT_XL_CONFIGURATIONS = [
    "basic",
    "telepresence",
    "autonomy",
    "manipulation",
    "manipulation_pro",
    "custom",
]


def _process_xacro(robot_model, mecanum, configuration):
    share = get_package_share_directory("rosbot_description")
    xacro_path = os.path.join(share, "urdf", f"{robot_model}.urdf.xacro")
    mappings = {"mecanum": mecanum}
    if robot_model == "rosbot_xl":
        mappings["configuration"] = configuration
    doc = xacro.process_file(xacro_path, mappings=mappings)
    return doc.toxml()


def _params(model, mecanum, configurations):
    return [(model, mecanum, cfg) for cfg in (configurations or [None])]


_TEST_MATRIX = (
    _params("rosbot", "True", None)
    + _params("rosbot", "False", None)
    + _params("rosbot_xl", "True", ROSBOT_XL_CONFIGURATIONS)
    + _params("rosbot_xl", "False", ROSBOT_XL_CONFIGURATIONS)
)


@pytest.mark.parametrize("robot_model,mecanum,configuration", _TEST_MATRIX)
def test_xacro_produces_valid_urdf(robot_model, mecanum, configuration):
    urdf = _process_xacro(robot_model, mecanum, configuration)
    # Cheap structural sanity: parsing succeeded and the document contains the
    # bare minimum of a robot description.
    assert (
        "<link" in urdf
    ), f"No <link> in URDF for ({robot_model}, mecanum={mecanum}, {configuration})"
    assert (
        "<joint" in urdf
    ), f"No <joint> in URDF for ({robot_model}, mecanum={mecanum}, {configuration})"
    assert f'<robot name="{robot_model}"' in urdf, "Robot name tag missing or mismatched"


@pytest.mark.parametrize("configuration", ROSBOT_XL_CONFIGURATIONS)
def test_manipulator_presence_matches_configuration(configuration):
    """CLAUDE.md §8: configuration=manipulation* spawns the OpenMANIPULATOR-X."""
    urdf = _process_xacro("rosbot_xl", "False", configuration)
    has_manipulator = '"joint1"' in urdf or 'name="joint1"' in urdf
    if configuration.startswith("manipulation"):
        assert has_manipulator, (
            f"configuration='{configuration}' must include the manipulator macro "
            "(joint1 missing from URDF)"
        )
    else:
        assert not has_manipulator, (
            f"configuration='{configuration}' must NOT include the manipulator "
            "(joint1 present unexpectedly)"
        )


def test_components_config_derived_from_configuration():
    """Soft-compat: when components_config is not given explicitly, it must
    derive from the configuration arg. manipulation.yaml ships an LDR06 lidar
    (rplidar_link in URDF); basic.yaml has no components. If derivation broke
    (always defaulted to basic.yaml), both URDFs would carry the same set."""
    basic_urdf = _process_xacro("rosbot_xl", "False", "basic")
    manipulation_urdf = _process_xacro("rosbot_xl", "False", "manipulation")
    assert "rplidar_link" not in basic_urdf, (
        "basic configuration unexpectedly carries lidar — components_config "
        "derivation is reading the wrong yaml."
    )
    assert "rplidar_link" in manipulation_urdf, (
        "manipulation configuration is missing the lidar component — "
        "components_config did not derive from configuration. "
        "Did rosbot_xl.urdf.xacro lose its soft-compat property?"
    )
