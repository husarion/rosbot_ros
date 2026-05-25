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

"""xacro -> URDF for every (robot_model, mecanum, configuration) combo."""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

# rosbot_xl picks components yaml in rosbot_description/config/rosbot_xl/ by name.
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


def test_gazebo_urdf_namespace_remappings():
    """Pins the gz_ros2_control <remapping> set — push_ros_namespace cannot
    reach this plugin (gz_sim hosts it outside the LaunchContext)."""
    share = get_package_share_directory("rosbot_description")
    xacro_path = os.path.join(share, "urdf", "rosbot_xl.urdf.xacro")
    doc = xacro.process_file(
        xacro_path,
        mappings={"mecanum": "False", "configuration": "basic", "use_sim": "True"},
    )
    urdf = doc.toxml()
    root = ET.fromstring(urdf)

    plugin = None
    for elem in root.iter("plugin"):
        if "gz_ros2_control" in (elem.get("filename") or ""):
            plugin = elem
            break
    assert plugin is not None, "gz_ros2_control-system plugin missing from URDF"

    actual_remaps = {r.text for r in plugin.iter("remapping") if r.text}

    expected_remaps = {
        "/diagnostics:=diagnostics",
        "/tf:=tf",
        "/tf_static:=tf_static",
        "/controller_manager/cleanup_controller:=controller_manager/cleanup_controller",
        "/controller_manager/configure_controller:=controller_manager/configure_controller",
        "/controller_manager/list_controllers:=controller_manager/list_controllers",
        "/controller_manager/list_controller_types:=controller_manager/list_controller_types",
        "/controller_manager/list_hardware_components:=controller_manager/list_hardware_components",
        "/controller_manager/list_hardware_interfaces:=controller_manager/list_hardware_interfaces",
        "/controller_manager/load_controller:=controller_manager/load_controller",
        "/controller_manager/reload_controller_libraries:=controller_manager/reload_controller_libraries",
        "/controller_manager/set_hardware_component_state:=controller_manager/set_hardware_component_state",
        "/controller_manager/switch_controller:=controller_manager/switch_controller",
        "/controller_manager/unload_controller:=controller_manager/unload_controller",
        "/controller_manager/activity:=controller_manager/activity",
        "/controller_manager/statistics:=controller_manager/statistics",
        "/controller_manager/introspection_data:=controller_manager/introspection_data",
    }
    missing = expected_remaps - actual_remaps
    assert not missing, (
        "gazebo.urdf.xacro <remapping> block is missing entries required for "
        "multi-robot namespace isolation: " + ", ".join(sorted(missing))
    )


def test_components_config_derived_from_configuration():
    """Soft-compat: components_config must derive from configuration arg
    (basic has no lidar; manipulation has rplidar_link)."""
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
