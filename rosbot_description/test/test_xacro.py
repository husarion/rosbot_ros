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
import tempfile
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml
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

# rosbot (2/3) has no internal "configuration" arg (unlike rosbot_xl) — the
# launch layer resolves configuration -> components_config path, so tests
# must pass components_config directly.
ROSBOT_CONFIGURATIONS = ["basic", "pro", "custom"]


def _rosbot_components_config_path(configuration):
    share = get_package_share_directory("rosbot_description")
    return os.path.join(share, "config", "rosbot", f"{configuration}.yaml")


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


def test_rosbot_basic_vs_pro_sensors():
    """ROSbot 3 basic = OAK-D Lite + RPLIDAR C1; ROSbot 3 PRO = OAK-D Pro + RPLIDAR S2."""
    share = get_package_share_directory("rosbot_description")
    xacro_path = os.path.join(share, "urdf", "rosbot.urdf.xacro")

    basic_urdf = xacro.process_file(
        xacro_path,
        mappings={
            "mecanum": "False",
            "components_config": _rosbot_components_config_path("basic"),
        },
    ).toxml()
    pro_urdf = xacro.process_file(
        xacro_path,
        mappings={
            "mecanum": "False",
            "components_config": _rosbot_components_config_path("pro"),
        },
    ).toxml()

    assert "rplidar/c1.glb" in basic_urdf, "basic configuration must use RPLIDAR C1"
    assert "rplidar/s2.glb" not in basic_urdf, "basic configuration must not use RPLIDAR S2"
    assert "OAK-D-LITE" in basic_urdf, "basic configuration must use OAK-D Lite"

    assert "rplidar/s2.glb" in pro_urdf, "pro configuration must use RPLIDAR S2"
    assert "rplidar/c1.glb" not in pro_urdf, "pro configuration must not use RPLIDAR C1"
    assert "OAK-D-PRO" in pro_urdf, "pro configuration must use OAK-D Pro"


def test_camera_mount_is_custom_component_with_angle_override():
    """camera_mount is a `type: custom` component (like man01_bracket) - present
    only where a config's components: list adds it, with camera_mount_angle_{1,2}
    read from that same yaml entry and defaulting to camera_mount.urdf.xacro's
    own values when absent."""
    share = get_package_share_directory("rosbot_description")
    xacro_path = os.path.join(share, "urdf", "rosbot_xl.urdf.xacro")

    # basic.yaml has no camera_mount entry -> no camera_mount_link at all.
    basic_urdf = xacro.process_file(
        xacro_path, mappings={"mecanum": "True", "configuration": "basic"}
    ).toxml()
    assert "camera_mount_bot_link" not in basic_urdf, "basic must not include camera_mount"

    # telepresence.yaml/autonomy.yaml both add camera_mount as a `type: custom`
    # component - real tuned angle values live in those files and change over
    # time, so just check the structure is there (not specific angle numbers).
    for configuration in ("telepresence", "autonomy"):
        urdf = xacro.process_file(
            xacro_path, mappings={"mecanum": "True", "configuration": configuration}
        ).toxml()
        assert "camera_mount_bot_link" in urdf, f"{configuration} must include camera_mount"

    # A from-scratch custom entry with no angle keys -> macro defaults apply.
    default_cfg = {
        "components": [
            {
                "type": "custom",
                "package": "rosbot_description",
                "file": "urdf/rosbot_xl/components/camera_mount.urdf.xacro",
                "macro_name": "camera_mount",
                "parent_link": "cover_link",
                "xyz": "-0.1 0.0 0.0",
                "rpy": "0.0 0.0 0.0",
            }
        ]
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        yaml.dump(default_cfg, f)
        default_cfg_path = f.name

    default_urdf = xacro.process_file(
        xacro_path, mappings={"mecanum": "True", "components_config": default_cfg_path}
    ).toxml()
    default_root = ET.fromstring(default_urdf)
    default_angle_1 = default_root.find(
        ".//joint[@name='camera_mount_bot_to_camera_mount_mid_joint']/origin"
    ).get("rpy")
    default_angle_2 = default_root.find(
        ".//joint[@name='camera_mount_mid_to_camera_mount_top_joint']/origin"
    ).get("rpy")
    assert (
        default_angle_1 == "0.0 0.0 0.0"
    ), f"unexpected default camera_mount_angle_1: {default_angle_1}"
    assert (
        default_angle_2 == "0.0 0.0 0.0"
    ), f"unexpected default camera_mount_angle_2: {default_angle_2}"

    # A from-scratch custom entry can override both angles at once.
    cfg = {
        "components": [
            {
                "type": "custom",
                "package": "rosbot_description",
                "file": "urdf/rosbot_xl/components/camera_mount.urdf.xacro",
                "macro_name": "camera_mount",
                "parent_link": "cover_link",
                "xyz": "-0.1 0.0 0.0",
                "rpy": "0.0 0.0 0.0",
                "camera_mount_angle_1": 1.0,
                "camera_mount_angle_2": -1.2,
            }
        ]
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        yaml.dump(cfg, f)
        cfg_path = f.name

    override_urdf = xacro.process_file(
        xacro_path, mappings={"mecanum": "True", "components_config": cfg_path}
    ).toxml()
    override_root = ET.fromstring(override_urdf)
    angle_1_override = override_root.find(
        ".//joint[@name='camera_mount_bot_to_camera_mount_mid_joint']/origin"
    ).get("rpy")
    angle_2_override = override_root.find(
        ".//joint[@name='camera_mount_mid_to_camera_mount_top_joint']/origin"
    ).get("rpy")
    assert (
        angle_1_override == "0.0 1.0 0.0"
    ), f"override for camera_mount_angle_1 not applied: {angle_1_override}"
    assert (
        angle_2_override == "0.0 -1.2 0.0"
    ), f"override for camera_mount_angle_2 not applied: {angle_2_override}"


def test_pro_configuration_rejected_for_rosbot_xl():
    """`pro` only exists under config/rosbot/ (ROSbot 3 PRO) — rosbot_xl has no
    config/rosbot_xl/pro.yaml, so picking it for rosbot_xl must fail loudly
    instead of silently falling back to some other components set."""
    with pytest.raises(Exception):
        _process_xacro("rosbot_xl", "True", "pro")


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
