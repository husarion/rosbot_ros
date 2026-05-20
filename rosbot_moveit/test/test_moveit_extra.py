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

"""Extra invariants on top of test_moveit_config.py.

Smoke-tests MoveItConfigsBuilder("rosbot_xl") and pins the servo<->SRDF
group-name link and the initial_positions / SRDF joint coverage.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def _share(rel_path: str) -> str:
    return os.path.join(get_package_share_directory("rosbot_moveit"), rel_path)


def test_moveit_configs_builder_loads():
    """Guards the 2026-05-15 typo regression ('robot_xl' vs 'rosbot_xl').

    MoveItConfigsBuilder reads .setup_assistant + config/*.yaml and silently
    falls back to defaults if the robot name does not match a registered
    package — the only signal that you have it wrong shows up at runtime as
    'No kinematics plugins defined'. This test fails the build instead.
    """
    from moveit_configs_utils import MoveItConfigsBuilder

    configs = MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_moveit").to_moveit_configs()
    # robot_description_semantic is loaded from rosbot_xl.srdf; the dict body
    # carries the FQN root tag.
    srdf = configs.robot_description_semantic["robot_description_semantic"]
    assert '<robot name="rosbot_xl">' in srdf, (
        "MoveItConfigsBuilder failed to load rosbot_xl.srdf — likely the "
        "robot_name argument no longer matches the .srdf file name."
    )
    # kinematics + joint_limits must be present for joy2servo (see servo.launch.py).
    assert configs.robot_description_kinematics
    assert configs.joint_limits


def test_servo_move_group_matches_srdf():
    """moveit_servo's move_group_name must reference a real SRDF group."""
    import xml.etree.ElementTree as ET

    with open(_share("config/moveit_servo.yaml")) as f:
        servo = yaml.safe_load(f)
    srdf_root = ET.parse(_share("config/rosbot_xl.srdf")).getroot()
    srdf_groups = {g.attrib["name"] for g in srdf_root.findall("group")}
    assert servo["move_group_name"] in srdf_groups, (
        f"move_group_name={servo['move_group_name']!r} is not defined in the SRDF "
        f"(groups: {sorted(srdf_groups)})"
    )


def test_servo_command_in_type_is_speed_units():
    """joy2servo publishes JointJog in rad/s (speed_units). Flipping this to
    'unitless' silently rescales joy2servo's scale_joint output by joy stick
    range and the arm crawls."""
    with open(_share("config/moveit_servo.yaml")) as f:
        servo = yaml.safe_load(f)
    assert servo["command_in_type"] == "speed_units"


def test_initial_positions_covers_actuated_srdf_joints():
    """initial_positions seeds the ros2_control fake/mock system. Every joint
    in a SRDF group must be present unless it is marked <passive_joint>."""
    import xml.etree.ElementTree as ET

    with open(_share("config/initial_positions.yaml")) as f:
        initial = yaml.safe_load(f)["initial_positions"]
    srdf_root = ET.parse(_share("config/rosbot_xl.srdf")).getroot()

    group_joints = {
        j.attrib["name"] for g in srdf_root.findall("group") for j in g.findall("joint")
    }
    passive = {pj.attrib["name"] for pj in srdf_root.findall("passive_joint")}
    # The URDF carries fixed joints; we cannot resolve those from the SRDF
    # alone, so allow that the initial_positions list is a *subset* match for
    # the actuated set. Explicit allow-list keeps the test debuggable.
    expected_actuated = {"joint1", "joint2", "joint3", "joint4", "gripper_left_joint"}
    assert expected_actuated <= group_joints - passive, (
        "Expected actuated joints diverged from SRDF — update the test, then verify "
        "initial_positions.yaml against the new SRDF."
    )
    missing = expected_actuated - set(initial)
    assert not missing, f"Missing initial positions for actuated joints: {missing}"
    assert (
        "gripper_right_joint" not in initial
    ), "gripper_right_joint is <passive_joint> in the SRDF; ros2_control should not seed it."
