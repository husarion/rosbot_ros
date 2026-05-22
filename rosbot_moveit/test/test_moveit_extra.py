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

"""Extra invariants: MoveItConfigsBuilder smoke + servo<->SRDF group link + joint coverage."""

from conftest import load_yaml, srdf_root


def test_moveit_configs_builder_loads():
    """Guards 2026-05-15 typo regression ('robot_xl' vs 'rosbot_xl').
    MoveItConfigsBuilder silently falls back to defaults on bad name -
    only runtime signal is 'No kinematics plugins defined'."""
    from moveit_configs_utils import MoveItConfigsBuilder

    configs = MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_moveit").to_moveit_configs()
    srdf = configs.robot_description_semantic["robot_description_semantic"]
    assert '<robot name="rosbot_xl">' in srdf, (
        "MoveItConfigsBuilder failed to load rosbot_xl.srdf — likely the "
        "robot_name argument no longer matches the .srdf file name."
    )
    assert configs.robot_description_kinematics
    assert configs.joint_limits


def test_servo_move_group_matches_srdf():
    """moveit_servo's move_group_name must reference a real SRDF group."""
    servo = load_yaml("config/moveit_servo.yaml")
    srdf_groups = {g.attrib["name"] for g in srdf_root().findall("group")}
    assert servo["move_group_name"] in srdf_groups, (
        f"move_group_name={servo['move_group_name']!r} is not defined in the SRDF "
        f"(groups: {sorted(srdf_groups)})"
    )


def test_servo_command_in_type_is_speed_units():
    """joy2servo publishes JointJog in rad/s; 'unitless' silently rescales and arm crawls."""
    servo = load_yaml("config/moveit_servo.yaml")
    assert servo["command_in_type"] == "speed_units"


def test_initial_positions_covers_actuated_srdf_joints():
    """Every non-passive SRDF group joint must appear in initial_positions."""
    initial = load_yaml("config/initial_positions.yaml")["initial_positions"]
    root = srdf_root()

    group_joints = {j.attrib["name"] for g in root.findall("group") for j in g.findall("joint")}
    passive = {pj.attrib["name"] for pj in root.findall("passive_joint")}
    # Explicit allow-list - fixed URDF joints aren't visible from SRDF.
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
