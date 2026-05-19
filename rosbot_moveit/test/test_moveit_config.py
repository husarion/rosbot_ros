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

import os
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory


def _share(rel_path: str) -> str:
    return os.path.join(get_package_share_directory("rosbot_moveit"), rel_path)


def test_srdf_groups_and_named_states():
    """SRDF must declare the manipulator/gripper groups and the named states the
    arm_pose_mover / joy2servo helpers refer to."""
    tree = ET.parse(_share("config/rosbot_xl.srdf"))
    root = tree.getroot()

    group_names = {g.attrib["name"] for g in root.findall("group")}
    assert group_names == {"manipulator", "gripper"}, group_names

    states = {(s.attrib["name"], s.attrib["group"]) for s in root.findall("group_state")}
    required = {
        ("Home", "manipulator"),
        ("Dock", "manipulator"),
        ("Zero", "manipulator"),
        ("Open", "gripper"),
        ("Close", "gripper"),
    }
    missing = required - states
    assert not missing, f"Missing SRDF named states: {missing}"


def test_kinematics_keeps_position_only_ik():
    """OpenMANIPULATOR-X is 4 DoF; without position_only_ik the KDL solver
    cannot satisfy orientation goals and IK fails. This is also what makes
    joy2servo's POSE mode work on a sub-6-DoF arm - KDL with
    position_only_ik zero-weights the orientation rows of the IK task."""
    with open(_share("config/kinematics.yaml")) as f:
        cfg = yaml.safe_load(f)
    assert cfg["manipulator"]["position_only_ik"] is True


def test_moveit_servo_topics_relative():
    """Topic params must stay relative so they live inside the robot namespace
    (regression guard for the /planning_scene namespace leak)."""
    with open(_share("config/moveit_servo.yaml")) as f:
        cfg = yaml.safe_load(f)
    assert cfg["monitored_planning_scene_topic"] == "planning_scene"
    assert cfg["joint_topic"] == "joint_states"


def test_singularity_thresholds_effectively_disabled():
    """4-DoF arm: the full 6xN Jacobian used by velocityScalingFactorForSingularity
    is rank-deficient by construction, so the condition number is effectively
    infinite. Both thresholds must be high enough that POSE mode does not get
    permanently halted with HALT_FOR_SINGULARITY."""
    with open(_share("config/moveit_servo.yaml")) as f:
        cfg = yaml.safe_load(f)
    assert cfg["lower_singularity_threshold"] >= 1.0e9
    assert cfg["hard_stop_singularity_threshold"] >= 1.0e9
    # Servo internally validates `hard > lower`.
    assert cfg["hard_stop_singularity_threshold"] > cfg["lower_singularity_threshold"]


def test_moveit_controllers_have_required_actions():
    """The arm controllers exposed to MoveIt must match the ros2_control side
    (manipulator_controller / gripper_controller)."""
    with open(_share("config/moveit_controllers.yaml")) as f:
        cfg = yaml.safe_load(f)
    controllers = cfg["moveit_simple_controller_manager"]["controller_names"]
    assert set(controllers) == {"manipulator_controller", "gripper_controller"}


def test_ompl_manipulator_planner_configured():
    """OMPL must declare at least one planner for the manipulator group."""
    with open(_share("config/ompl_planning.yaml")) as f:
        cfg = yaml.safe_load(f)
    planners = cfg["ompl"]["manipulator"]["planner_configs"]
    assert len(planners) >= 1
    assert "RRTConnectkConfigDefault" in planners


def test_joint_limits_drops_passive_joint():
    """gripper_right_joint is <passive_joint> in the SRDF; MoveIt would not
    plan it, and stale limits here have already caused confusion before."""
    with open(_share("config/joint_limits.yaml")) as f:
        cfg = yaml.safe_load(f)
    assert "gripper_right_joint" not in cfg["joint_limits"]
    for joint in ["gripper_left_joint", "joint1", "joint2", "joint3", "joint4"]:
        assert joint in cfg["joint_limits"], f"Missing joint limits for {joint}"
