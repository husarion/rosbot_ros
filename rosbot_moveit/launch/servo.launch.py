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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _apply_config_dir_overrides(builder, config_dir):
    """See move_group.launch.py's copy of this helper for the full rationale
    (including why ompl_planning.yaml can't be routed this way) -- this one
    only overrides what servo_node/joy2servo actually take as parameters
    below."""
    if not config_dir:
        return builder
    base = os.path.join(config_dir, "rosbot_moveit", "config")
    return (
        builder.robot_description_semantic(file_path=os.path.join(base, "rosbot_xl.srdf"))
        .robot_description_kinematics(file_path=os.path.join(base, "kinematics.yaml"))
        .joint_limits(file_path=os.path.join(base, "joint_limits.yaml"))
    )


def _launch_setup(context):
    config_dir = LaunchConfiguration("config_dir").perform(context)

    servo_yaml_path = (
        os.path.join(config_dir, "rosbot_moveit", "config", "moveit_servo.yaml")
        if config_dir
        else os.path.join(
            get_package_share_directory("rosbot_moveit"), "config", "moveit_servo.yaml"
        )
    )
    with open(servo_yaml_path) as f:
        servo_params = {"moveit_servo": yaml.safe_load(f)}

    pkg_config_dir = PythonExpression(
        [
            "'",
            config_dir,
            "/rosbot_joy' if '",
            config_dir,
            "' else '",
            FindPackageShare("rosbot_joy"),
            "'",
        ]
    )
    joy_config = PathJoinSubstitution([pkg_config_dir, "config", "config.yaml"])

    # URDF must match move_group; otherwise servo's model diverges from live.
    components_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "config", "rosbot_xl", "manipulation.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "urdf",
                    "rosbot_xl.urdf.xacro",
                ]
            ),
            " components_config:=",
            components_config,
            " configuration:='manipulation'",
        ]
    )

    moveit_config = _apply_config_dir_overrides(
        MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_moveit"), config_dir
    ).to_moveit_configs()
    moveit_config.robot_description = {"robot_description": robot_description_content}

    servo_enabled_condition = IfCondition(LaunchConfiguration("servo_enabled"))

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=servo_enabled_condition,
    )

    joy2servo = Node(
        package="rosbot_moveit",
        executable="joy2servo",
        parameters=[
            joy_config,
            # MGI needs SRDF + kinematics + joint_limits for named targets /
            # gripper move(); otherwise logs "No kinematics plugins defined".
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        # WARN default silences KDL "Using position only ik" spam (printed per
        # searchPositionIK call at 20-50 Hz). Per-logger filter doesn't work in
        # jazzy: MoveIt attaches plugin loggers under auto-named internal nodes.
        ros_arguments=[
            "--log-level",
            "warn",
            "--log-level",
            "joy2servo:=info",
        ],
        # joy2servo only publishes into servo_node's command topics -- with
        # servo_node off it has no effect, so gate it the same way instead of
        # leaving a dead process running.
        condition=servo_enabled_condition,
    )

    return [servo_node, joy2servo]


def generate_launch_description():
    declare_config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description="Path to the common configuration directory. You can create such common configuration directory with `ros2 run rosbot_utils create_config_dir {directory}`.",
    )

    declare_servo_enabled_arg = DeclareLaunchArgument(
        "servo_enabled",
        default_value="true",
        description=(
            "Whether to start servo_node + joy2servo (joystick teleop backend for the "
            "manipulator). servo_node's collision-checking loop runs continuously once "
            "started and measured ~91% of one CPU core on a Jetson Orin Nano even while "
            "idle (2026-08-17 profiling) -- set to false to skip it entirely when the arm "
            "is not in use, e.g. together with arm_activate:=false."
        ),
    )

    return LaunchDescription(
        [declare_config_dir_arg, declare_servo_enabled_arg, OpaqueFunction(function=_launch_setup)]
    )
