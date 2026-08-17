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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _apply_config_dir_overrides(builder, config_dir):
    """Route the SRDF/kinematics/joint_limits/moveit_controllers configs
    through config_dir when set, mirroring moveit_servo.yaml's handling in
    servo.launch.py -- otherwise MoveItConfigsBuilder always reads them from
    rosbot_moveit's installed package share, config_dir or not.

    ompl_planning.yaml can't be included here: MoveItConfigsBuilder's
    planning_pipelines() has no file_path override, it always scans
    <package_share>/config/*_planning.yaml with no way to redirect that scan
    directory -- a moveit_configs_utils limitation, not something to route
    around with a manual post-hoc dict merge.
    """
    if not config_dir:
        return builder
    base = os.path.join(config_dir, "rosbot_moveit", "config")
    return (
        builder.robot_description_semantic(file_path=os.path.join(base, "rosbot_xl.srdf"))
        .robot_description_kinematics(file_path=os.path.join(base, "kinematics.yaml"))
        .joint_limits(file_path=os.path.join(base, "joint_limits.yaml"))
        .trajectory_execution(file_path=os.path.join(base, "moveit_controllers.yaml"))
    )


def _launch_setup(context):
    config_dir = LaunchConfiguration("config_dir").perform(context)

    moveit_config = _apply_config_dir_overrides(
        MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_moveit"), config_dir
    ).to_moveit_configs()

    components_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "config", "rosbot_xl", "manipulation.yaml"]
    )

    # Override builder's vanilla xacro so URDF matches bringup
    # (components_config + configuration:='manipulation').
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
    moveit_config.robot_description = {"robot_description": robot_description_content}

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        # OccupancyMapMonitor runs unconditionally inside PlanningSceneMonitor;
        # without sensors_3d.yaml it logs one ERROR + WARN at startup. Accepted.
        "disable_capabilities": "",
        "monitor_dynamics": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.0,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    return [move_group_node]


def generate_launch_description():
    declare_config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description="Path to the common configuration directory. You can create such common configuration directory with `ros2 run rosbot_utils create_config_dir {directory}`.",
    )

    return LaunchDescription([declare_config_dir_arg, OpaqueFunction(function=_launch_setup)])
