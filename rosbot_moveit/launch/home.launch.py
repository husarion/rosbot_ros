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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _apply_config_dir_overrides(builder, config_dir):
    """See move_group.launch.py's copy of this helper for the full rationale
    (including why ompl_planning.yaml can't be routed this way) -- this one
    only overrides what home_node actually takes as parameters below."""
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

    moveit_config = _apply_config_dir_overrides(
        MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_moveit"), config_dir
    ).to_moveit_configs()

    # No `namespace` arg: included under parent's push_ros_namespace; declaring
    # one would double-stack. For standalone use see dock.launch.py.
    home_node = Node(
        package="rosbot_moveit",
        executable="home",
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    return [home_node]


def generate_launch_description():
    declare_config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description="Path to the common configuration directory. You can create such common configuration directory with `ros2 run rosbot_utils create_config_dir {directory}`.",
    )

    return LaunchDescription([declare_config_dir_arg, OpaqueFunction(function=_launch_setup)])
