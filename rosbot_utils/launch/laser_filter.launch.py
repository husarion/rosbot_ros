#!/usr/bin/env python3

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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value=""),
        description="Specify robot model",
        choices=["rosbot", "rosbot_xl"],
    )

    rosbot_utils = FindPackageShare("rosbot_utils")

    laser_filter_config = PathJoinSubstitution(
        [rosbot_utils, "config", robot_model, "laser_filter.yaml"]
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_filter",
        parameters=[laser_filter_config],
    )

    return LaunchDescription([declare_robot_model_arg, laser_filter_node])
