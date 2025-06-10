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
    joy_config = LaunchConfiguration("joy_config")
    joy_vel = LaunchConfiguration("joy_vel")
    namespace = LaunchConfiguration("namespace")  # Capture the namespace

    rosbot_joy = FindPackageShare("rosbot_joy")

    declare_joy_config_arg = DeclareLaunchArgument(
        "joy_config",
        default_value=PathJoinSubstitution([rosbot_joy, "config", "joy.yaml"]),
        description="The file path to the configuration YAML file for the teleop_twist_joy node.",
    )

    declare_joy_vel_arg = DeclareLaunchArgument(
        "joy_vel",
        default_value="cmd_vel",
        description="The topic name to which velocity commands will be published.",
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[joy_config],
    )

    joy2twist_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="joy2twist",
        parameters=[joy_config],
        remappings={("/cmd_vel", joy_vel)},
    )

    return LaunchDescription(
        [
            declare_joy_config_arg,
            declare_joy_vel_arg,
            declare_namespace_arg,
            joy_node,
            joy2twist_node,
        ]
    )
