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
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = LaunchConfiguration("config_dir")
    joy_vel = LaunchConfiguration("joy_vel")

    declare_config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description="Path to the common configuration directory. You can create such common configuration directory with `ros2 run rosbot_utils create_config_dir {directory}`.",
    )

    config_rosbot_joy_dir = PythonExpression(
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

    joy_config = PathJoinSubstitution([config_rosbot_joy_dir, "config", "joy.yaml"])

    declare_joy_vel_arg = DeclareLaunchArgument(
        "joy_vel",
        default_value="cmd_vel",
        description="The topic name to which velocity commands will be published.",
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
            declare_config_dir_arg,
            declare_joy_vel_arg,
            joy_node,
            joy2twist_node,
        ]
    )
