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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    joy_config = LaunchConfiguration("joy_config")
    declare_servo_joy_arg = DeclareLaunchArgument(
        "joy_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rosbot_joy"), "config", "config.yaml"]
        ),
        description="ROS2 parameters file to use with joy_servo node",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    moveit_config = (
        MoveItConfigsBuilder("robot_xl", package_name="rosbot_moveit").joint_limits(
            file_path="config/joint_limits.yaml"
        )
    ).to_moveit_configs()

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("rosbot_moveit")
        .yaml("config/moveit_servo.yaml")
        .to_dict()
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            # acceleration_filter_update_period,
            # planning_group_name,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    joy2servo = Node(
        package="rosbot_joy",
        executable="joy2servo",
        parameters=[joy_config],
    )

    actions = [
        declare_servo_joy_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        servo_node,
        joy2servo,
    ]

    return LaunchDescription(actions)
