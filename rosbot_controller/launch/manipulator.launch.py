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
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    manipulator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "manipulator_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        output="screen",
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("open_manipulator_x_moveit"), "launch", "move_group.launch.py"]
            )
        ),
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("open_manipulator_x_moveit"), "launch", "servo.launch.py"]
            )
        )
    )

    home_node = Node(package="open_manipulator_x_moveit", executable="home")
    move_to_home_pose = TimerAction(period=10.0, actions=[home_node])

    return LaunchDescription(
        [
            manipulator_controller_spawner,
            gripper_controller_spawner,
            move_group_launch,
            servo_launch,
            move_to_home_pose,
        ]
    )
