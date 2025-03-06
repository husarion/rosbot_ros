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
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
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

    ready_node = Node(package="open_manipulator_x_moveit", executable="ready")
    move_to_ready_pose = TimerAction(period=5.0, actions=[ready_node])

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

    delayed_servo_launch = RegisterEventHandler(
        OnProcessExit(target_action=ready_node, on_exit=[servo_launch])
    )

    return LaunchDescription(
        [
            manipulator_controller_spawner,
            gripper_controller_spawner,
            move_group_launch,
            move_to_ready_pose,
            delayed_servo_launch,
        ]
    )
