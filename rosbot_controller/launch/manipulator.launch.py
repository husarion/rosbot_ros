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
from launch.actions import (
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    manipulator_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "manipulator_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "15",
        ],
        output="screen",
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "15",
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

    controllers = [manipulator_controller, gripper_controller]

    def check_if_log_is_fatal(event):
        red_color = "\033[91m"
        reset_color = "\033[0m"
        msg = event.text.decode().lower()
        if ("fatal" in msg or "failed" in msg) and "attempt" not in msg:
            print(f"{red_color}Fatal error: {event.text}. Emitting shutdown...{reset_color}")
            return EmitEvent(event=Shutdown(reason="Spawner failed"))

    controllers_monitor = [
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawner,
                on_stderr=check_if_log_is_fatal,
            )
        )
        for spawner in controllers
    ]

    controllers_monitor = GroupAction(controllers_monitor)

    return LaunchDescription(
        [
            manipulator_controller,
            gripper_controller,
            move_group_launch,
            servo_launch,
            move_to_home_pose,
        ]
    )
