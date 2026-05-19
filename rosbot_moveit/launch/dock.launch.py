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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    moveit_config = (
        MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_moveit").joint_limits(
            file_path="config/joint_limits.yaml"
        )
    ).to_moveit_configs()

    # `dock` uses MoveGroupInterface, which loads a RobotModelLoader that needs
    # `robot_description_kinematics` to avoid the "No kinematics plugins defined" warning.
    # Standalone-only wrapper (the auto-home path runs `home.launch.py` from
    # manipulator.yaml; there is no auto-dock equivalent). Uses `Node(namespace=...)`
    # rather than `PushRosNamespace` so that the empty default does not double-stack
    # an existing parent namespace when this file is hypothetically included from
    # somewhere already inside a `push_ros_namespace` group.
    dock_node = Node(
        package="rosbot_moveit",
        executable="dock",
        namespace=namespace,
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Robot namespace; leave empty when the robot was launched without one.",
            ),
            dock_node,
        ]
    )
