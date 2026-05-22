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
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config_dir = LaunchConfiguration("config_dir")

    declare_config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description="Path to the common configuration directory. You can create such common configuration directory with `ros2 run rosbot_utils create_config_dir {directory}`.",
    )

    pkg_config_dir = PythonExpression(
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
    joy_config = PathJoinSubstitution([pkg_config_dir, "config", "config.yaml"])

    # URDF must match move_group; otherwise servo's model diverges from live.
    components_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "config", "rosbot_xl", "manipulation.yaml"]
    )
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

    moveit_config = MoveItConfigsBuilder(
        "rosbot_xl", package_name="rosbot_moveit"
    ).to_moveit_configs()
    moveit_config.robot_description = {"robot_description": robot_description_content}

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
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    joy2servo = Node(
        package="rosbot_moveit",
        executable="joy2servo",
        parameters=[
            joy_config,
            # MGI needs SRDF + kinematics + joint_limits for named targets /
            # gripper move(); otherwise logs "No kinematics plugins defined".
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        # WARN default silences KDL "Using position only ik" spam (printed per
        # searchPositionIK call at 20-50 Hz). Per-logger filter doesn't work in
        # jazzy: MoveIt attaches plugin loggers under auto-named internal nodes.
        ros_arguments=[
            "--log-level",
            "warn",
            "--log-level",
            "joy2servo:=info",
        ],
    )

    return LaunchDescription([declare_config_dir_arg, servo_node, joy2servo])
