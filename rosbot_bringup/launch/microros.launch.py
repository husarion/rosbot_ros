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
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_microros_agent_node(context, *args, **kwargs):
    env_setup_actions = []

    ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
    if ros_domain_id:
        env_setup_actions.append(
            SetEnvironmentVariable(name="XRCE_DOMAIN_ID_OVERRIDE", value=ros_domain_id)
        )

    config_dir = LaunchConfiguration("config_dir").perform(context)
    port = LaunchConfiguration("port").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    serial_baudrate = LaunchConfiguration("serial_baudrate").perform(context)
    serial_port = LaunchConfiguration("serial_port").perform(context)

    config_rosbot_bringup_dir = PythonExpression(
        [
            "'",
            config_dir,
            "/rosbot_bringup' if '",
            config_dir,
            "' else '",
            FindPackageShare("rosbot_bringup"),
            "'",
        ]
    )
    fastrtps_profiles = PathJoinSubstitution(
        [config_rosbot_bringup_dir, "config", "microros_localhost_only.xml"]
    )

    micoros_communication_args = {
        "rosbot": ["serial", "-b", serial_baudrate, "-D", serial_port],
        "rosbot_xl": ["udp4", "--port", port],
    }

    if os.environ.get("ROS_LOCALHOST_ONLY") == "1":
        env_setup_actions.extend(
            [
                LogInfo(
                    msg=[
                        "ROS_LOCALHOST_ONLY set to 1. Using FASTRTPS_DEFAULT_PROFILES_FILE=",
                        fastrtps_profiles,
                    ]
                ),
                SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"),
                SetEnvironmentVariable(
                    name="FASTRTPS_DEFAULT_PROFILES_FILE",
                    value=fastrtps_profiles,
                ),
            ]
        )

    microros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=micoros_communication_args[robot_model],
        output="screen",
    )

    return env_setup_actions + [microros_agent_node]


def generate_launch_description():
    declare_config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="",
        description="Path to the common configuration directory. You can create such common configuration directory with `ros2 run rosbot_utils create_config_dir {directory}`.",
    )

    declare_port_arg = DeclareLaunchArgument(
        "port",
        default_value="8888",
        description="ROSbot XL only. UDP4 port for micro-ROS agent",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value=""),
        description="Specify robot model",
        choices=["rosbot", "rosbot_xl"],
    )

    declare_serial_baudrate_arg = DeclareLaunchArgument(
        "serial_baudrate",
        default_value="576000",
        description="ROSbot only. Baud rate for serial communication",
    )

    declare_serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttySERIAL",
        description="ROSbot only. Serial port for micro-ROS agent",
    )

    return LaunchDescription(
        [
            declare_config_dir_arg,
            declare_port_arg,
            declare_robot_model_arg,
            declare_serial_baudrate_arg,
            declare_serial_port_arg,
            OpaqueFunction(function=generate_microros_agent_node),
        ]
    )
