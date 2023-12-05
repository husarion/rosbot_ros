# Copyright 2023 Husarion
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
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_microros_agent_node(context, *args, **kwargs):
    # Additional environment variable setup actions
    env_setup_actions = []

    # Check if ROS_DOMAIN_ID is set and not empty
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
    if ros_domain_id:
        env_setup_actions.append(
            SetEnvironmentVariable(name="XRCE_DOMAIN_ID_OVERRIDE", value=ros_domain_id)
        )

    serial_port = LaunchConfiguration("serial_port").perform(context)
    serial_baudrate = LaunchConfiguration("serial_baudrate").perform(context)
    localhost_only_fastrtps_profiles_file = LaunchConfiguration(
        "localhost_only_fastrtps_profiles_file"
    ).perform(context)

    if os.environ.get("ROS_LOCALHOST_ONLY") == "1":
        # with localhost only setup fastdds is required with a custom config
        rmw_implementation = "rmw_fastrtps_cpp"

        env_setup_actions.extend(
            [
                SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value=rmw_implementation),
                SetEnvironmentVariable(
                    name="FASTRTPS_DEFAULT_PROFILES_FILE",
                    value=localhost_only_fastrtps_profiles_file,
                ),
            ]
        )

        microros_agent_node = Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            arguments=["serial", "-D", serial_port, "-b", serial_baudrate],
            output="screen",
        )

        return env_setup_actions + [microros_agent_node]
    else:
        microros_agent_node = Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            arguments=["serial", "-D", serial_port, "-b", serial_baudrate],
            output="screen",
        )

        return env_setup_actions + [microros_agent_node]


def generate_launch_description():
    declare_serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttySERIAL",
        description="Serial port for micro-ROS agent",
    )

    declare_serial_baudrate_arg = DeclareLaunchArgument(
        "serial_baudrate", default_value="576000", description="Baud rate for serial communication"
    )

    # Locate the rosbot_bringup package
    package_dir = FindPackageShare("rosbot_bringup").find("rosbot_bringup")

    # Construct the path to the XML file within the package
    fastrtps_profiles_file = os.path.join(package_dir, "config", "microros_localhost_only.xml")

    declare_localhost_only_fastrtps_profiles_file_arg = DeclareLaunchArgument(
        "localhost_only_fastrtps_profiles_file",
        default_value=fastrtps_profiles_file,
        description=(
            "Path to the Fast RTPS default profiles file for Micro-ROS agent for localhost only"
            " setup"
        ),
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/bringup.launch.py"])
    )

    return LaunchDescription(
        [
            declare_serial_port_arg,
            declare_serial_baudrate_arg,
            declare_localhost_only_fastrtps_profiles_file_arg,
            OpaqueFunction(function=generate_microros_agent_node),
            bringup_launch,
        ]
    )
