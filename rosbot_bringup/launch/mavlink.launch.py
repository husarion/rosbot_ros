# Copyright 2026 Husarion sp. z o.o.
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

# MAVLink sibling of microros.launch.py. Requires the rosbot_mavlink_bridge
# package on the overlay (shipped with the MAVLink firmware release).

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_bridge_launch(context, *args, **kwargs):

    namespace = LaunchConfiguration("namespace").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    serial_port = LaunchConfiguration("serial_port").perform(context)
    serial_baudrate = LaunchConfiguration("serial_baudrate").perform(context)

    pre_communication_cmd = [
        "ros2",
        "run",
        "rosbot_utils",
        "configure_robot",
        "--robot-model",
        robot_model,
        "--backend",
        "mavlink",
    ]
    if namespace:
        pre_communication_cmd.extend(["--namespace", namespace])
    if robot_model != "rosbot":
        pre_communication_cmd.extend(["--usb"])

    pre_communication = ExecuteProcess(
        cmd=pre_communication_cmd,
        output="screen",
        name="pre_communication",
    )

    bridge_launch_args = [("namespace", namespace)]
    if robot_model == "rosbot":
        bridge_launch_args.extend(
            [
                ("serial_port", serial_port),
                ("serial_baudrate", serial_baudrate),
            ]
        )

    bridge_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("rosbot_mavlink_bridge"), "launch", f"{robot_model}.launch.py"]
        ),
        launch_arguments=bridge_launch_args,
    )

    def on_pre_comm_exit(event, context):
        if event.returncode == 0:
            return [bridge_launch]
        return [EmitEvent(event=Shutdown(reason="Pre-communication failed"))]

    handle_exit = RegisterEventHandler(
        OnProcessExit(target_action=pre_communication, on_exit=on_pre_comm_exit)
    )

    return [pre_communication, handle_exit]


def generate_launch_description():

    declare_microros_mode_arg = DeclareLaunchArgument(
        "microros_mode",
        default_value="default",
        description="Compatibility-only placeholder. Ignored by the MAVLink launch — the bridge uses a fixed transport per robot model.",
        choices=["default", "udp", "serial"],
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    declare_port_arg = DeclareLaunchArgument(
        "port",
        default_value="8888",
        description="Compatibility-only placeholder. The MAVLink bridge uses mavros default ports (14550/14555).",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL", default_value=""),
        description="Specify robot model",
        choices=["rosbot", "rosbot_xl"],
    )

    declare_serial_baudrate_arg = DeclareLaunchArgument(
        "serial_baudrate",
        default_value="921600",
        description="ROSbot only. Baud rate for the SBC<->MCU serial line.",
    )

    declare_serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttySERIAL",
        description="ROSbot only. Serial port the bridge opens to talk MAVLink to the MCU.",
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_port_arg,
            declare_robot_model_arg,
            declare_serial_baudrate_arg,
            declare_serial_port_arg,
            declare_microros_mode_arg,
            OpaqueFunction(function=generate_bridge_launch),
        ]
    )
