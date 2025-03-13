# Copyright 2020 ros2_control Development Team
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
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString

from rosbot_utils.utils import find_device_port


def generate_launch_description():
    configuration = LaunchConfiguration("configuration")
    controller_config = LaunchConfiguration("controller_config")
    manipulator_port = LaunchConfiguration("manipulator_port")
    mecanum = LaunchConfiguration("mecanum")
    namespace = LaunchConfiguration("namespace")
    robot_model = LaunchConfiguration("robot_model")
    use_sim = LaunchConfiguration("use_sim", default="False")

    base_controller_prefix = PythonExpression(
        ["'mecanum_drive' if ", mecanum, " else 'diff_drive'"]
    )
    manipulator = PythonExpression(["'", configuration, "'.startswith('manipulation')"])
    manipulator_prefix = PythonExpression(["'manipulator_' if ", manipulator, " else ''"])
    controller_config_file = PythonExpression(
        ["'", base_controller_prefix, "' + '_' + '", manipulator_prefix, "' + 'controller.yaml'"]
    )
    default_controller_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_controller"), "config", robot_model, controller_config_file]
    )

    declare_controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=default_controller_config,
        description="Path to controller configuration file.",
    )

    declare_configuration_arg = DeclareLaunchArgument(
        "configuration",
        default_value="basic",
        description=(
            "Specify configuration packages. Currently only ROSbot XL has available packages."
        ),
        choices=["basic", "telepresence", "autonomy", "manipulation", "manipulation_pro"],
    )

    default_manipulator_port = find_device_port("0403", "6014", "/dev/ttyUSB0")
    declare_manipulator_port_arg = DeclareLaunchArgument(
        "manipulator_port",
        default_value=default_manipulator_port,
        description="Port to connect to the manipulator.",
    )

    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller, otherwise use diff drive",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value=""),
        description="Specify robot model",
        choices=["rosbot", "rosbot_xl"],
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    ns_controller_config = ReplaceString(controller_config, {"<namespace>/": ns})

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbot_description"), "launch", "load_urdf.launch.py"]
            )
        ),
        launch_arguments={
            "configuration": configuration,
            "controller_config": ns_controller_config,
            "manipulator_port": manipulator_port,
            "mock_joints": "False",
            "robot_model": robot_model,
            "use_sim": use_sim,
        }.items(),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ns_controller_config],
        remappings=[
            ("imu_sensor_node/imu", "/_imu/data_raw"),
            ("~/motors_cmd", "/_motors_cmd"),
            ("~/motors_response", "/_motors_response"),
            ("rosbot_base_controller/cmd_vel", "cmd_vel"),
        ],
        condition=UnlessCondition(use_sim),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    robot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rosbot_base_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbot_controller"), "launch", "manipulator.launch.py"]
            )
        ),
        condition=IfCondition(manipulator),
    )

    controllers = [joint_state_broadcaster, imu_broadcaster, robot_controller]

    # spawners expect ros2_control_node to be running
    delayed_spawner_nodes = TimerAction(
        period=3.0,
        actions=controllers + [manipulator_launch],
    )

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
            declare_configuration_arg,
            declare_manipulator_port_arg,
            declare_mecanum_arg,
            declare_robot_model_arg,
            declare_controller_config_arg,  # controler_config base on mecanum and robot_model arg
            load_urdf,
            control_node,
            delayed_spawner_nodes,
            controllers_monitor,
        ]
    )
