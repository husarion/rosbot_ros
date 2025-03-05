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
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    mecanum = LaunchConfiguration("mecanum")
    use_sim = LaunchConfiguration("use_sim", default="False")

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Adds a namespace to all running nodes.",
    )

    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    controller_config_name = PythonExpression(
        [
            "'mecanum_drive_controller.yaml' if ",
            mecanum,
            " else 'diff_drive_controller.yaml'",
        ]
    )

    namespace_ext = PythonExpression(
        ["''", " if '", namespace, "' == '' ", "else ", "'", namespace, "/'"]
    )

    controller_manager_name = LaunchConfiguration(
        "controller_manager_name",
        default=[namespace_ext, "controller_manager"],
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "urdf",
                    "rosbot.urdf.xacro",
                ]
            ),
            " mecanum:=",
            mecanum,
            " namespace:=",
            namespace,
            " use_sim:=",
            use_sim,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rosbot_controller"),
            "config",
            controller_config_name,
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            ("imu_sensor_node/imu", "/_imu/data_raw"),
            ("~/motors_cmd", "/_motors_cmd"),
            ("~/motors_response", "/_motors_response"),
            ("rosbot_base_controller/cmd_vel_unstamped", "cmd_vel"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        condition=UnlessCondition(use_sim),
        namespace=namespace,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        namespace=namespace,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "20",
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rosbot_base_controller",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "20",
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "20",
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # spawners expect ros2_control_node to be running
    delayed_spawner_nodes = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            imu_broadcaster_spawner,
        ],
    )

    def check_if_log_is_fatal(event):
        red_color = "\033[91m"
        reset_color = "\033[0m"
        if "fatal" in event.text.decode().lower() or "failed" in event.text.decode().lower():
            print(f"{red_color}Fatal error: {event.text}. Emitting shutdown...{reset_color}")
            return EmitEvent(event=Shutdown(reason="Spawner failed"))

    joint_state_monitor = RegisterEventHandler(
        OnProcessIO(
            target_action=joint_state_broadcaster_spawner,
            on_stderr=lambda event: check_if_log_is_fatal(event),
        )
    )
    robot_controller_monitor = RegisterEventHandler(
        OnProcessIO(
            target_action=robot_controller_spawner,
            on_stderr=lambda event: check_if_log_is_fatal(event),
        )
    )
    imu_broadcaster_monitor = RegisterEventHandler(
        OnProcessIO(
            target_action=imu_broadcaster_spawner,
            on_stderr=lambda event: check_if_log_is_fatal(event),
        )
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mecanum_arg,
            control_node,
            robot_state_pub_node,
            delayed_spawner_nodes,
            joint_state_monitor,
            robot_controller_monitor,
            imu_broadcaster_monitor,
        ]
    )
