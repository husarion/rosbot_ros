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
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    microros = LaunchConfiguration("microros")
    namespace = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim", default="False")

    declare_microros_arg = DeclareLaunchArgument(
        "microros",
        default_value="True",
        description="Automatically connect with hardware using microros.",
        choices=["True", "true", "False", "false"],
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace for all topics and tfs",
    )

    rosbot_bringup = FindPackageShare("rosbot_bringup")
    rosbot_controller = FindPackageShare("rosbot_controller")

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rosbot_controller, "launch", "controller.launch.py"])
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    microros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rosbot_bringup, "launch", "microros.launch.py"])
        ),
        condition=IfCondition(PythonExpression([microros, " and not ", use_sim])),
    )

    ekf_config = PathJoinSubstitution([rosbot_bringup, "config", "ekf.yaml"])

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        output="screen",
        parameters=[ekf_config],
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=namespace,
    )

    laser_filter_config = PathJoinSubstitution([rosbot_bringup, "config", "laser_filter.yaml"])

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[laser_filter_config],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=namespace,
    )

    green_color = "\033[92m"
    reset_color = "\033[0m"

    status_info = TimerAction(
        period=30.0,
        actions=[LogInfo(msg=f"{green_color}All systems are up and running!{reset_color}")],
    )

    actions = [
        declare_microros_arg,
        declare_namespace_arg,
        controller_launch,
        microros_launch,
        laser_filter_node,
        robot_localization_node,
        status_info,
    ]

    return LaunchDescription(actions)
