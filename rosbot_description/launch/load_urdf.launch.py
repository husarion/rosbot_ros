#!/usr/bin/env python3

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

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare


def contains_cam_component(yaml_fil):
    with open(yaml_fil, "r") as file:
        data = yaml.safe_load(file)
        if "components" in data:
            return any(item["type"].startswith("CAM") for item in data["components"])
    return False


def launch_setup(context, *args, **kwargs):
    components_config = LaunchConfiguration("components_config").perform(context)
    configuration = LaunchConfiguration("configuration").perform(context)
    controller_config = LaunchConfiguration("controller_config").perform(context)
    mecanum = LaunchConfiguration("mecanum").perform(context)
    mock_joints = LaunchConfiguration("mock_joints", default="True").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    use_sim = LaunchConfiguration("use_sim", default="False").perform(context)

    if robot_model != "rosbot_xl" and configuration != "basic":
        raise ValueError("Invalid configuration and robot model combination. Only 'rosbot_xl' has configuration options.")

    urdf_file = robot_model + ".urdf.xacro"
    include_camera_mount = str(contains_cam_component(components_config))
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("rosbot_description"), "urdf", urdf_file]),
            " components_config:=",
            components_config,
            " configuration:=",
            configuration,
            " controller_config:=",
            controller_config,
            " include_camera_mount:=",
            include_camera_mount,
            " mecanum:=",
            mecanum,
            " namespace:=",
            namespace,
            " use_sim:=",
            use_sim,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(mock_joints),
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),
        robot_state_pub_node,
        joint_state_publisher_node,
    ]


def generate_launch_description():
    configuration = LaunchConfiguration("configuration")
    robot_model = LaunchConfiguration("robot_model")
    components_file = PythonExpression(["'", configuration, "' + '.yaml'"])
    default_components_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "config", robot_model, components_file]
    )
    declare_components_config_arg = DeclareLaunchArgument(
        "components_config",
        default_value=default_components_config,
        description=(
            "Specify file which contains components. These components will be included in URDF. "
            "Available options can be found in [ros_components_description](https://github.com/husarion/ros_components_description/blob/jazzy/README.md#available-urdf-sensors)"
        ),
    )

    declare_configuration_arg = DeclareLaunchArgument(
        "configuration",
        default_value="basic",
        description=(
            "Specify configuration packages. Currently only ROSbot XL has available packages"
        ),
        choices=["basic", "telepresence", "autonomy", "manipulation", "manipulation_pro"]
    )

    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller, otherwise use diff drive",
        choices=["True", "False"],
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value=""),
        description="Specify robot model",
        choices=["rosbot", "rosbot_xl"],
    )

    publish_robot_description = OpaqueFunction(function=launch_setup)

    return LaunchDescription(
        [
            declare_configuration_arg,
            declare_robot_model_arg,
            declare_components_config_arg, # depends on configuration and robot model
            declare_mecanum_arg,
            publish_robot_description,
        ]
    )
