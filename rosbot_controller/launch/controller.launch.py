# Copyright 2020 ros2_control Development Team
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
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    PythonExpression,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all topics and tfs",
    )

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description=(
            "Whether to use mecanum drive controller (otherwise diff drive controller is used)"
        ),
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    use_gpu = LaunchConfiguration("use_gpu")
    declare_use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="False",
        description="Whether GPU acceleration is used",
    )

    simulation_engine = LaunchConfiguration("simulation_engine")
    declare_simulation_engine_arg = DeclareLaunchArgument(
        "simulation_engine",
        default_value="webots",
        description="Which simulation engine to be used",
        choices=["ignition-gazebo", "gazebo-classic", "webots"],
    )

    controller_config_name = PythonExpression(
        [
            "'mecanum_drive_controller.yaml' if ",
            mecanum,
            " else 'diff_drive_controller.yaml'",
        ]
    )

    controller_manager_type_name = PythonExpression(
        [
            "'simulation_controller_manager' if ",
            use_sim,
            " else 'controller_manager'",
        ]
    )

    controller_manager_name = LaunchConfiguration("controller_manager_name")
    namespace_for_controller_name = PythonExpression(
        ["''", " if '", namespace, "' == '' ", "else ", "'", namespace, "/'"]
    )

    declare_controller_manager_name_arg = DeclareLaunchArgument(
        "controller_manager_name",
        default_value=[namespace_for_controller_name, controller_manager_type_name],
        description="ros2_control controller manager name",
    )

    # Get URDF via xacro
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
            " use_sim:=",
            use_sim,
            " use_gpu:=",
            use_gpu,
            " simulation_engine:=",
            simulation_engine,
            " tf_prefix:=",
            namespace,
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
        ],
        condition=UnlessCondition(use_sim),
        namespace=namespace,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
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
            "120",
            "--namespace",
            namespace,
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rosbot_base_controller",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
            "--namespace",
            namespace,
        ],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
            "--namespace",
            namespace,
        ],
    )

    # Delay start of imu_broadcaster after robot_controller
    # when spawning without delay ros2_control_node sometimes crashed
    delay_imu_broadcaster_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[imu_broadcaster_spawner],
        )
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_use_sim_arg,
            declare_use_gpu_arg,
            declare_simulation_engine_arg,
            declare_controller_manager_name_arg,
            SetParameter("use_sim_time", value=use_sim),
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            delay_imu_broadcaster_spawner_after_robot_controller_spawner,
        ]
    )
