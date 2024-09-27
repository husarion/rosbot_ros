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

# Import necessary modules
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
# from nav2_common.launch import ReplaceString

def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all topics and TFs",
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
            " == 'True' else 'diff_drive_controller.yaml'",
        ]
    )

    namespace_ext = PythonExpression(
        ["''", " if '", namespace, "' == '' ", "else ", "'", namespace, "/'"]
    )

    controller_manager_name = LaunchConfiguration(
        "controller_manager_name",
        default=[namespace_ext, "controller_manager"],
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
            " namespace:=",
            namespace,
            # Uncomment the line below if you need to include the 'use_ros2_control' parameter
            # " use_ros2_control:=True",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller configurations
    # robot_controllers_config = PathJoinSubstitution(
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rosbot_controller"),
            "config",
            controller_config_name,
        ]
    )

    # namespaced_robot_controllers_config = ReplaceString(
    #     source_file=robot_controllers_config,
    #     replacements={"<robot_namespace>": namespace, "//": "/"},
    # )

    # Define nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            # namespaced_robot_controllers_config,
            robot_controllers,
        ],
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
        respawn=True,
        respawn_delay=2.0,
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        namespace=namespace,
    )

    # Create spawner nodes
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "10",
            # "--namespace",
            # namespace,
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
            "10",
            # "--namespace",
            # namespace,
        ],
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "10",
            # "--namespace",
            # namespace,
        ],
    )

    # Wrap the spawner nodes in a TimerAction to delay execution by 2 seconds
    delayed_spawner_nodes = TimerAction(
        period=1.0,
        actions=[
            control_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            imu_broadcaster_spawner,
        ],
    )

    # Set 'use_sim_time' parameter
    # set_use_sim_time = SetParameter('use_sim_time', value=use_sim)fr

    # Assemble the LaunchDescription
    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_use_sim_arg,
            declare_use_gpu_arg,
            declare_simulation_engine_arg,
            SetParameter("use_sim_time", value=use_sim),
            robot_state_pub_node,
            delayed_spawner_nodes,  # Add the delayed spawner nodes here
        ]
    )
