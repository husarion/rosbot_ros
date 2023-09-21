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
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
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

    rosbot_controller = get_package_share_directory("rosbot_controller")
    rosbot_bringup = get_package_share_directory("rosbot_bringup")

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description=(
            "Whether to use mecanum drive controller "
            "(otherwise diff drive controller is used)"
        ),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_controller,
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim": use_sim,
            "mecanum": mecanum,
            "use_gpu": use_gpu,
            "simulation_engine": simulation_engine,
        }.items(),
    )

    ekf_config = PathJoinSubstitution([rosbot_bringup, "config", "ekf.yaml"])

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
    )

    actions = [
        declare_mecanum_arg,
        declare_use_sim_arg,
        declare_use_gpu_arg,
        declare_simulation_engine_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        controller_launch,
        robot_localization_node,
    ]

    return LaunchDescription(actions)
