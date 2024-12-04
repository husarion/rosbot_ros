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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all topics and tfs",
    )

    declare_x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Initial robot position in the global 'x' axis."
    )

    declare_y_arg = DeclareLaunchArgument(
        "y", default_value="-2.0", description="Initial robot position in the global 'y' axis."
    )

    declare_z_arg = DeclareLaunchArgument(
        "z", default_value="0.0", description="Initial robot position in the global 'z' axis."
    )

    declare_roll_arg = DeclareLaunchArgument(
        "roll", default_value="0.0", description="Initial robot 'roll' orientation."
    )

    declare_pitch_arg = DeclareLaunchArgument(
        "pitch", default_value="0.0", description="Initial robot 'pitch' orientation."
    )

    declare_yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="Initial robot 'yaw' orientation."
    )

    namespace_ext = PythonExpression(
        ["''", " if '", namespace, "' == '' ", "else ", "'/", namespace, "'"]
    )

    gz_remappings_file = PathJoinSubstitution(
        [FindPackageShare("rosbot_gazebo"), "config", "gz_remappings.yaml"]
    )

    namespaced_gz_remappings_file = ReplaceString(
        source_file=gz_remappings_file,
        replacements={"<robot_namespace>": (namespace_ext)},
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            namespace,
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
        namespace=namespace,
    )

    welcome_msg = LogInfo(
        msg=[
            "Spawning ROSbot\n\tNamespace: '",
            namespace,
            "'\n\tInitial pose: (",
            x,
            ", ",
            y,
            ", ",
            z,
            ", ",
            roll,
            ", ",
            pitch,
            ", ",
            yaw,
            ")",
        ]
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        parameters=[{"config_file": namespaced_gz_remappings_file}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        namespace=namespace,
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim": "True",
            "namespace": namespace,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_x_arg,
            declare_y_arg,
            declare_z_arg,
            declare_roll_arg,
            declare_pitch_arg,
            declare_yaw_arg,
            SetParameter(name="use_sim_time", value=True),
            welcome_msg,
            ign_bridge,
            gz_spawn_entity,
            bringup_launch,
        ]
    )
