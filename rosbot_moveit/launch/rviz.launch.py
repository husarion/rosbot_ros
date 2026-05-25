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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder


def _resolve_rviz_config_and_spawn(context):
    """Patch the bundled moveit.rviz so the MotionPlanning panel reaches the
    namespaced ``move_group``. RViz hardcodes ``Move Group Namespace: ""`` in
    its config; without a substitution the panel constructs MGI on / and
    cannot find /<ns>/move_group. Mirrors the resolved-config pattern used by
    rosbot_controller's controller.yaml (sed -> /tmp/rosbot_*_<ns>.<ext>).
    """
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    move_group_ns = "/" + namespace if namespace else ""
    resolved_path = (
        f"/tmp/rosbot_moveit_{namespace}.rviz" if namespace else "/tmp/rosbot_moveit.rviz"
    )
    source_path = os.path.join(
        get_package_share_directory("rosbot_moveit"), "config", "moveit.rviz"
    )
    with open(source_path) as f:
        content = f.read()
    content = content.replace(
        'Move Group Namespace: ""', f'Move Group Namespace: "{move_group_ns}"'
    )
    with open(resolved_path, "w") as f:
        f.write(content)

    moveit_config = MoveItConfigsBuilder(
        "rosbot_xl", package_name="rosbot_moveit"
    ).to_moveit_configs()

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            namespace=namespace,
            arguments=["-d", resolved_path],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
            # Same /tf, /tf_static, /diagnostics remap as every other node in
            # the bringup group: keep TF traffic on the namespaced topics so
            # RViz reads the same tree the rest of the stack publishes.
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("/diagnostics", "diagnostics"),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim",
                default_value="False",
                description="Whether simulation is used",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Robot namespace; matches the bringup so RViz reaches the namespaced move_group.",
            ),
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim")),
            OpaqueFunction(function=_resolve_rviz_config_and_spawn),
        ]
    )
