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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_gz_worlds"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_log_level": "1"}.items(),
    )

    gz_bridge_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_gazebo"), "config", "gz_bridge.yaml"]
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gz_bridge_config}],
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_gazebo"),
                    "launch",
                    "spawn_robot.launch.py",
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            SetRemap("/diagnostics", "diagnostics"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_bridge,
            spawn_robot,
        ]
    )
