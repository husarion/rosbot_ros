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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x", default="0.0")
    y = LaunchConfiguration("y", default="2.0")
    z = LaunchConfiguration("z", default="0.0")
    roll = LaunchConfiguration("roll", default="0.0")
    pitch = LaunchConfiguration("pitch", default="0.0")
    yaw = LaunchConfiguration("yaw", default="0.0")

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace for all topics and tfs",
    )

    declare_robots_arg = DeclareLaunchArgument(
        "robots",
        default_value="",
        description=(
            "Spawning multiple robots at positions with yaw orientations e.g."
            "robots:='robot1={x: 0.0, y: -1.0}; robot2={x: 1.0, y: -1.0}; robot3={x: 2.0, y: -1.0};'"
        ),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_gz_worlds"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_log_level": "1"}.items(),
    )

    robots_list = ParseMultiRobotPose("robots").value()
    if len(robots_list) == 0:
        robots_list = {
            namespace: {
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
        }
    else:
        for robot_name in robots_list:
            init_pose = robots_list[robot_name]
            for key, value in init_pose.items():
                init_pose[key] = TextSubstitution(text=str(value))

    spawn_group = []
    for idx, robot_name in enumerate(robots_list):
        init_pose = robots_list[robot_name]
        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("rosbot_gazebo"),
                        "launch",
                        "spawn.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "use_sim": "True",
                "namespace": robot_name,
                "x": init_pose["x"],
                "y": init_pose["y"],
                "z": init_pose["z"],
                "roll": init_pose["roll"],
                "pitch": init_pose["pitch"],
                "yaw": init_pose["yaw"],
            }.items(),
        )
        spawn_robot_delay = TimerAction(period=5.0 * idx, actions=[spawn_robot])
        spawn_group.append(spawn_robot_delay)

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_robots_arg,
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            *spawn_group,
        ]
    )
