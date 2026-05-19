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
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_moveit").joint_limits(
            file_path="config/joint_limits.yaml"
        )
    ).to_moveit_configs()

    # `home` uses MoveGroupInterface, which loads a RobotModelLoader that needs
    # `robot_description_kinematics` to avoid the "No kinematics plugins defined" warning.
    # No namespace arg here: this file is included from rosbot_controller/launch/manipulator.yaml,
    # which is itself inside the bringup's push_ros_namespace(rosbot_xl) group. Declaring a
    # `namespace` LaunchArgument here would conflict with the parent's `namespace` value and
    # PushRosNamespace would double-stack (saw /rosbot_xl/rosbot_xl/move_to_home_pose in HW
    # testing). Standalone invocation can use `ros2 run rosbot_moveit home --ros-args
    # -r __ns:=/<ns>`; see `dock.launch.py` for the standalone-friendly variant.
    home_node = Node(
        package="rosbot_moveit",
        executable="home",
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    return LaunchDescription([home_node])
