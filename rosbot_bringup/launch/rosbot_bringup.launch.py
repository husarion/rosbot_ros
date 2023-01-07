from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import  PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    rosbot_hardware = get_package_share_directory("rosbot_hardware")
    rosbot_bringup = get_package_share_directory("rosbot_bringup")

    rosbot_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    rosbot_hardware,
                    "launch",
                    "controller.launch.py",
                ]
            )
        )
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
        rosbot_hardware_launch,
        robot_localization_node,
    ]

    return LaunchDescription(actions)
