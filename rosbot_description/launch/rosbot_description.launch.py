from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rosbot_description = get_package_share_directory("rosbot_description")
    xacro_file = PathJoinSubstitution([rosbot_description, "urdf", "rosbot.urdf.xacro"])

    robot_description = {
        "robot_description": Command(["xacro --verbosity 0 ", xacro_file])
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    actions = [
        robot_state_publisher_node,
    ]

    return LaunchDescription(actions)
