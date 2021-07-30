import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rosbot_description = get_package_share_directory('rosbot_description')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/rosbot_sim.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/rosbot_navigation_sim.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/slam_toolbox_sim.launch.py']),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            node_node_node_node_executable='rviz2',
            name="rviz2",
            arguments=['-d', rosbot_description+"/config/rosbot.rviz"],
        ),
    ])