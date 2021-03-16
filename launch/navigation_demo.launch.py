import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rosbot_description = get_package_share_directory('rosbot_description')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/rosbot.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/rosbot_navigation_pro.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/slam_toolbox.launch.py']),
        ),
    ])