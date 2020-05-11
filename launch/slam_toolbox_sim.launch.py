import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    rosbot_description = get_package_share_directory('rosbot_description')
 
    slam_toolbox_node = launch_ros.actions.Node(
        	parameters=[
                rosbot_description + '/config/slam_toolbox_sim.yaml'
        	],
            package='slam_toolbox',
            node_executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        slam_toolbox_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
