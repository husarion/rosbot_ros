import os
import sys
import launch
from launch_ros import get_default_launch_description
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ###########################
    ##  SLAM toolbox Node  ##
    ###########################
    rosbot_description = get_package_share_directory('rosbot_description')
    slam_toolbox_node = launch_ros.actions.Node(
        	parameters=[
                rosbot_description + '/config/slam_toolbox.yaml'
        	],
            package='slam_toolbox',
            node_executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ########################
    ##  Launch Evalution  ##
    ########################
    return launch.LaunchDescription([slam_toolbox_node])

if __name__ == '__main__':
    generate_launch_description()


