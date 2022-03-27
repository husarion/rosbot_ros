import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    rosbot_description = get_package_share_directory('rosbot_description')
    rosbot_rviz_config_path = os.path.join(rosbot_description, 'rviz/rosbot.rviz')
    xacro_file = os.path.join(rosbot_description, 'models', 'rosbot', 'rosbot.urdf.xacro')
    
    
    robot_description = {
      'robot_description' : Command([
        'xacro --verbosity 0 ', xacro_file,
        ' use_sim:=', use_sim_time
      ])
    }

    return LaunchDescription([
      DeclareLaunchArgument(
        name='rvizconfig',
        default_value=rosbot_rviz_config_path,
        description='Absolute path to rviz config file'
      ),
      
      Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
      )
    ])


if __name__ == '__main__':
  generate_launch_description()