#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  rosbot_description = get_package_share_directory('rosbot_description')
  world = LaunchConfiguration('world', default=os.path.join(rosbot_description, 'worlds', ' turtlebot_playground.world'))

  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([rosbot_description, '/launch/world.launch.py']),
      launch_arguments = {
        'world' : world,
      }.items(),
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([rosbot_description, '/launch/rosbot_spawn.launch.py']),
      launch_arguments = {
        'pos_x' : '0.0',
        'pos_y' : '0.0',
      }.items(),
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([rosbot_description, '/launch/rviz2.launch.py'])
    ),
  ])


if __name__ == '__main__':
  generate_launch_description()