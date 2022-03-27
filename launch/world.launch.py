#!/usr/bin/env python3
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

  gazebo_ros = get_package_share_directory('gazebo_ros')
  
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  world = LaunchConfiguration('world', default='')

  return LaunchDescription([

    DeclareLaunchArgument(
      name='gui',
      default_value='true'
    ),

    DeclareLaunchArgument(
      name='use_sim_time',
      default_value=use_sim_time
    ),

    DeclareLaunchArgument(
      name='world',
      default_value=[world],
      description='Path to Gazebo world file.'
    ),

    DeclareLaunchArgument(
      name='verbose',
      default_value='true',
      description='Set "true" to increase debug message level.'
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
    ),
  ])

if __name__ == '__main__':
    generate_launch_description()