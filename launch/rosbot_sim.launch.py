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
from launch_ros.actions import Node


def generate_launch_description():
    world_file_name =  'willow_garage.world'

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'urdf/rosbot.urdf'
    urdf = os.path.join(
        get_package_share_directory('rosbot_description'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    gazebo_ros = get_package_share_directory('gazebo_ros')
    rosbot_description = get_package_share_directory('rosbot_description')

    gazebo_client = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
                condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    )
    
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )

    spawn_rosbot = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(rosbot_description, 'launch', 'rosbot_spawn.launch.py'))
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(rosbot_description, 'worlds', world_file_name), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        DeclareLaunchArgument('gdb', default_value='false',
                              description='Set "true" to run gzserver with gdb'),
        DeclareLaunchArgument('state', default_value='true',
                                description='Set "false" not to load "libgazebo_ros_state.so"'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        gazebo_server,
        gazebo_client,
        spawn_rosbot,
    ])

if __name__ == '__main__':
    generate_launch_description()
