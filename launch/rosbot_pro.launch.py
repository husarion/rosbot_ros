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
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    rosbot_description = get_package_share_directory('rosbot_description')
    rplidar_ros = get_package_share_directory('rplidar_ros')
    
    camera_depth_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['0', '0', '0', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_depth_frame'],
        parameters=[
    		rosbot_description + '/config/static_tf.yaml'
    	],
    )
    
    camera_link_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['-0.03', '0', '0.11', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[
    		rosbot_description + '/config/static_tf.yaml'
        ]
    )

    laser_frame_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.08', '-3.141593', '0.0', '0.0', 'base_link', 'laser'],
        parameters=[
    		rosbot_description + '/config/static_tf.yaml'
        ]
    )

    rosserial = launch_ros.actions.Node(
        package='rosbot_description',
        executable='rosserial_node.py',
        output='screen',
        parameters=[
    		rosbot_description + '/config/rosserial_pro.yaml'
        ]
    )

    rosbot_tf = launch_ros.actions.Node(
        package='rosbot_description',
        executable='rosbot_tf',
        output='log',
    )

    rp_lidar = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros, 'launch', 'rplidar_a3.launch.py'))
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        rp_lidar,
        laser_frame_tf,
        rosserial,
        rosbot_tf,
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.08', '0.1', '0', '0', '0', '0', 'base_link', 'front_left_wheel'],
            ),        
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.08', '-0.1', '0', '0', '0', '0', 'base_link', 'front_right_wheel'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.08', '0.1', '0', '0', '0', '0', 'base_link', 'rear_left_wheel'],
            ),        
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.08', '-0.1', '0', '0', '0', '0', 'base_link', 'rear_right_wheel'],
            ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'top'],
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'camera_link'],
        ),

    ])

if __name__ == '__main__':
    generate_launch_description()
