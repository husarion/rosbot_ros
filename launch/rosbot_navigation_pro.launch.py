import os

from launch import LaunchDescription
import launch.actions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    rosbot_description = get_package_share_directory('rosbot_description')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time',
                                                            default='false')
    autostart = launch.substitutions.LaunchConfiguration('autostart')
    params_file = launch.substitutions.LaunchConfiguration('params')
    default_bt_xml_filename = launch.substitutions.LaunchConfiguration(
        'default_bt_xml_filename')

    remappings = []

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
    }

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        launch.actions.DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        launch.actions.DeclareLaunchArgument(
            'params',
            default_value=[rosbot_description,
                           '/config/nav2_params_pro.yaml'],
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        launch_ros.actions.Node(
            package='nav2_controller',
            node_executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        launch_ros.actions.Node(
            package='nav2_planner',
            node_executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),


        launch_ros.actions.Node(
            package='nav2_recoveries',
            node_executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=remappings),

        launch_ros.actions.Node(
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        launch_ros.actions.Node(
            package='nav2_waypoint_follower',
            node_executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

    ])
