from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_microros_agent_node(context, *args, **kwargs):
    serial_port = LaunchConfiguration('serial_port').perform(context)
    serial_baudrate = LaunchConfiguration('serial_baudrate').perform(context)
    localhost_only_fastrtps_profiles_file = LaunchConfiguration('localhost_only_fastrtps_profiles_file').perform(context)

    # Check ROS_LOCALHOST_ONLY environment variable
    if os.environ.get('ROS_LOCALHOST_ONLY') == '1':
        # Set RMW_IMPLEMENTATION and FASTRTPS_DEFAULT_PROFILES_FILE
        rmw_implementation = 'rmw_fastrtps_cpp'
        return [
            SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value=rmw_implementation),
            SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=localhost_only_fastrtps_profiles_file),
            Node(
                package='micro_ros_agent',
                executable='micro_ros_agent',
                arguments=['serial', '-D', serial_port, '-b', serial_baudrate],
                output='screen'
            )
        ]
    else:
        return [
            Node(
                package='micro_ros_agent',
                executable='micro_ros_agent',
                arguments=['serial', '-D', serial_port, '-b', serial_baudrate],
                output='screen'
            )
        ]

def generate_launch_description():
    declare_serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttySERIAL',
        description='Serial port for micro-ROS agent'
    )

    declare_serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='576000',
        description='Baud rate for serial communication'
    )

    declare_localhost_only_fastrtps_profiles_file_arg = DeclareLaunchArgument(
        'localhost_only_fastrtps_profiles_file',
        default_value='/microros_locahost_only.xml',
        description='Path to the Fast RTPS default profiles file'
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/bringup.launch.py'])
    )

    return LaunchDescription([
        declare_serial_port_arg,
        declare_serial_baudrate_arg,
        declare_localhost_only_fastrtps_profiles_file_arg,
        OpaqueFunction(function=generate_microros_agent_node),
        bringup_launch
    ])


