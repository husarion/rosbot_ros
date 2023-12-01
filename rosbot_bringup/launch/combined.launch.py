from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for the serial port and baud rate
    declare_serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='//dev/ttySERIAL',  # Default value, replace if needed
        description='Serial port for micro-ROS agent'
    )

    declare_serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='576000',  # Default baud rate, replace if needed
        description='Baud rate for serial communication'
    )

    # Retrieve launch configurations
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')

    # Node configuration for micro-ROS agent
    microros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '-D', serial_port, '-b', serial_baudrate],
        output='screen'
    )

    # Include bringup.launch.py
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/bringup.launch.py'])
    )

    return LaunchDescription([
        declare_serial_port_arg,
        declare_serial_baudrate_arg,
        microros_agent_node,
        bringup_launch
    ])
