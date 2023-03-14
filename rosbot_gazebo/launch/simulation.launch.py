#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    map_package = get_package_share_directory("husarion_office_gz")
    world_file = PathJoinSubstitution([map_package, "worlds", "husarion_world.sdf"])
    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=["-r ", world_file], description="SDF world file"
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": world_cfg}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "rosbot",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "2.0",
            "-z",
            "0.2",
        ],
        output="screen",
    )
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            "/camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
            "/camera/depth_image" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
            "/camera/image" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
            "/camera/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
        ],
        output="screen",
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim": "True",
            "simulation_engine": "ignition-gazebo"
        }.items(),
    )

    return LaunchDescription(
        [
            declare_world_arg,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            ign_bridge,
            gz_spawn_entity,
            bringup_launch,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
