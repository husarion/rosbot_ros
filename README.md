# Rosbot ROS
ROS2 packages for ROSbot 2R and ROSbot 2 PRO.

## ROS packages
### `rosbot`
Metapackage that contains dependencies to other repositories.

### `rosbot_bringup`
Package that contains launch, which starts all base functionalities. Also configs for [robot_localization](https://github.com/cra-ros-pkg/robot_localization) and [ros2_controllers](https://github.com/ros-controls/ros2_controllers) are defined there.

### `rosbot_description`
URDF model used as a source of transforms on the physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

### `rosbot_gazebo`
Launch files for Ignition Gazebo working with ROS2 control.

### `rosbot_controller`
ROS2 hardware controllers configuration for ROSbots.

## Demo
Bellow you can find demos with ROSbots:
- in [rosbot-docker](https://github.com/husarion/rosbot-docker/tree/ros2) you will find a simple example how to use ROSbot.
- in [rosbot-mapping](https://github.com/husarion/rosbot-mapping) you will find an example how to use ROSbot with the [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/).
- in [rosbot-navigation](https://github.com/husarion/rosbot-navigation) you will find an example how to use ROSbot with the [navigation2](https://github.com/ros-planning/navigation2) stack.
