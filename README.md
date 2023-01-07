# Rosbot ROS
ROS2 packages for ROSbot 2R and ROSbot 2 PRO.

## ROS packages

### `rosbot_bringup`
Package that contains launch, which starts all base functionalities. Also configs for [robot_localization](https://github.com/cra-ros-pkg/robot_localization) and [ros2_controllers](https://github.com/ros-controls/ros2_controllers) are defined there.

### `rosbot_description`
URDF model used as a source of transforms on the physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

### `rosbot_hardware_interfaces`
ROS2 hardware controllers for ROSbots. Inputs and outputs data from ROS2 control and forwards it via ROS topic to be read by microros.

## Demo
In [rosbot-docker](https://github.com/husarion/rosbot-docker/tree/ros2) you will find an example how to use ROSbot with the navigation stack.
