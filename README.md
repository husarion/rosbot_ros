# Rosbot ROS
ROS2 packages for ROSbot 2R and ROSbot 2 PRO.

## ROS packages
### `rosbot`
Metapackage that contains dependencies to other repositories.

### `rosbot_bringup`
Package that contains launch, which starts all base functionalities. Also configs for [robot_localization](https://github.com/cra-ros-pkg/robot_localization).

### `rosbot_description`
URDF model used as a source of transforms on the physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

### `rosbot_hardware`
ROS2 hardware controllers configuration for ROSbots.

## Demo
In [rosbot-mapping](https://github.com/husarion/rosbot-mapping) you will find an example how to use ROSbot with the slam_toolbox.
In [rosbot-navigation](https://github.com/husarion/rosbot-navigation) you will find an example how to use ROSbot with the navigation stack.

## ROS API
Available in [ROS_API.md](./ROS_API.md)