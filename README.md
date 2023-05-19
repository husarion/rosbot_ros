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

## Source build

### Prerequisites

Install `colcon`, `vsc` and `rosdep`:
```
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-vcstool python3-rosdep
```

Create workspace folder and clone `rosbot_ros` repository:
```
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/husarion/rosbot_ros src/
```

### Build and run on hardware

Building:
```
export HUSARION_ROS_BUILD=hardware

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot/rosbot_hardware.repos

rm -r src/rosbot_gazebo

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build
```

Running:
```
source install/setup.bash
ros2 launch rosbot_bringup bringup.launch.py
```

### Build and run Gazebo simulation

Building:
```
export HUSARION_ROS_BUILD=simulation

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot/rosbot_hardware.repos
vcs import src < src/rosbot/rosbot_simulation.repos

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build
```

Running:
```
source install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py
```

## Demo
Bellow you can find demos with ROSbots:
- in [rosbot-docker](https://github.com/husarion/rosbot-docker/tree/ros2) you will find a simple example how to drive ROSbot with `teleop_twist_keyboard`.
- in [rosbot-sensors](https://github.com/husarion/rosbot-sensors) you will find an example how to visualize all ROSbot sensors.
- in [rosbot-mapping](https://github.com/husarion/rosbot-mapping) you will find an example how to use ROSbot with the [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/).
- in [rosbot-navigation](https://github.com/husarion/rosbot-navigation) you will find an example how to use ROSbot with the [navigation2](https://github.com/ros-planning/navigation2) stack.