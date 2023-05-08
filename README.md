# Rosbot ROS
ROS2 packages for ROSbot 2R and ROSbot 2 PRO.

## ROS packages
### `rosbot`
Metapackage that contains dependencies to other repositories.

### `rosbot_bringup`
Package that contains launch, which starts all base functionalities. Also configs for [robot_localization](https://github.com/cra-ros-pkg/robot_localization) and [ros2_controllers](https://github.com/ros-controls/ros2_controllers) are defined there.

### `rosbot_description`
URDF model used as a source of transforms on the physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

### `rosbot_controller`
ROS2 hardware controllers configuration for ROSbots.

## Demo
Bellow you can find demos with ROSbots:
- in [rosbot-docker](https://github.com/husarion/rosbot-docker/tree/ros2) you will find a simple example how to use ROSbot.
- in [rosbot-mapping](https://github.com/husarion/rosbot-mapping) you will find an example how to use ROSbot with the [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/).
- in [rosbot-navigation](https://github.com/husarion/rosbot-navigation) you will find an example how to use ROSbot with the [navigation2](https://github.com/ros-planning/navigation2) stack.

# Build
### Prerequirements
```bash
mkdir -p rosbot_ws/src
cd rosbot_ws
git clone https://github.com/husarion/rosbot_ros src/
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep python3-vcstool python3-rosdep2
```

### Build
Inside `rosbot_ws` catalog:
```bash
vcs import src < src/rosbot/rosbot_hardware.repos
vcs import src < src/rosbot/rosbot_simulation.repos
rosdep init
rosdep update --rosdistro humble
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build
```

# Install and run
### Installation
Inside `rosbot_ws` catalog:
```bash
source install/setup.bash
```

### Running hardware
After installation:
```bash
ros2 launch rosbot_bringup bringup.launch.py
```

### Running simulation
After installation:
```bash
ros2 launch rosbot_gazebo simulation.launch.py
```