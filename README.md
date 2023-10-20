# Rosbot ROS
ROS2 packages for ROSbot 2R and ROSbot 2 PRO.

## ROS packages
### `rosbot`
Metapackage that contains dependencies to other repositories.

### `rosbot_bringup`
Package that contains launch, which starts all base functionalities. Also configuration for [robot_localization](https://github.com/cra-ros-pkg/robot_localization) and [ros2_controllers](https://github.com/ros-controls/ros2_controllers) are defined there.

### `rosbot_description`
URDF model used as a source of transforms on the physical robot. It was written to be compatible with ROS Industrial and preconfigured for ROS2 control.

### `rosbot_gazebo`
Launch files for Ignition Gazebo working with ROS2 control.

### `rosbot_controller`
ROS2 hardware controllers configuration for ROSbots.

## ROS API

Available in [ROS_API.md](./ROS_API.md)

## Usage on hardware

To run the software on real ROSbot 2R, 2 PRO, also communication with the CORE2 will be necessary.
First update your firmware to make sure that you use the latest version, then run the `micro-ROS` agent.
For detailed instructions refer to the [rosbot_ros2_firmware repository](https://github.com/husarion/rosbot_ros2_firmware).

## Source build

### Prerequisites

Install `colcon`, `vcs` and `rosdep`:
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

> **Prerequisites**
>
> Before starting the software on the robot please make sure that you're using the latest firmware and run the `micro-ROS` agent (as described in the *Usage on hardware* step).

Running:
```
source install/setup.bash
ros2 launch rosbot_bringup bringup.launch.py
```

### Build and run Gazebo simulation
Prerequisites:

> **Warning**
> The simulation is compatible with the Gazebo Fortress LTS version. Use [this installation guide ](https://gazebosim.org/docs/fortress/install_ubuntu#binary-installation-on-ubuntu) to install the proper version and remove the another versions e. g. Gazebo Garden:
> `sudo apt remove gz-garden && sudo apt autoremove`
> Look at [the table](https://gazebosim.org/docs/garden/ros_installation#summary-of-compatible-ros-and-gazebo-combinations) to see the compatible ROS 2 and Gazebo versions.

If you have installed multiple versions of Gazebo use the global variable to select the correct one:
```bash
export GZ_VERSION=fortress
```

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

## Testing package

### pre-commit
[pre-commit configuration](.pre-commit-config.yaml) prepares plenty of tests helping for developing and contributing. Usage:

```bash
# install pre-commit
pip install pre-commit

# initialize pre-commit workspace
pre-commit install

# manually run tests
pre-commit run -a
```

After initialization [pre-commit configuration](.pre-commit-config.yaml) will applied on every commit.

### Industrial CI
```
colcon test
```

> [!NOTE]
> Command `colcon test` does not build the code. Remember to build your code after changes.

If tests finish with errors print logs:
```
colcon test-result --verbose
```

### Format python code with [Black](https://github.com/psf/black)
```
cd src/
black rosbot*
```

## Demo
Bellow you can find demos with ROSbots:
- in [rosbot-docker](https://github.com/husarion/rosbot-docker/tree/ros2) you will find a simple example how to drive ROSbot with `teleop_twist_keyboard`.
- in [rosbot-sensors](https://github.com/husarion/rosbot-sensors) you will find an example how to visualize all ROSbot sensors.
- in [rosbot-mapping](https://github.com/husarion/rosbot-mapping) you will find an example how to use ROSbot with the [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox/).
- in [rosbot-navigation](https://github.com/husarion/rosbot-navigation) you will find an example how to use ROSbot with the [navigation2](https://github.com/ros-planning/navigation2) stack.
