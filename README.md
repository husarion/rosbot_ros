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

Install all necessary tools:

```bash
sudo apt-get update
sudo apt-get install -y python3-pip ros-dev-tools stm32flash
```

Create workspace folder and clone `rosbot_ros` repository:

```bash
mkdir -p ros2_ws/src
cd ros2_ws
git clone https://github.com/husarion/rosbot_ros src/
```

### Build and run on hardware

Building:

```bash
export HUSARION_ROS_BUILD=hardware

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot/rosbot_hardware.repos

# Build only diff_drive_controller and imu_sensor_broadcaster from ros2_controllers
cp -r src/ros2_controllers/diff_drive_controller src && cp -r src/ros2_controllers/imu_sensor_broadcaster src && rm -rf src/ros2_controllers

rm -r src/rosbot_gazebo

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Flash firmware:

```bash
source install/setup.bash
ros2 run rosbot_utils flash_firmware
```

Running:

```bash
source install/setup.bash
ros2 launch rosbot_bringup combined.launch.py
```

### Build and run Gazebo simulation

Prerequisites:

> [!TIP]
> The default version of Gazebo Ignition will be installed with the instructions below. If you want to install a different version of the simulator, it is necessary to:
>
> - Check compatible versions of ROS 2 and Gazebo in [this table](https://gazebosim.org/docs/garden/ros_installation#summary-of-compatible-ros-and-gazebo-combinations)
> - [Install the appropriate version](https://gazebosim.org/docs/fortress/install_ubuntu#binary-installation-on-ubuntu),
> - Add the `GZ_VERSION` environment variable appropriate to your version
>
>   ```bash
>   export GZ_VERSION=fortress
>   ```

If you have installed multiple versions of Gazebo use the global variable to select the correct one:

```bash
export GZ_VERSION=fortress
```

Building:

```bash
export HUSARION_ROS_BUILD=simulation

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot/rosbot_hardware.repos
vcs import src < src/rosbot/rosbot_simulation.repos

# Build only diff_drive_controller and imu_sensor_broadcaster from ros2_controllers
cp -r src/ros2_controllers/diff_drive_controller src && cp -r src/ros2_controllers/imu_sensor_broadcaster src && rm -rf src/ros2_controllers

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Running:

```bash
source install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py
```

## Developer info

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

```bash
colcon test
```

> [!NOTE]
> Command `colcon test` does not build the code. Remember to build your code after changes.

If tests finish with errors print logs:

``` bash
colcon test-result --verbose
```

### Format python code with [Black](https://github.com/psf/black)

```bash
cd src/
black rosbot*
```

### Testing `.github/workflows/industrial_ci.yaml` Locally

At fist install [act](https://github.com/nektos/act):

```bash
cd /
curl -s https://raw.githubusercontent.com/nektos/act/master/install.sh | sudo bash
```

And test the workflow with:

```bash
act -W .github/workflows/industrial_ci.yaml
```

## Demo

Below you can find demos with ROSbots:
| link | description |
| - | - |
| [rosbot-docker](https://github.com/husarion/rosbot-docker/tree/ros2) | Simple example how to drive ROSbot with `teleop_twist_keyboard` using docker |
| [rosbot-sensors](https://github.com/husarion/rosbot-sensors) | Visualize all ROSbot sensors |
| [rosbot-gamepad](https://github.com/husarion/rosbot-gamepad) | Stream a live video from Orbbec Astra to a window on your PC. Control the robot using `teleop-twist-keyboard` |
| [rosbot-telepresence](https://github.com/husarion/rosbot-telepresence) | Stream a live video from Orbbec Astra to a window on your PC. Control the robot using `teleop-twist-keyboard` |
| [rosbot-autonomy](https://github.com/husarion/rosbot-autonomy) | A combination of `mapping` and `navigation` projects allowing simultaneous mapping and navigation in unknown environments.  |
