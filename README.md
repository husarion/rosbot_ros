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

## Quick start

### Prerequisites

1. Install all necessary tools:

    ```bash
    sudo apt-get update
    sudo apt-get install -y python3-pip ros-dev-tools stm32flash
    ```

2. Create workspace folder and clone `rosbot_ros` repository:

    ```bash
    mkdir -p ros2_ws
    cd ros2_ws
    git clone https://github.com/husarion/rosbot_ros src/rosbot_ros
    ```

### Build

1. Configure environment

    The repository is used to run the code both on the real robot and in the simulation. Specify `HUSARION_ROS_BUILD_TYPE` the variable according to your needs.

    Real robot:

    ``` bash
    export HUSARION_ROS_BUILD_TYPE=hardware
    ```

    Simulation:

    ```bash
    export HUSARION_ROS_BUILD_TYPE=simulation
    ```

2. Install dependencies:

    ``` bash
    ./src/rosbot_ros/rosbot/install_dependencies.sh
    ```

3. Build:

    ```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

>[!NOTE]
> To build code on a real robot you need to run above commands on the Panther Built-in Computer.

### Running

#### Real robot

1. Flash firmware:

    ```bash
    sudo su
    source install/setup.bash
    ros2 run rosbot_utils flash_firmware
    exit
    ```

> ![NOTE]
> To run the software on real ROSbot 2R, 2 PRO, communication with the CORE2 will be necessary.
> First update your firmware to make sure that you use the latest version, then run the `micro-ROS` agent.
> For detailed instructions refer to the [rosbot_ros2_firmware repository](https://github.com/husarion/rosbot_ros2_firmware).

2. Launch:

    ```bash
    source install/setup.bash
    ros2 launch rosbot_bringup bringup.launch.py
    ```

#### Simulation

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
cd src/rosbot_ros
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
