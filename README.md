# Rosbot ROS

ROS 2 packages for Husarion ROSbot series.

![ROSbot](https://husarion.com/assets/images/rosbot3-preview2-f7dee8f0b4ea4de02e80d4dc9f2ca286.png)

## 📚 ROS API

Documentation is available in ROS_API.md.

## 🚀 Quick Start

### ⚙️ Prerequisites

1. Install all necessary tools:

    ```bash
    sudo apt-get update
    sudo apt-get install -y python3-pip ros-dev-tools stm32flash
    ```

2. Create a workspace folder and clone the rosbot_ros repository:

    ```bash
    mkdir -p ros2_ws
    cd ros2_ws
    git clone -b jazzy https://github.com/husarion/rosbot_ros src/rosbot_ros
    ```

### 🤖 Hardware

#### Building

```bash
export HUSARION_ROS_BUILD_TYPE=hardware

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos
cp -r src/ros2_controllers/diff_drive_controller src/
cp -r src/ros2_controllers/imu_sensor_broadcaster src/
rm -rf src/ros2_controllers
rm -r src/rosbot_ros/rosbot_gazebo

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Run the Robot

1. Flash the firmware:

    ```bash
    sudo su
    source install/setup.bash
    ros2 run rosbot_utils flash_firmware
    exit
    ```

> [!NOTE]
> To run the software on real ROSbots, communication with the CORE2 is required. Ensure the firmware is updated before running the micro-ROS agent. For detailed instructions, refer to the rosbot_ros2_firmware repository.

2. Launch the robot:

    ```bash
    source install/setup.bash
    ros2 launch rosbot_bringup bringup.launch.py
    ```

### 🖥️ Simulation

#### Building

```bash
export HUSARION_ROS_BUILD_TYPE=simulation

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos
cp -r src/ros2_controllers/diff_drive_controller src/
cp -r src/ros2_controllers/imu_sensor_broadcaster src/
rm -rf src/ros2_controllers

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Run the Simulation

```bash
source install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py
```

### Launch Arguments

| Symbol | Meaning                      |
| ------ | ---------------------------- |
| 🤖      | Available for physical robot |
| 🖥️      | Available in simulation      |

| 🤖   | 🖥️   | Argument            | Description <br/> ***Type:*** `Default`                                                                                                                                                            |
| --- | --- | ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ✅   | ✅   | `namespace`         | Namespace for all topics and tfs. <br/> ***string:*** `env(ROBOT_NAMESPACE)`                                                                                                                       |
| ✅   | ❌   | `mecanum`           | Whether to use mecanum drive controller (otherwise diff drive controller is used). <br/> ***bool:*** `False`                                                                                       |
| ✅   | ❌   | `microros`          | Automatically connect with hardware using microros. <br/> ***bool:*** `True`                                                                                                                       |
| ✅   | ❌   | `serial_baudrate`   | Baud rate for serial communication . <br/> ***string:*** `576000`                                                                                                                                  |
| ✅   | ❌   | `serial_port`       | Automatically connect with hardware using microros. <br/> ***string:*** `/dev/ttySERIAL`                                                                                                           |
| ✅   | ❌   | `fastrtps_profiles` | Path to the Fast RTPS default profiles file for Micro-ROS agent for localhost only setup. <br/> ***string:*** [`microros_localhost_only.xml`](./rosbot_bringup/config/microros_localhost_only.xml) |
| ❌   | ✅   | `gz_gui`            | Run simulation with specific GUI layout. <br/> ***string:*** [`teleop.config`](https://github.com/husarion/husarion_gz_worlds/blob/main/config/teleop.config)                                      |
| ❌   | ✅   | `gz_headless_mode`  | Run the simulation in headless mode. Useful when a GUI is not needed or to reduce the number of calculations. <br/> ***bool:*** `False`                                                            |
| ❌   | ✅   | `gz_log_level`      | Adjust the level of console output. <br/> ***int:*** `1` (choices: `0`, `1`, `2`, `3`, `4`)                                                                                                        |
| ❌   | ✅   | `gz_world`          | Absolute path to SDF world file. <br/> ***string:*** [`husarion_world.sdf`](https://github.com/husarion/husarion_gz_worlds/blob/main/worlds/husarion_world.sdf)                                    |
| ❌   | ✅   | `robots`            | Spawning multiple robots at positions with yaw orientations e.g.robots:='robot1={x: 0.0, y: -1.0}; robot2={x: 1.0, y: -1.0};' <br/> ***string:*** `''`                                             |
| ❌   | ✅   | `x`                 | Initial robot position in the global 'x' axis. <br/> ***float:*** `0.0`                                                                                                                            |
| ❌   | ✅   | `y`                 | Initial robot position in the global 'y' axis. <br/> ***float:*** `2.0`                                                                                                                           |
| ❌   | ✅   | `z`                 | Initial robot position in the global 'z' axis. <br/> ***float:*** `0.0`                                                                                                                            |
| ❌   | ✅   | `roll`              | Initial robot 'roll' orientation. <br/> ***float:*** `0.0`                                                                                                                                         |
| ❌   | ✅   | `pitch`             | Initial robot 'pitch' orientation. <br/> ***float:*** `0.0`                                                                                                                                        |
| ❌   | ✅   | `yaw`               | Initial robot 'yaw' orientation. <br/> ***float:*** `0.0`                                                                                                                                          |

> [!TIP]
>
> To read the arguments for individual packages, add the `-s` flag to the `ros2 launch` command (e.g. `ros2 launch rosbot_bringup bringup.launch.py ​​-s`)

## 🕹️ Demo

Explore demos showcasing the capabilities of ROSbots:

| 📎 Link                                                                 | 📖 Description                                                                                    |
| ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| [rosbot-telepresence](https://github.com/husarion/rosbot-telepresence) | Stream live video from Orbbec Astra to a PC and control the robot using `teleop-twist-keyboard`  |
| [rosbot-autonomy](https://github.com/husarion/rosbot-autonomy)         | Enables simultaneous mapping and navigation, allowing the robot to move in unknown environments. |
