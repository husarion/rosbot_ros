# Rosbot ROS

ROS 2 packages for Husarion ROSbot series.

![ROSbot](https://husarion.com/assets/images/2r_colour_perspective-14e3679e451eb9fe4e79eeecf7b82e65.png)

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
    git clone https://github.com/husarion/rosbot_ros src/rosbot_ros
    ```

### 🤖 Hardware

#### Building

```bash
export HUSARION_ROS_BUILD_TYPE=hardware

source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos

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

vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos
vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos

# Build only imu_sensor_broadcaster from ros2_controllers
cp -r src/ros2_controllers/imu_sensor_broadcaster src && rm -rf src/ros2_controllers

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

## 🕹️ Demo

Explore demos showcasing the capabilities of ROSbots:

| 📎 Link                                                                 | 📖 Description                                                                                    |
| ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| [rosbot-telepresence](https://github.com/husarion/rosbot-telepresence) | Stream live video from Orbbec Astra to a PC and control the robot using `teleop-twist-keyboard`  |
| [rosbot-autonomy](https://github.com/husarion/rosbot-autonomy)         | Enables simultaneous mapping and navigation, allowing the robot to move in unknown environments. |
