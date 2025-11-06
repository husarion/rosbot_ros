# ROSbot ROS

ROS 2 packages for Husarion ROSbot Series.

<div style="display: flex; justify-content: center; gap: 10px;">
  <img src="https://husarion.com/assets/images/2r_colour_perspective-bdf84fd9c22667bb07ddfddd9741d0df.png" alt="ROSbot" style="width: 45%;"/>
  <img src="https://husarion.com/assets/images/RbXL_5_medium-a3a59b28a740574879d9e80d56c5268f.png" alt="ROSbot XL" style="width: 40%;"/>
</div>

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
   mkdir rosbot_ws
   cd rosbot_ws
   git clone -b humble https://github.com/husarion/rosbot_ros.git src/rosbot_ros
   ```

### Configure environment

The repository is used to run the code both on the real robot and in the simulation. Specify `HUSARION_ROS_BUILD_TYPE` the variable according to your needs.

Real robot:

```bash
export HUSARION_ROS_BUILD_TYPE=hardware
```

Simulation:

```bash
export HUSARION_ROS_BUILD_TYPE=simulation
```

### Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_ros/rosbot/rosbot_${HUSARION_ROS_BUILD_TYPE}.repos
vcs import src < src/rosbot_ros/rosbot/manipulator.repos # For ROSbot XL manipulation package

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Run the Robot

For ROSbot XL, you can specify a particular configuration using the launch `configuration` argument. If you are using the `manipulation` configuration, please refer to [MANIPULATOR.md](MANIPULATOR.md) for detailed instructions.

**Real robot:**

```bash
source install/setup.bash
ros2 launch rosbot_bringup bringup.launch.py robot_model:=<rosbot/rosbot_xl>
```

> [!NOTE]
> To run the software on real ROSbots, communication with the CORE2 is required. Ensure the firmware is updated before running the micro-ROS agent. For detailed instructions, refer to the rosbot_ros2_firmware repository.
>
> ```bash
> sudo su
> source install/setup.bash
> ros2 run rosbot_utils flash_firmware --robot-model <rosbot/rosbot_xl>
> exit
> ```

**Simulation:**

```bash
source install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=<rosbot/rosbot_xl>
```

### Launch Arguments

| Symbol | Meaning                      |
| ------ | ---------------------------- |
| 🤖     | Available for physical robot |
| 🖥️     | Available in simulation      |

| 🤖  | 🖥️  | Argument            | Description <br/> **_Type:_** `Default`                                                                                                                                                            |
| --- | --- | ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ✅  | ✅  | `config_dir`    | Path to the common configuration directory. You can create such common configuration directory with `ros2 run rosbot_utils create_config_dir {directory}`. <br/> **_string:_** `""`                                                                          |
| ✅  | ✅  | `configuration` | Specify configuration packages. Currently only ROSbot XL has available packages. Packages: `basic`, `telepresence`, `autonomy`, `manipulation`, `manipulation_pro`, `custom`. <br/> **_string:_** 'basic'                                                                          |
| ✅  | ✅  | `joy_vel`       | The topic name to which velocity commands will be published. <br/> **_string:_** `cmd_vel` |
| ✅  | ✅  | `mecanum`           | Whether to use mecanum drive controller, otherwise use diff drive. <br/> **_bool:_** `False`                                                                                       |
| ✅  | ✅  | `namespace`         | Add namespace to all launched nodes. <br/> **_string:_** `env(ROBOT_NAMESPACE)`                                                                                                                       |
| ✅  | ✅  | `robot_model`       | Specify robot model. <br/> **_string:_** `env(ROBOT_MODEL_NAME)` (choices: `rosbot`, `rosbot_xl`)                                                                                                                       |
| ✅  | ❌  | `manipulator_serial_port`  | Port to connect to the manipulator. <br/> **_string:_** `8888`                                                                                                                                  |
| ✅  | ❌  | `microros`          | Automatically connect with hardware using microros. <br/> **_bool:_** `True`                                                                                                                       |
| ✅  | ❌  | `port`              | **ROSbot XL only.** UDP4 port for micro-ROS agent. <br/> **_string:_** `8888`                                                                                                                         |
| ✅  | ❌  | `serial_baudrate`   | ROSbot only. Baud rate for serial communication. <br/> **_string:_** `576000`                                                                                                                                  |
| ✅  | ❌  | `serial_port`       | ROSbot only. Serial port for micro-ROS agent. <br/> **_string:_** `/dev/ttySERIAL`                                                                                                           |
| ❌  | ✅  | `gz_gui`            | Run simulation with specific GUI layout. <br/> **_string:_** [`teleop.config`](https://github.com/husarion/husarion_gz_worlds/blob/main/config/teleop.config)                                      |
| ❌  | ✅  | `gz_headless_mode`  | Run the simulation in headless mode. Useful when a GUI is not needed or to reduce the number of calculations. <br/> **_bool:_** `False`                                                            |
| ❌  | ✅  | `gz_log_level`      | Adjust the level of console output. <br/> **_int:_** `1` (choices: `0`, `1`, `2`, `3`, `4`)                                                                                                        |
| ❌  | ✅  | `gz_world`          | Absolute path to SDF world file. <br/> **_string:_** [`husarion_world.sdf`](https://github.com/husarion/husarion_gz_worlds/blob/main/worlds/husarion_world.sdf)                                    |
| ❌  | ✅  | `rviz`          | Run RViz simultaneously. <br/> **_bool:_** `True`                                    |
| ❌  | ✅  | `x`                 | Initial robot position in the global 'x' axis. <br/> **_float:_** `0.0`                                                                                                                            |
| ❌  | ✅  | `y`                 | Initial robot position in the global 'y' axis. <br/> **_float:_** `2.0`                                                                                                                            |
| ❌  | ✅  | `z`                 | Initial robot position in the global 'z' axis. <br/> **_float:_** `0.0`                                                                                                                            |
| ❌  | ✅  | `roll`              | Initial robot 'roll' orientation. <br/> **_float:_** `0.0`                                                                                                                                         |
| ❌  | ✅  | `pitch`             | Initial robot 'pitch' orientation. <br/> **_float:_** `0.0`                                                                                                                                        |
| ❌  | ✅  | `yaw`               | Initial robot 'yaw' orientation. <br/> **_float:_** `0.0`                                                                                                                                          |

> [!TIP]
> To read the arguments for individual launch files, add the `-s` flag to the `ros2 launch` command (e.g. `ros2 launch rosbot_bringup bringup.launch.py ​​-s`)

## 🕹️ Demo

Explore demos showcasing the capabilities of ROSbots:

| 📎 Link                                                                | 📖 Description                                                                                   |
| ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| [rosbot-telepresence](https://github.com/husarion/rosbot-telepresence) | Stream live video from Orbbec Astra to a PC and control the robot using `teleop-twist-keyboard`  |
| [rosbot-autonomy](https://github.com/husarion/rosbot-autonomy)         | Enables simultaneous mapping and navigation, allowing the robot to move in unknown environments. |
