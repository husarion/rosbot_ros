# ROSbot ROS

ROS 2 packages for Husarion ROSbot Series.

<div style="display: flex; justify-content: center; gap: 10px;">
  <img src="https://husarion.com/assets/images/rosbot3-preview2-f7dee8f0b4ea4de02e80d4dc9f2ca286.png" alt="ROSbot" style="width: 45%;"/>
  <img src="https://husarion.com/assets/images/RbXL_5_medium-a3a59b28a740574879d9e80d56c5268f.png" alt="ROSbot XL" style="width: 40%;"/>
</div>

## 📚 ROS API

Documentation is available in ROS_API.md.

## 🚀 Quick Start

### ⚙️ Prerequisites

1. Install all necessary tools:

   ```bash
   sudo apt-get update
   sudo apt-get install -y python3-pip ros-dev-tools
   ```

2. Create a workspace folder and clone the rosbot_ros repository:

   ```bash
   mkdir rosbot_ws
   cd rosbot_ws
   git clone -b jazzy https://github.com/husarion/rosbot_ros.git src/rosbot_ros
   ```

### Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos
vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos

# Optional: speed up build by removing unnecessary packages
# For hardware build only, remove simulation package:
# rm -rf src/rosbot_ros/rosbot_gazebo
# For simulation build only, remove hardware package:
# rm -rf src/rosbot_ros/rosbot_bringup

export PIP_BREAK_SYSTEM_PACKAGES=1
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Run the Robot

For ROSbot XL, you can specify a particular configuration using the launch `configuration` argument. If you are using the `manipulation` configuration, please refer to [MANIPULATOR.md](MANIPULATOR.md) for detailed instructions — including the **gamepad controls** for the OpenMANIPULATOR-X (`X` = joint-jog mode, `Y` = Cartesian XYZ via MoveIt Servo POSE, `RT + Back/Start` = Dock/Home named poses).

**Real robot:**

```bash
source install/setup.bash
ros2 launch rosbot_bringup <rosbot/rosbot_xl>.yaml
```

> [!NOTE]
> The ROSbot ROS Driver is strongly dependent on the firmware version. If you change driver version or ROS distro, ensure the firmware is compatible. Firmware can be updated with the `flash_firmware` script.
>
> ```bash
> source install/setup.bash
> ros2 run rosbot_utils flash_firmware --robot-model <rosbot/rosbot_xl>
> ```
>
> The runtime-switch firmware variant covers both backends; flash it the usual way and pick the link at boot via `backend:=microros` (default `mavlink`).

**Simulation:**

```bash
source install/setup.bash
ros2 launch rosbot_gazebo simulation.yaml robot_model:=<rosbot/rosbot_xl>
```

> [!TIP]
> You can spawn multiple robots in the simulation. To do that, run the launch file multiple times with different namespaces and initial positions (x, y, z). For example:
>
> ```bash
> source install/setup.bash
> ros2 launch rosbot_gazebo spawn_robot.yaml robot_model:=<rosbot/rosbot_xl> namespace:=robot1 x:=0 y:=0
> ```
>
> All topics/services/actions are namespaced under `/<namespace>/` on HW
> and sim — see [Namespace policy](ROS_API.md#namespace-policy).

### Launch Arguments

| Symbol | Meaning                      |
| ------ | ---------------------------- |
| 🤖     | Available for physical robot |
| 🖥️     | Available in simulation      |

| 🤖  | 🖥️  | Argument            | Description <br/> **_Type:_** `Default`                                                                                                                                                            |
| --- | --- | ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ✅  | ✅  | `arm_activate`    | Whether to activate the manipulator arm on startup. <br/> **_bool:_** `False` (`True` for simulation)                                                                          |
| ✅  | ✅  | `config_dir`    | Path to the common configuration directory. You can create such common configuration directory with `ros2 run rosbot_utils create_config_dir {directory}`. <br/> **_string:_** `""`                                                                          |
| ✅  | ✅  | `configuration` | Specify configuration packages. ROSbot XL has the full set; ROSbot 2/3 only `basic`/`custom`. Packages: `basic`, `telepresence`, `autonomy`, `manipulation`, `manipulation_pro`, `custom`. <br/> **_string:_** `basic`                                                                          |
| ✅  | ✅  | `controller_manager_timeout` | Seconds the spawner waits for `controller_manager` to load + activate the controllers (raised from 20 for slower SBCs such as the Raspberry Pi 5 on ROSbot 3). <br/> **_int:_** `60`                                                                          |
| ✅  | ✅  | `joy_vel`       | The topic name to which velocity commands will be published. <br/> **_string:_** `cmd_vel` |
| ✅  | ❌  | `led_strip`       | ROSbot XL only. Enable or disable the LED strip. <br/> **_bool:_** `True` |
| ✅  | ✅  | `mecanum`           | Whether to use mecanum drive controller, otherwise use diff drive. <br/> **_bool:_** `True` for `rosbot_xl`, `False` otherwise                                                                                       |
| ✅  | ✅  | `namespace`         | Add namespace to all launched nodes. <br/> **_string:_** `env(ROBOT_NAMESPACE)`                                                                                                                       |
| ✅  | ✅  | `robot_model`       | Specify robot model. <br/> **_string:_** `env(ROBOT_MODEL)` (choices: `rosbot`, `rosbot_xl`)                                                                                                                       |
| ✅  | ❌  | `backend`           | MCU↔SBC upstream-link backend the hardware bridge drives. `microros` starts the XRCE-DDS agent (`micro_ros_agent`); `mavlink` starts the `rosbot_mavlink_bridge` node. The matching `BACKEND:` line is emitted to the MCU during the pre-comm handshake; runtime-switch firmware brings up the chosen path. <br/> **_string:_** `mavlink` (choices: `microros`, `mavlink`)                                                                                                                          |
| ✅  | ❌  | `hardware_bridge`   | Whether to launch the SBC↔MCU bridge (selected by `backend`). Set to `False` to skip it entirely (e.g. when running the bridge/agent in a separate container). <br/> **_bool:_** `True`                                                                                                                       |
| ✅  | ❌  | `manipulator_serial_port`  | Port to connect to the manipulator. <br/> **_string:_** `/dev/manipulator`                                                                                                                                  |
| ✅  | ❌  | `port`              | **ROSbot XL only.** UDP4 port for micro-ROS agent. <br/> **_string:_** `8888`                                                                                                                         |
| ✅  | ❌  | `serial_baudrate`   | ROSbot only. Baud rate for serial communication. <br/> **_string:_** `921600`                                                                                                                                  |
| ✅  | ❌  | `serial_port`       | ROSbot only. Serial port for micro-ROS agent. <br/> **_string:_** `/dev/ttySERIAL`                                                                                                           |
| ✅  | ✅  | `tf_namespace_bridge` | Bridge robot's namespaced TF to the global /tf and /tf_static. Only active when `namespace` is set. <br/> **_bool:_** `True`                                                              |
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
> To read the arguments for individual launch files, add the `-s` flag to the `ros2 launch` command (e.g. `ros2 launch <pkg> <launch> ​​-s`)

## 🕹️ Demo

Explore demos showcasing the capabilities of ROSbots:

| 📎 Link                                                                | 📖 Description                                                                                   |
| ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| [rosbot-telepresence](https://github.com/husarion/rosbot-telepresence) | Stream live video from Orbbec Astra to a PC and control the robot using `teleop-twist-keyboard`  |
| [rosbot-autonomy](https://github.com/husarion/rosbot-autonomy)         | Enables simultaneous mapping and navigation, allowing the robot to move in unknown environments. |
