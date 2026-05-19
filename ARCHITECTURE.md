# ARCHITECTURE.md — `rosbot_ros` architecture

A description of *how* the repo is wired together: packages, their roles, how they interact, where the integration points are. For public topic / node details see [ROS_API.md](ROS_API.md). For workflow (build, pre-commit, tests) — [CLAUDE.md](CLAUDE.md).

---

## 1. Big picture

```text
                         ┌────────────────────────────────┐
   user input            │    rosbot_bringup / *_gazebo   │   entry point (single launch call)
   ─────────────         │    rosbot.yaml / rosbot_xl.yaml│
   ros2 launch ...       │    simulation.yaml             │
                         └─────────────┬──────────────────┘
                                       │ include
                ┌──────────────────────┼──────────────────────────┐
                ▼                      ▼                          ▼
       ┌────────────────┐    ┌─────────────────┐         ┌──────────────────┐
       │ microros.launch│    │ rosbot_controller│         │  rosbot_joy      │
       │  (HW only)     │    │   controller.yaml│         │   joy.yaml       │
       └───────┬────────┘    └─────────┬───────┘         └──────────────────┘
               │                       │
               │             include   │ include
               │                       ▼
               │            ┌──────────────────────┐
               │            │ rosbot_description    │ — URDF/xacro + robot_state_publisher
               │            │  rosbot.yaml /        │
               │            │  rosbot_xl.yaml       │
               │            └──────┬───────────────┘
               │                   │ <ros2_control> in URDF
               ▼                   ▼
       ┌─────────────────┐   ┌─────────────────────────────┐
       │ micro_ros_agent │   │ controller_manager + drivers │
       │  (serial / udp) │   │  diff/mecanum, imu_broadcaster│
       └────────┬────────┘   │  joint_state_broadcaster      │
                │            │  (manipulator_controller XL)  │
                │            └──────────────┬───────────────┘
                ▼                           │ uses
       ┌──────────────────────┐             ▼
       │ STM32 firmware       │   ┌──────────────────────────┐
       │ (v1.1.0-jazzy)       │   │ rosbot_hardware_interfaces│ (HW only)
       └──────────────────────┘   │  RosbotSystem (motors)    │
                                  │  RosbotImuSensor          │
                                  └──────────────────────────┘

       ┌────────────────────────┐    ┌──────────────────────┐
       │ rosbot_localization    │    │ rosbot_utils          │
       │  ekf_node (odom+imu)   │    │  laser_filter, battery│
       │  → odometry/filtered   │    │  flash_firmware, LED  │
       └────────────────────────┘    └──────────────────────┘

       ┌────────────────────────────────────────┐
       │ rosbot_moveit  (XL manipulation only)  │ — move_group, servo, dock/home
       └────────────────────────────────────────┘
```

Real / simulation split:

- **HW:** `microros.launch.py` starts the agent and (after pre-communication) `controller_manager` with `RosbotSystem` + `RosbotImuSensor` as `<ros2_control>` plugins.
- **Sim:** in `rosbot_gazebo/launch/spawn_robot.yaml` instead of `controller_manager` ros2_control is hosted by the `gz_ros2_control/GazeboSimSystem` plugin inside Gazebo.

---

## 2. Packages

### `rosbot` — meta

[rosbot/](rosbot/) is an `ament_cmake` meta-package with no code. Its role is to gather dependencies and (most importantly) ship the **`rosbot_hardware.repos`** and **`rosbot_simulation.repos`** files used by `vcs import` during workspace setup. Those `.repos` pin commits of external dependencies:

- `husarion_components_description` — components (LDR06, CAM11, mounts) used in `<xacro:components.create_components>`.
- `husarion_controllers` — including `husarion_mecanum_drive_controller`.
- `husarion_gz_worlds` — SDF worlds for Gazebo + `gz_sim.launch.py`.
- `tf_namespace_bridge` — bridge `/<ns>/tf` ↔ `/tf`.
- `micro-ROS-Agent` — hardware repos only.
- (commented out) `dynamixel_hardware_interface`, `open_manipulator` — installed via apt; tested versions are kept here for reference.

### `rosbot_bringup` — HW entry point

Three launches:

- [rosbot.yaml](rosbot_bringup/launch/rosbot.yaml), [rosbot_xl.yaml](rosbot_bringup/launch/rosbot_xl.yaml) — specific model.
- [bringup.yaml](rosbot_bringup/launch/bringup.yaml) — alias dispatching to one of the above based on `robot_model`/`ROBOT_MODEL`.
- [microros.launch.py](rosbot_bringup/launch/microros.launch.py) — starts `micro_ros_agent` in `serial` or `udp4` mode. Beforehand it runs **pre-communication** (`ros2 run rosbot_utils configure_robot`), which: (a) verifies the firmware version (must be `v1.1.0-jazzy`), (b) sets the namespace over serial. If pre-comm exits with rc≠0 — the whole launch issues a `Shutdown`.

XL additionally has: arg `led_strip` (starts `rosbot_utils/led_strip_car_wave`) and the `microros_mode=udp` path (port 8888).

`config_dir` → `<config_dir>/rosbot_bringup/config/microros_localhost_only.xml` (FastDDS limited to 127.0.0.1, used when `ROS_LOCALHOST_ONLY=1`).

Tests: [test_bringup.py](rosbot_bringup/test/test_bringup.py), [test_multirobot.py](rosbot_bringup/test/test_multirobot.py) — they launch the full bringup with `microros:=False` and faked HW publishers (`_imu/data`, `_motors/feedback`), and assert that `joint_states`, `odometry/wheels`, `imu/data`, `odometry/filtered` show up. **Skipped in CI** (local only).

### `rosbot_controller` — ros2_control + manipulator

- [controller.yaml](rosbot_controller/launch/controller.yaml) — the orchestrator: loads URDF, generates a resolved YAML at `/tmp/rosbot_controller_<ns>.yaml` (via `sed`, replacing `<namespace>/` and `<manipulator_state>`), starts `controller_manager` (unless `use_sim`), and after 3 s the controller spawner: `differential_drive_controller` or `mecanum_drive_controller` (depending on `mecanum`), `imu_broadcaster`, `joint_state_broadcaster`. After 5 s — `manipulator.yaml` if `configuration` starts with `manipulation`.
- [manipulator.yaml](rosbot_controller/launch/manipulator.yaml) — spawner for `manipulator_controller` and `gripper_controller` (both `JointTrajectoryController`; gripper drives a single prismatic joint), inclusion of `move_group.launch.py` and `servo.launch.py`, plus include of `rosbot_moveit/launch/home.launch.py` after 10 s (the `home` executable receives the MoveIt config — kinematics/SRDF/joint_limits — via that wrapper so `MoveGroupInterface` doesn't warn about missing kinematics plugins).
- [config/](rosbot_controller/config/) — `controllers.yaml` per model (kinematic parameters, limits, IMU covariances).
- [scripts/arm_control](rosbot_controller/scripts/arm_control) — CLI: `active`/`inactive` toggles `OpenManipulatorXSystem` and the arm controllers.

> **`cmd_vel`/`odom`/`imu` remapping** in the spawner: `~/cmd_vel:=cmd_vel`, `~/odom:=odometry/wheels`, `~/imu:=imu/data`. This is where the public topic names are made canonical.

### `rosbot_description` — URDF, configurations

xacro files in [urdf/](rosbot_description/urdf/):

- `rosbot.urdf.xacro`, `rosbot_xl.urdf.xacro` — top-level, parameterized with `mecanum`, `controller_config`, `namespace`, `use_sim`, `components_config`, (XL) `manipulator_serial_port`, `configuration`.
- `common/ros2_control.urdf.xacro` — `<ros2_control>` definitions for HW and Gazebo (different `<plugin>`).
- `common/gazebo.urdf.xacro` — Gazebo-specific bits.
- `rosbot/`, `rosbot_xl/` — internal body/wheel macros.
- `open_manipulator/` — OpenManipulator-X definitions (used when XL `configuration` starts with `manipulation`).

Configurations (XL): [config/rosbot_xl/](rosbot_description/config/rosbot_xl/) — `basic.yaml` (no components), `telepresence.yaml`, `autonomy.yaml` (LDR06 + CAM11 with mount), `manipulation.yaml`, `manipulation_pro.yaml`, `custom.yaml`. For ROSbot: `basic.yaml` (LDR02 + CAM11), `custom.yaml`. **The component schema** comes from `husarion_components_description`.

[launch/rosbot.yaml](rosbot_description/launch/rosbot.yaml), [launch/rosbot_xl.yaml](rosbot_description/launch/rosbot_xl.yaml) — start `robot_state_publisher` with the URDF generated by `xacro` (output saved to `/tmp/rosbot[_xl]_description_<ns>.urdf`). Optionally `joint_state_publisher` in `mock_joints` mode. [launch/rviz.yaml](rosbot_description/launch/rviz.yaml) starts RViz with the default config.

`hooks/setup_envs.sh.in` appends `share/` to `GZ_SIM_RESOURCE_PATH` and `GAZEBO_MODEL_PATH` — important so that Gazebo can find meshes and the SDF plugin.

Tests: [test_xacro.py](rosbot_description/test/test_xacro.py) — sanity-checks parsing for all `(robot_model, mecanum)` combinations.

### `rosbot_gazebo` — simulation

[simulation.yaml](rosbot_gazebo/launch/simulation.yaml) starts `gz_sim` (via `husarion_gz_worlds/launch/gz_sim.launch.py`), `parameter_bridge` from [config/gz_bridge.yaml](rosbot_gazebo/config/gz_bridge.yaml) (the `/clock` bridge), and delegates to [spawn_robot.yaml](rosbot_gazebo/launch/spawn_robot.yaml).

`spawn_robot.yaml` is the heart of the simulation flow:

1. resolves [rosbot_bridge.yaml](rosbot_gazebo/config/rosbot_bridge.yaml) (sed `<namespace>/`),
2. creates a group with `push_ros_namespace` and `use_sim_time:=True`,
3. starts `tf_namespace_bridge` if namespace ≠ '',
4. runs `parameter_bridge` (cmd_vel bridge) and `ros_gz_sim/create` (spawns the model from the `robot_description` topic),
5. `husarion_components_description/launch/gz_components.launch.py` bridges component sensors (camera, lidar, …),
6. includes `rosbot_controller/controller.yaml` with `use_sim:=True` (i.e. **without** `controller_manager` — the Gazebo plugin takes that role),
7. `rosbot_joy`, `rosbot_localization`, `rosbot_utils/laser_filter`, optionally RViz.

Tests: [test_sim.py](rosbot_gazebo/test/test_sim.py), [test_multirobot.py](rosbot_gazebo/test/test_multirobot.py) — `gz_headless_mode:=True`, parameterized over `(mecanum, namespace, robot_model)`. **Skipped in CI** (Gazebo is not container-CI friendly).

### `rosbot_hardware_interfaces` — ros2_control plugins

C++, `ament_cmake`. Exports two classes via `pluginlib`:

- `rosbot_hardware_interfaces::RosbotSystem` (`hardware_interface::SystemInterface`) — accepts `velocity` on the 4 wheel joints, provides `position`/`velocity`. Maps to the MCU via `motors_cmd` Float32MultiArray (order from the `velocity_command_joint_order` parameter).
- `rosbot_hardware_interfaces::RosbotImuSensor` (`hardware_interface::SensorInterface`) — subscribes to `_imu/data` from the MCU, exposes 10 state interfaces.

Plugin parameters: `connection_timeout_ms`, `connection_check_period_ms`. Defined in [common/ros2_control.urdf.xacro](rosbot_description/urdf/common/ros2_control.urdf.xacro).

> **This is the firmware ABI.** Changing a topic / payload requires a synchronized firmware update.

### `rosbot_joy` — joystick (drive)

- **Config-only package** (no compiled code). Pad-driven arm control (`joy2servo`) lives in `rosbot_moveit` now.
- Launch [joy.yaml](rosbot_joy/launch/joy.yaml) starts the standard `joy/joy_node` + `teleop_twist_joy/teleop_node` (mapped to `cmd_vel` via the `joy_vel` arg).
- [config/config.yaml](rosbot_joy/config/config.yaml) — `joy_node` + `teleop_twist_joy` params (drive only). Pad layout: see `.docs/gamepad_*.drawio.png`.

### `rosbot_localization` — EKF

[ekf.yaml](rosbot_localization/launch/ekf.yaml) starts `robot_localization/ekf_node` with [config/config.yaml](rosbot_localization/config/config.yaml). Fuses `odometry/wheels` (vx, vy, vyaw) + `imu/data` (yaw, dyaw) → `odometry/filtered` at 25 Hz, `two_d_mode: true`. EKF publishes the `odom→base_link` TF (the drive controllers have `enable_odom_tf: false`).

The covariance matrices are **tuned empirically** — comments such as "values measured experimentally" / "selected values experimentally" in `controllers.yaml` and `config.yaml` are not placeholders.

### `rosbot_moveit` — manipulation (XL)

- MoveIt config for `rosbot_xl` with OpenMANIPULATOR-X (SRDF, kinematics, OMPL, Pilz, joint_limits, moveit_servo, moveit_controllers, initial_positions). 4-DoF arm → `kinematics.yaml` keeps `position_only_ik: true`; without it KDL cannot satisfy orientation goals.
- [launch/move_group.launch.py](rosbot_moveit/launch/move_group.launch.py) — builds the config via `MoveItConfigsBuilder`, **overrides `robot_description`** using the same xacro as bringup with `configuration:='manipulation'` (so the URDF stays consistent with the rest of the stack).
- [launch/servo.launch.py](rosbot_moveit/launch/servo.launch.py) — `moveit_servo/servo_node` + `joy2servo` (joint-jog from the pad — TWIST mode is unusable on <6 DoF arms since MoveIt jazzy, see [moveit_msgs#185](https://github.com/moveit/moveit_msgs/issues/185)). `moveit_servo.yaml` overrides `monitored_planning_scene_topic` to the relative `planning_scene` — the upstream default `/planning_scene` is absolute and would strip the namespace from the PSM's private sub-node (`servo_node_private_*`).
- [launch/rviz.launch.py](rosbot_moveit/launch/rviz.launch.py) — RViz with MotionPlanning + servo.
- C++ artifacts:
  - [include/rosbot_moveit/arm_pose_mover.hpp](rosbot_moveit/include/rosbot_moveit/arm_pose_mover.hpp) + [src/arm_pose_mover.cpp](rosbot_moveit/src/arm_pose_mover.cpp) — shared library wrapping `MoveGroupInterface` (executor lifetime, retry loop, namespaced `Options`). Used by both [dock.cpp](rosbot_moveit/src/dock.cpp) (`Close` gripper → arm to `Dock`) and [home.cpp](rosbot_moveit/src/home.cpp) (arm to `Home` → `Open` gripper).
  - [src/joy2servo.cpp](rosbot_moveit/src/joy2servo.cpp) — formerly `rosbot_joy/joy2servo`; moved here in 2026-05-15 because it is purely a MoveIt Servo client. Two runtime modes (toggle: `X` / `Y` buttons): **JOINT_JOG** (per-joint, no IK) and **POSE** (Cartesian XYZ — integrates joystick velocity into an absolute `PoseStamped` target on `servo_node/pose_target_cmds`; KDL with `position_only_ik: true` ignores orientation, which is the only working Cartesian path on the 4-DoF arm in MoveIt jazzy). TWIST mode is removed because it hits the pseudo-inverse Jacobian singularity check below 6 DoF.
  - All three pass `node->get_namespace()` as the 3rd `MoveGroupInterface::Options` argument so MoveIt's internal topics (`trajectory_execution_event`, `attached_collision_object`) and the `move_group` action stay inside the robot namespace.
- OMPL planner list trimmed to 3 (`RRTConnect` default + `RRTstar` + `PRMstar`); the gripper group keeps a single `RRTConnect` config because `setNamedTarget("Open"/"Close") + move()` from `dock`/`home` still runs through OMPL before MoveIt forwards the trajectory to the JTC via `FollowJointTrajectory`. Continuous joystick gripper control goes around MoveIt entirely — `joy2servo` publishes `trajectory_msgs/JointTrajectory` directly on `gripper_controller/joint_trajectory`.

See [MANIPULATOR.md](MANIPULATOR.md) — limits, troubleshooting, safety rules.

### `rosbot_utils` — utilities

The broadest package, a Python + ament_cmake mix:

- **Scripts** (installed under `lib/rosbot_utils`):
  - `flash_firmware` — flashes the STM32 with binaries from [firmware/](rosbot_utils/firmware/) (built-in `rosbot-v1.1.0-jazzy.bin`, `rosbot_xl-v1.1.0-jazzy.bin`).
  - `configure_robot` — pre-communication: verifies the firmware version, sets the namespace over serial. Called from `microros.launch.py`.
  - `create_config_dir <dst>` — copies `share/<pkg>/config` from each rosbot_* package into `<dst>/<pkg>/config`. Used to build a "user" config dir for the Husarion snap.
  - `install_udev_rules` — installs [udev/99-rosbot.rules](rosbot_utils/udev/99-rosbot.rules) (FTDI 0403:6015 → `/dev/rosbot`, 0403:6014 → `/dev/manipulator`).
  - `battery_alert` — a Python node (with [src/battery_alert_parameters.yaml](rosbot_utils/src/battery_alert_parameters.yaml) generated by `generate_parameter_library`), plays a sound below `<percentage_threshold`.
  - `led_strip_car_wave`, `led_strip_rainbow` — LED effects (XL).
- **Python modules** (`rosbot_utils/`): `mcu_manager_ftdi.py`, `mcu_manager_uart.py`, `utils.py` (find_device_port).
- **Launches:** `laser_filter.yaml` (crops scan inside the robot's bbox), `battery_alert.yaml`.
- **Per-model configs:** [config/rosbot/config.yaml](rosbot_utils/config/rosbot/config.yaml), [config/rosbot_xl/config.yaml](rosbot_utils/config/rosbot_xl/config.yaml) — battery_alert + laser_filter.

---

## 3. Launch flows

### 3.1 Real ROSbot (XL)

1. `ros2 launch rosbot_bringup rosbot_xl.yaml`.
2. `microros.launch.py`:
   - `pre_communication` (`configure_robot`) is run as `ExecuteProcess`. It opens the serial port, reads `FW: v1.1.0-jazzy`, sends `NS:<namespace>\n`, waits for `ACK`. Exit ≠ 0 → `Shutdown`.
   - on pre-comm success: `OnProcessExit` adds `micro_ros_agent` (`udp4 --port 8888` for XL).
3. `rosbot_controller/controller.yaml`:
   - sed-resolves `controllers.yaml` into `/tmp/rosbot_controller_<ns>.yaml` (`<namespace>/`, `<manipulator_state>`).
   - `rosbot_description/rosbot_xl.yaml` → `robot_state_publisher` with the resolved URDF (with `controller_config:=<resolved>`), `use_sim=False`, `<ros2_control>` uses `RosbotSystem`/`RosbotImuSensor`.
   - after 3 s: `controller_manager` (`ros2_control_node`) + the controller spawner.
   - if `configuration ∈ {manipulation, manipulation_pro}` — after 5 s `manipulator.yaml` (controllers + move_group + servo + home).
4. `rosbot_joy/joy.yaml` — joystick.
5. `rosbot_localization/ekf.yaml` — EKF.
6. `rosbot_utils/laser_filter.yaml` — scan filter.
7. (XL) `led_strip_car_wave` if `led_strip:=True`.

### 3.2 Simulation (XL)

1. `ros2 launch rosbot_gazebo simulation.yaml robot_model:=rosbot_xl`.
2. `gz_sim` with `husarion_gz_worlds/husarion_world.sdf`.
3. `parameter_bridge` bridges `/clock` (Gazebo → ROS).
4. `spawn_robot.yaml` inside a group with `push_ros_namespace`:
   - resolve `rosbot_bridge.yaml` → `cmd_vel` bridge (Twist ↔ TwistStamped),
   - `ros_gz_sim/create -topic robot_description` — spawn the model in the world,
   - `gz_components.launch.py` — sensor bridges,
   - `rosbot_controller/controller.yaml use_sim:=True` (controller hosted by the GazeboSimSystem plugin),
   - `rosbot_joy`, `rosbot_localization`, `laser_filter`, RViz.

### 3.3 Manipulator (HW XL with `configuration:=manipulation`)

- the arm is idle on startup unless `arm_activate:=True`.
- runtime activation: `ros2 run rosbot_controller arm_control active`.
- control: pad (mapping in `rosbot_joy/config/config.yaml`) **or** RViz MotionPlanning. Do not use both at the same time.
- safe shutdown: `arm_control inactive` (warning: the arm will fall under gravity — hold it).

---

## 4. External configuration (`config_dir`)

Every launch accepts `config_dir` (default `''`). When non-empty, config paths flip from `<pkg-share>/config/...` to `<config_dir>/<pkg>/config/...`. This lets the Husarion snap ship with native config editing **without** rebuilding the package.

Creating one:

```bash
ros2 run rosbot_utils create_config_dir ~/my_rosbot_config
```

Subdirectories created under `~/my_rosbot_config/`: `rosbot_bringup/config/`, `rosbot_controller/config/`, `rosbot_description/config/`, `rosbot_joy/config/`, `rosbot_localization/config/`, `rosbot_moveit/config/`, `rosbot_utils/config/` + `firmware/`.

---

## 5. Namespacing and multirobot

- `ROBOT_NAMESPACE` env var → `namespace` arg.
- Inside a launch: `push_ros_namespace` shifts every node under the `/<ns>/` prefix.
- TF is namespaced (`set_remap /tf → tf` and `/tf_static → tf_static` in every launch).
- Global `/tf` (needed for nav2 / a global RViz view) is bridged by [tf_namespace_bridge](https://github.com/husarion/tf_namespace_bridge) — it starts when `tf_namespace_bridge:=True` and `namespace ≠ ''`. Which frames are bridged is controlled by the `frame_filters` parameter (glob patterns against `child_frame_id`); the default empty list = pass-through. Edit [rosbot_bringup/config/tf_namespace_bridge.yaml](rosbot_bringup/config/tf_namespace_bridge.yaml) for HW or [rosbot_gazebo/config/tf_namespace_bridge.yaml](rosbot_gazebo/config/tf_namespace_bridge.yaml) for sim.
- Multirobot in sim: run `spawn_robot.yaml` for each robot with a different `namespace` and `(x,y,z)`. Example: see the `test_multirobot.py` tests in `rosbot_bringup` and `rosbot_gazebo`.

In the IMU controllers `sensor_name: <namespace>/imu` (placeholder sed-replaced) — Gazebo requires unique sensor names, otherwise the `gz_ros2_control` plugin overwrites readings.

---

## 6. Public topics and nodes

For the official list of names — see **[ROS_API.md](ROS_API.md)**. The most important ones:

| Direction | Topic | Comment |
|---|---|---|
| sub | `cmd_vel` (TwistStamped) | drive controller; XL = mecanum, ROSbot = diff |
| pub | `odometry/wheels` | drive controller, covariances tuned empirically |
| pub | `imu/data` | imu_broadcaster (on HW: rosbot_imu_sensor → broadcaster) |
| pub | `odometry/filtered` | EKF, fusion of the above |
| pub | `tf`, `tf_static` | rsp + drive (with `enable_odom_tf:false`, so mostly EKF) |
| pub | `joint_states` | joint_state_broadcaster |
| pub | `scan`, `scan_filtered` | raw lidar / after the bbox filter |
| pub/sub | `joy` | manual control |
| pub | `dynamic_joint_states`, `diagnostics`, `robot_description` | ros2_control standard |

All topics are namespaced when `ROBOT_NAMESPACE` is set.

---

## 7. Firmware and pre-communication

- Binaries: `rosbot_utils/firmware/{rosbot,rosbot_xl}-v1.1.0-jazzy.bin`. The version pin is hardcoded in [scripts/configure_robot](rosbot_utils/scripts/configure_robot) (`expected_fw = "v1.1.0-jazzy"`) and [scripts/flash_firmware](rosbot_utils/scripts/flash_firmware).
- Flash: `ros2 run rosbot_utils flash_firmware --robot-model rosbot[_xl]`. For XL the `--usb` flag is set automatically (FTDI), for ROSbot it defaults to UART over GPIO (resets via gpiochip).
- Pre-communication serial protocol:
  - boot: the MCU emits `FW: v1.1.0-jazzy\n`,
  - host: `NS:<namespace>\n`,
  - MCU: `ACK\n` on success.
- USB-B development (on a PC, no Raspberry): see [CONTRIBUTING.md](CONTRIBUTING.md). The connection requires holding `btn1`/`btn2` + reset by hand.

---

## 8. Where to find what — quick map

| Looking for... | Go to... |
|---|---|
| public topic / node names | [ROS_API.md](ROS_API.md) |
| launch arguments | [README.md](README.md) (table), `ros2 launch <p> <f> -s` |
| how to build / commit | [CLAUDE.md](CLAUDE.md), [CONTRIBUTING.md](CONTRIBUTING.md) |
| how the XL arm behaves / restart | [MANIPULATOR.md](MANIPULATOR.md) |
| drive parameters (speed limits, covariances) | `rosbot_controller/config/<model>/controllers.yaml` |
| EKF parameters | `rosbot_localization/config/config.yaml` |
| an example YAML launch frontend | any `*.yaml` file under `*/launch/` |
| URDF for editing geometry | `rosbot_description/urdf/<model>/body.urdf.xacro` |
| MoveIt SRDF / IK | `rosbot_moveit/config/rosbot_xl.srdf`, `kinematics.yaml` |
| hardware plugins | `rosbot_hardware_interfaces/src/*.cpp`, `rosbot_hardware_interfaces.xml` |
| firmware and protocol | `rosbot_utils/firmware/`, `rosbot_utils/scripts/configure_robot` |
| Docker images | `docker/Dockerfile.{hardware,simulation}`, `docker/compose.*.yaml` |
| CI | `.github/workflows/{integration,run-tests,build-docker,backport}.yaml` |

---

## 9. Boundaries — what lives *outside* the repo

- `husarion_components_description` — component schema, per-type xacro macros (LDR06, CAM11, …). Changing them = URDF rebuild.
- `husarion_controllers/husarion_mecanum_drive_controller` — the mecanum controller.
- `husarion_gz_worlds` — SDF world file plus plugins. `husarion_world.sdf` is the default here.
- `tf_namespace_bridge` — the TF bridge code.
- `rosbot-firmware` (<https://github.com/husarion/rosbot-firmware>) — STM32 firmware code. The MCU topics seen by the micro-ROS agent are documented in its own ROS_API.md.

All pinned versions live in `rosbot/rosbot_*.repos`. Bumping any of them = check that `test_xacro` still passes and that `controllers.yaml` parameters do not collide.
