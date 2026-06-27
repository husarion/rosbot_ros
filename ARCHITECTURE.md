# ARCHITECTURE.md — `rosbot_ros`

How the repo is wired: packages, roles, integration points. Public topics → [ROS_API.md](ROS_API.md). Build/test workflow → [CLAUDE.md](CLAUDE.md).

---

## 1. Big picture

```text
                         ┌────────────────────────────────┐
   ros2 launch ...       │    rosbot_bringup / *_gazebo   │   entry point (single launch call)
                         │    rosbot.yaml / rosbot_xl.yaml│
                         │    simulation.yaml             │
                         └─────────────┬──────────────────┘
                ┌──────────────────────┼──────────────────────────┐
                ▼                      ▼                          ▼
       ┌────────────────┐    ┌─────────────────┐         ┌──────────────────┐
       │ microros.launch│    │ rosbot_controller│         │  rosbot_joy      │
       │  (HW only)     │    │   controller.yaml│         │   joy.yaml       │
       └───────┬────────┘    └─────────┬───────┘         └──────────────────┘
               │                       │ include
               │                       ▼
               │            ┌──────────────────────┐
               │            │ rosbot_description    │ — URDF/xacro + robot_state_publisher
               │            └──────┬───────────────┘
               ▼                   ▼ <ros2_control> in URDF
       ┌──────────────────────┐   ┌─────────────────────────────┐
       │ backend (backend=…)  │   │ controller_manager + drivers │
       │  micro_ros_agent OR  │   │  diff/mecanum, imu_broadcaster│
       │  rosbot_mavlink_bridge│  │  joint_state_broadcaster      │
       └─────────┬────────────┘   │  (manipulator_controller XL)  │
                 │                └──────────────┬───────────────┘
                 ▼                               ▼
       ┌──────────────────────────────┐    ┌──────────────────────────┐
       │ STM32 firmware (runtime-     │    │ rosbot_hardware_interfaces│ (HW only)
       │  switch — ${FIRMWARE_VERSION})│   │  RosbotSystem, RosbotImuSensor │
       │  micro-ROS + MAVLink in one  │    └──────────────────────────┘
       └──────────────────────────────┘

       ┌────────────────────────┐    ┌──────────────────────┐
       │ rosbot_localization    │    │ rosbot_utils          │
       │  ekf → odom/filtered   │    │  battery_alert        │
       └────────────────────────┘    │  flash_firmware, LED  │
                                     └──────────────────────┘

       ┌────────────────────────────────────────┐
       │ rosbot_moveit  (XL manipulation only)  │ — move_group, servo, dock/home
       └────────────────────────────────────────┘
```

**HW vs sim split:** HW → the `backend` arg picks `microros.launch.py` (micro-ROS XRCE-DDS agent) or `mavlink.launch.py` (`rosbot_mavlink_bridge`), then `controller_manager` runs with `RosbotSystem`/`RosbotImuSensor` plugins regardless of backend. Sim → `rosbot_gazebo/spawn_robot.yaml` skips `controller_manager` (hosted by `gz_ros2_control/GazeboSimSystem` plugin inside Gazebo).

---

## 2. Packages

### `rosbot` — meta

`ament_cmake`, no code. Ships [rosbot/rosbot_hardware.repos](rosbot/rosbot_hardware.repos) + [rosbot/rosbot_simulation.repos](rosbot/rosbot_simulation.repos) — pin commits of `husarion_components_description`, `husarion_controllers`, `husarion_gz_worlds`, `tf_namespace_bridge`, `micro-ROS-Agent` (HW only). `dynamixel_hardware_interface` / `open_manipulator` commented out (apt-installed; pinned for reference).

### `rosbot_bringup` — HW entry point

- [rosbot.yaml](rosbot_bringup/launch/rosbot.yaml) / [rosbot_xl.yaml](rosbot_bringup/launch/rosbot_xl.yaml) per model; [bringup.yaml](rosbot_bringup/launch/bringup.yaml) dispatches by `robot_model`.
- [microros.launch.py](rosbot_bringup/launch/microros.launch.py) — selected by `backend:=microros`. Runs `configure_robot --backend microros` (firmware version check against `FIRMWARE_VERSION` + `BACKEND:` and `NS:<ns>` handshake; rc≠0 → `Shutdown`), then `micro_ros_agent` (serial 921600 / udp4:8888 for XL).
- [mavlink.launch.py](rosbot_bringup/launch/mavlink.launch.py) — selected by `backend:=mavlink` (default). Same pre-comm with `--backend mavlink`, then `rosbot_mavlink_bridge` instead of the XRCE-DDS agent. Requires the runtime-switch firmware on the MCU (one binary that brings up either backend based on the boot `BACKEND:` line) and the `rosbot_mavlink_bridge` package on the ROS overlay.
- XL extras: `led_strip` arg, `microros_mode=udp`. `config_dir` → `microros_localhost_only.xml` when `ROS_LOCALHOST_ONLY=1`.
- Tests: [test_bringup.py](rosbot_bringup/test/test_bringup.py), [test_launch_offline.py](rosbot_bringup/test/test_launch_offline.py), [test_namespace_isolation.py](rosbot_bringup/test/test_namespace_isolation.py) (shared helpers in [bringup_helpers.py](rosbot_bringup/test/bringup_helpers.py)). Driven with `hardware_bridge:=False` + fake HW topics, so they run in CI.

### `rosbot_controller` — ros2_control + manipulator

- [controller.yaml](rosbot_controller/launch/controller.yaml) — sed-resolves `controllers.yaml` → `/tmp/rosbot_controller_<ns>.yaml` (substitutes `<namespace>/`, `<manipulator_state>`), starts `controller_manager` (HW only), spawns `{differential,mecanum}_drive_controller` + `imu_broadcaster` + `joint_state_broadcaster` after 3 s. If `configuration` starts with `manipulation` → `manipulator.yaml` after 5 s.
- [manipulator.yaml](rosbot_controller/launch/manipulator.yaml) — `manipulator_controller` + `gripper_controller` (both JTC), `move_group.launch.py`, `servo.launch.py`, `home.launch.py` (after 10 s, with MoveIt config injection).
- Spawner remaps drive controller's `~/cmd_vel:=cmd_vel`, `~/odom:=odometry/wheels`, `~/imu:=imu/data` — canonical public names.
- `scripts/arm_control active|inactive` — toggles `OpenManipulatorXSystem` + arm controllers.

### `rosbot_description` — URDF, configurations

- xacro in [urdf/](rosbot_description/urdf/): `rosbot[_xl].urdf.xacro` (top-level, args: `mecanum`, `controller_config`, `namespace`, `use_sim`, `components_config`, XL+`manipulator_serial_port`, `configuration`); `common/ros2_control.urdf.xacro` (HW vs Gazebo `<plugin>`); `common/gazebo.urdf.xacro` (sim-specific + namespace remap block for CM services); `open_manipulator/` for XL manipulation.
- XL configurations in [config/rosbot_xl/](rosbot_description/config/rosbot_xl/): `basic`, `telepresence`, `autonomy` (LDR06+CAM11), `manipulation`, `manipulation_pro`, `custom`. Component schema from `husarion_components_description`.
- [launch/rosbot[_xl].yaml](rosbot_description/launch/) starts `robot_state_publisher` (URDF → `/tmp/rosbot[_xl]_description_<ns>.urdf`); optional `joint_state_publisher` mock_joints. [launch/rviz.yaml](rosbot_description/launch/rviz.yaml).
- `hooks/setup_envs.sh.in` appends `share/` to `GZ_SIM_RESOURCE_PATH` + `GAZEBO_MODEL_PATH`.
- Tests: [test_xacro.py](rosbot_description/test/test_xacro.py).

### `rosbot_gazebo` — simulation

- [simulation.yaml](rosbot_gazebo/launch/simulation.yaml) → `gz_sim` (via `husarion_gz_worlds`) + `parameter_bridge` (`/clock`) + delegates to [spawn_robot.yaml](rosbot_gazebo/launch/spawn_robot.yaml).
- `spawn_robot.yaml`: sed-resolves [rosbot_bridge.yaml](rosbot_gazebo/config/rosbot_bridge.yaml) → `push_ros_namespace` group + `use_sim_time:=True` → `tf_namespace_bridge` (if ns≠'') → `cmd_vel` bridge + `ros_gz_sim/create` → `husarion_components_description/gz_components.launch.py` (sensor bridges) → `rosbot_controller/controller.yaml use_sim:=True` (no CM — Gazebo plugin) → `rosbot_joy` / `rosbot_localization` / optional RViz.
- Tests: [test_launch_offline.py](rosbot_gazebo/test/test_launch_offline.py) — offline launch/schema checks. Real-Gazebo tests were removed; this runs in CI.

### `rosbot_hardware_interfaces` — ros2_control plugins (C++)

`pluginlib` exports:

- `RosbotSystem` (`SystemInterface`) — 4 wheel joints, `velocity` cmd → MCU via `motors_cmd` `Float32MultiArray` (order from `velocity_command_joint_order` param). State interfaces per joint: `position`, `velocity`, and optional `effort` (motor torque from `_motors/feedback`; declared in the HW URDF, absent in sim → effort published as `NaN` there).
- `RosbotImuSensor` (`SensorInterface`) — subscribes `_imu/data` from MCU, exposes 10 state interfaces.
- Params: `connection_timeout_ms`, `connection_check_period_ms`. Defined in [common/ros2_control.urdf.xacro](rosbot_description/urdf/common/ros2_control.urdf.xacro).

**This is the firmware ABI** — topic / payload changes require synchronized firmware update.

### `rosbot_joy` — joystick (drive only)

Config-only. `joy.yaml` starts standard `joy/joy_node` + `teleop_twist_joy/teleop_node` (mapped to `cmd_vel`). Arm control (`joy2servo`) moved to `rosbot_moveit` in 2026-05-15. Pad layout: `.docs/gamepad_*.drawio.png`.

### `rosbot_localization` — EKF

[ekf.yaml](rosbot_localization/launch/ekf.yaml) → `robot_localization/ekf_node` fusing `odometry/wheels` (vx, vy, vyaw) + `imu/data` (yaw, dyaw) → `odometry/filtered` @ 25 Hz, `two_d_mode: true`. EKF publishes `odom→base_link` TF (drive controllers have `enable_odom_tf: false`). Covariances **empirically tuned** — "values measured experimentally" comments are NOT placeholders.

### `rosbot_moveit` — manipulation (XL only)

- MoveIt config for `rosbot_xl` + OpenMANIPULATOR-X (SRDF, kinematics, OMPL, Pilz, joint_limits, moveit_servo, moveit_controllers, initial_positions). 4-DoF arm → `kinematics.yaml` keeps `position_only_ik: true` (KDL can't satisfy orientation otherwise).
- [move_group.launch.py](rosbot_moveit/launch/move_group.launch.py) — `MoveItConfigsBuilder`, overrides `robot_description` with same xacro as bringup (`configuration:='manipulation'`).
- [servo.launch.py](rosbot_moveit/launch/servo.launch.py) — `moveit_servo/servo_node` + `joy2servo`. `moveit_servo.yaml` uses relative `monitored_planning_scene_topic` (upstream `/planning_scene` strips namespace).
- [rviz.launch.py](rosbot_moveit/launch/rviz.launch.py) — RViz + MotionPlanning + servo; sed-resolved `/tmp/rosbot_moveit_<ns>.rviz`.
- C++: [arm_pose_mover](rosbot_moveit/src/arm_pose_mover.cpp) shared lib (RAII executor + retry loop + namespaced MGI Options) used by [dock](rosbot_moveit/src/dock.cpp) + [home](rosbot_moveit/src/home.cpp). [joy2servo](rosbot_moveit/src/joy2servo.cpp): two modes (`X`=JOINT_JOG, `Y`=Cartesian XYZ via local KDL IK with `position_only_ik` → JointJog to skip servo's singularity guard). TWIST removed (pseudo-inverse Jacobian singularity check fails <6 DoF — see [moveit_msgs#185](https://github.com/moveit/moveit_msgs/issues/185)). All three pass `node->get_namespace()` as 3rd `MGI::Options` arg.
- OMPL trimmed to 3 planners (`RRTConnect`, `RRTstar`, `PRMstar`); gripper group keeps single `RRTConnect` (named-target moves still go through OMPL). Joystick gripper bypasses MoveIt — `joy2servo` publishes `JointTrajectory` directly on `gripper_controller/joint_trajectory`.
- See [MANIPULATOR.md](MANIPULATOR.md) for limits, troubleshooting, safety.

### `rosbot_utils` — utilities

- Scripts (in `lib/rosbot_utils`): `flash_firmware` (flashes `rosbot[_xl]-${FIRMWARE_VERSION}.bin` from [firmware/](rosbot_utils/firmware/) — single runtime-switch binary covers both backends), `configure_robot` (pre-comm: FW string check + `BACKEND:` + `NS:` handshake; `--backend microros|mavlink` selects upstream link), `create_config_dir <dst>` (snap config), `install_udev_rules` (FTDI 0403:6015 → `/dev/rosbot`, 0403:6014 → `/dev/manipulator`), `battery_alert` (Python node with `generate_parameter_library` schema), `led_strip_car_wave`, `led_strip_rainbow` (both node-named `led_strip_manager`, publish `led_strip`; a `led_strip/enable` `SetBool` service stops/resumes computing + publishing the image at runtime).
- Python modules: `mcu_manager_ftdi.py`, `mcu_manager_uart.py`, `utils.py`, `firmware_version.py` (single FW version source).
- Launches: `battery_alert.yaml`.
- Per-model configs: [config/rosbot_xl/config.yaml](rosbot_utils/config/rosbot_xl/config.yaml).

---

## 3. Launch flows

### 3.1 Real ROSbot (XL)

1. `ros2 launch rosbot_bringup rosbot_xl.yaml`.
2. `backend:=microros|mavlink` (default `mavlink`) picks `microros.launch.py` or `mavlink.launch.py`. Each runs `configure_robot` (FW check vs `FIRMWARE_VERSION` + `BACKEND:<backend>` ACK + `NS:<ns>` ACK + `END` close) → on success starts the matching upstream node (`micro_ros_agent udp4 --port 8888` or `rosbot_mavlink_bridge`).
3. `rosbot_controller/controller.yaml` → sed-resolves `controllers.yaml`, `robot_state_publisher` with resolved URDF (`use_sim=False`, `<ros2_control>` uses `RosbotSystem`/`RosbotImuSensor`), after 3 s `controller_manager` + spawners, after 5 s `manipulator.yaml` if `configuration ∈ {manipulation, manipulation_pro}`.
4. `rosbot_joy/joy.yaml`, `rosbot_localization/ekf.yaml`, (XL) `led_strip_car_wave` if `led_strip:=True`.

### 3.2 Simulation (XL)

1. `ros2 launch rosbot_gazebo simulation.yaml robot_model:=rosbot_xl`.
2. `gz_sim` with `husarion_world.sdf` + `/clock` bridge.
3. `spawn_robot.yaml` group: resolve `rosbot_bridge.yaml` → cmd_vel bridge → `ros_gz_sim/create -topic robot_description` → `gz_components` sensor bridges → `controller.yaml use_sim:=True` → joy / EKF / RViz.

### 3.3 Manipulator (HW XL `configuration:=manipulation`)

- Arm idle on startup unless `arm_activate:=True`.
- Runtime: `ros2 run rosbot_controller arm_control active`.
- Control: pad OR RViz MotionPlanning. **Not both at once.**
- Safe shutdown: `arm_control inactive` (warning: arm falls under gravity — hold it).

---

## 4. External configuration (`config_dir`)

Every launch accepts `config_dir` (default `''`). When non-empty, paths flip from `<pkg-share>/config/…` to `<config_dir>/<pkg>/config/…`. Snap relies on this for native editing without rebuild.

```bash
ros2 run rosbot_utils create_config_dir ~/my_rosbot_config
```

Creates: `rosbot_bringup/config/`, `rosbot_controller/config/`, `rosbot_description/config/`, `rosbot_joy/config/`, `rosbot_localization/config/`, `rosbot_moveit/config/`, `rosbot_utils/config/` + `firmware/`.

---

## 5. Namespacing + multirobot

- `ROBOT_NAMESPACE` env → `namespace` arg. Inside launch: `push_ros_namespace` prefixes every node with `/<ns>/`.
- TF namespaced via `set_remap /tf → tf` and `/tf_static → tf_static` in every launch. Global `/tf` (for nav2 / multirobot RViz) bridged by [tf_namespace_bridge](https://github.com/husarion/tf_namespace_bridge) — `tf_namespace_bridge:=True` + `namespace≠''`. `frame_filters` (glob on `child_frame_id`) controls which frames bridge; default empty = pass-through. Configs: [rosbot_bringup/config/tf_namespace_bridge.yaml](rosbot_bringup/config/tf_namespace_bridge.yaml), [rosbot_gazebo/config/tf_namespace_bridge.yaml](rosbot_gazebo/config/tf_namespace_bridge.yaml).
- Multirobot sim: `spawn_robot.yaml` per robot with distinct `namespace` + `(x,y,z)`.
- IMU `sensor_name: <namespace>/imu` (placeholder sed-replaced) — Gazebo needs unique sensor names.

### Sim vs HW divergence (intentional, upstream constraint)

- **HW** — every node lives in `GroupAction` with `push_ros_namespace`. Canonical idiom, handles every service/topic/action.
- **Sim** — `controller_manager` is hosted by `gz_ros2_control-system` plugin inside the Gazebo process, loaded by `gz_sim` BEFORE `LaunchContext` can apply `push_ros_namespace`. The plugin's `<ros><namespace>` handles relative names; absolute names (`/controller_manager/list_controllers` etc. + `/diagnostics` / `/tf` / `/tf_static` / introspection topics) need explicit URDF `<remapping>` entries in [common/gazebo.urdf.xacro](rosbot_description/urdf/common/gazebo.urdf.xacro). Current block covers 14 entries — full public CM surface in `ros-jazzy-controller-manager` (11 services + 3 diagnostic topics).

Regression guards: [test_namespace_isolation.py](rosbot_bringup/test/test_namespace_isolation.py) (HW), [test_xacro::test_gazebo_urdf_namespace_remappings](rosbot_description/test/test_xacro.py) (sim URDF).

---

## 6. Public topics (excerpt)

Full list → [ROS_API.md](ROS_API.md). All namespaced when `ROBOT_NAMESPACE` set.

| Direction | Topic | Comment |
|---|---|---|
| sub | `cmd_vel` (TwistStamped) | drive controller; XL=mecanum, ROSbot=diff |
| pub | `odometry/wheels` | drive controller, covariances tuned empirically |
| pub | `imu/data` | imu_broadcaster (HW: rosbot_imu_sensor → broadcaster) |
| pub | `odometry/filtered` | EKF fusion |
| pub | `tf`, `tf_static` | rsp + drive (`enable_odom_tf:false`, so mostly EKF) |
| pub | `joint_states` | joint_state_broadcaster |
| pub | `scan` | raw lidar |
| pub/sub | `joy` | manual control |
| pub | `dynamic_joint_states`, `diagnostics`, `robot_description` | ros2_control standard |

---

## 7. Firmware + pre-communication

- Single runtime-switch binary: `rosbot_utils/firmware/rosbot[_xl]-${FIRMWARE_VERSION}.bin` (covers both micro-ROS and MAVLink; backend chosen at boot via the `BACKEND:` handshake line). Version pinned in [`rosbot_utils.firmware_version`](rosbot_utils/rosbot_utils/firmware_version.py) — `configure_robot` rejects any other FW string.
- Flash: `ros2 run rosbot_utils flash_firmware --robot-model rosbot[_xl]`. XL → `--usb` auto (FTDI); ROSbot → UART over GPIO (gpiochip reset).
- Pre-comm serial protocol: MCU emits `FW: ${FIRMWARE_VERSION}\n` → host (optional) `BACKEND:microros|mavlink\n` → MCU `ACK\n` → host `NS:<namespace>\n` → MCU `ACK\n` → host `END\n` (closes pre-comm without waiting on the 2.5 s firmware timeout; older firmware ignores it).
- USB-B PC development: see [CONTRIBUTING.md](CONTRIBUTING.md) (requires manual `btn1`/`btn2` + reset).

---

## 8. Where to find what

| Looking for... | Go to... |
|---|---|
| public topic / node names | [ROS_API.md](ROS_API.md) |
| launch arguments | [README.md](README.md), `ros2 launch <p> <f> -s` |
| build / commit | [CLAUDE.md](CLAUDE.md), [CONTRIBUTING.md](CONTRIBUTING.md) |
| XL arm behavior / restart | [MANIPULATOR.md](MANIPULATOR.md) |
| drive parameters | `rosbot_controller/config/<model>/controllers.yaml` |
| EKF parameters | `rosbot_localization/config/config.yaml` |
| YAML launch example | any `*.yaml` under `*/launch/` |
| URDF geometry | `rosbot_description/urdf/<model>/body.urdf.xacro` |
| MoveIt SRDF / IK | `rosbot_moveit/config/rosbot_xl.srdf`, `kinematics.yaml` |
| HW plugins | `rosbot_hardware_interfaces/src/*.cpp`, `rosbot_hardware_interfaces.xml` |
| firmware + protocol | `rosbot_utils/firmware/`, `rosbot_utils/scripts/configure_robot` |
| Docker | `docker/Dockerfile.{hardware,simulation}`, `docker/compose.*.yaml` |
| CI | `.github/workflows/{ci,tests,build-docker,backport}.yaml` |

---

## 9. External boundaries

- `husarion_components_description` — component schema, per-type xacro macros (LDR06, CAM11, …). Change = URDF rebuild.
- `husarion_controllers/husarion_mecanum_drive_controller` — mecanum controller.
- `husarion_gz_worlds` — SDF world + plugins. Default: `husarion_world.sdf`.
- `tf_namespace_bridge` — TF bridge code.
- `rosbot-firmware` (<https://github.com/husarion/rosbot-firmware>) — STM32 firmware. MCU topics documented in its own `ROS_API.md`.

All pinned in `rosbot/rosbot_*.repos`. Bump = check `test_xacro` + `controllers.yaml` collisions.
