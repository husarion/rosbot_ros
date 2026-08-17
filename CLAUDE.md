# CLAUDE.md — `rosbot_ros` working guide

How to work here. Architecture and inter-package contracts → [ARCHITECTURE.md](ARCHITECTURE.md).

> **Top rule:** draft a short spec + flag sensitive spots **before** coding non-trivial changes.

---

## 1. Context

- ROS 2 **jazzy** (Ubuntu 24.04) packages for Husarion **ROSbot 2/3** and **ROSbot XL** (optional OpenMANIPULATOR-X arm).
- Workspace: `~/Husarion/Workspaces/rosbot_ws/`, repo as `src/rosbot_ros`.
- Main / PR target branch: **`jazzy`** (not `main`).
- Targets: real robot (micro-ROS) **or** Gazebo Harmonic sim — shared URDF + ros2_control.
- Packages: `rosbot` (meta), `rosbot_bringup`, `rosbot_controller`, `rosbot_description`, `rosbot_gazebo`, `rosbot_hardware_interfaces`, `rosbot_joy`, `rosbot_localization`, `rosbot_moveit`, `rosbot_utils`.

---

## 2. Spec before code (mandatory)

For new launches, public args, hardware interfaces, controllers, nodes, URDF/firmware changes — first state:

1. **Goal** (one sentence).
2. **Impact** (packages, public args, firmware/snap compat).
3. **Sensitive spots** — `rosbot_hardware_interfaces` (ABI ↔ firmware), `controllers.yaml` "Based on real measurements" lines, URDF/xacro (MoveIt SRDF regen), public topics in [ROS_API.md](ROS_API.md).
4. **Test plan** (pre-commit, `colcon test`, manual sim/HW run).

15 min alignment beats a re-do after PR review.

---

## 3. Workflow

### 3.1 First-time setup

```bash
cd ~/Husarion/Workspaces/rosbot_ws
vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos
vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos
export PIP_BREAK_SYSTEM_PACKAGES=1
sudo rosdep init   # only if not previously initialized
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
pip install pre-commit && (cd src/rosbot_ros && pre-commit install)
```

### 3.2 Daily loop (at workspace root)

```bash
source /opt/ros/jazzy/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <pkg>
# or --packages-up-to <pkg> after URDF/dep changes
colcon test --packages-select <pkg> && colcon test-result --verbose
source install/setup.bash
```

**Symlink-install** symlinks `launch/`, `config/`, `urdf/`, Python `scripts/` — edit in `src/` works without rebuild. C++ changes still need rebuild.

### 3.3 Pre-commit + commit

`pre-commit run -a` runs all hooks (also fires on `git commit`). Hooks: `black` (LL=99), `isort` (black profile), `flake8` (ignore E501,W503), `clang-format`, `cmake-format`, `codespell`, `doc8` (LL=100), `markdownlint-fix`, `prettier-package-xml`, `sort-package-xml`, `ament_copyright`, `yamlfmt`. **Don't add new formatters without agreement.**

### 3.4 Commits + PRs

- **No direct commits to `jazzy`.** Feature branch + PR.
- PR template has a `## Changelog description` section — fill it.
- Backports via `backport.yaml` after labeling.

### 3.5 Run after build

```bash
ros2 launch rosbot_bringup rosbot.yaml             # ROSbot 2/3
ros2 launch rosbot_bringup rosbot_xl.yaml          # ROSbot XL
ros2 launch rosbot_gazebo simulation.yaml robot_model:=rosbot_xl
# args: README.md "Launch Arguments" or `ros2 launch <pkg> <file> -s`
```

---

## 4. CI / Docker / Devcontainer

- **Devcontainer:** `.devcontainer/compose.yaml` mounts repo at `/home/husarion/ros2_ws/src/rosbot_ros`. `runtime: nvidia` (needs NVIDIA Container Toolkit).
- **Production images:** `docker/Dockerfile.{hardware,simulation}`, built by `build-docker.yaml` on push to `jazzy`. HW drops `rosbot_gazebo`, sim drops `rosbot_bringup`.
- **CI:** `ci.yaml` orchestrates `pre-commit` → `tests.yaml` (`colcon test` over `packages-select-regex: rosbot*`) → (on `jazzy`) `build-docker.yaml`. `rosbot_bringup` tests run with fake HW topics (`hardware_bridge:=False`); `rosbot_gazebo` keeps only offline launch/schema tests (real-Gazebo tests removed).
- **VS Code tasks:** inherited from `panther` repo, `build.sh` uses `--packages-up-to panther` — prefer manual `colcon build` from §3.2.

---

## 5. Conventions

- **License header** (Apache 2.0) in every new source file (`ament_copyright` enforced). Maintainer: `support@husarion.com`.
- **Python:** `black` LL=99, type hints encouraged. Scripts → `<pkg>/scripts/`, installed via `install(PROGRAMS …)` to `lib/${PROJECT_NAME}`. Modules → `ament_python_install_package`.
- **C++:** `clang-format` (default ROS), `-Wall -Wextra -Wpedantic` (+`-Wshadow -Wold-style-cast` in `rosbot_joy`). C++17 min. HW plugins via `pluginlib` in `rosbot_hardware_interfaces.xml`.
- **Launch:** prefer YAML frontend. Use Python only when logic is required (example: [microros.launch.py](rosbot_bringup/launch/microros.launch.py)). Keep arg shape — tests, snaps, sibling packages anchor to `namespace`, `robot_model`, `config_dir`, `mecanum`, `use_sim`.
- **Configs:** every package accepts `config_dir`. Default = `share/<pkg>/config/`; with `config_dir` set, reads `<config_dir>/<pkg>/config/…` (snap relies on this — **don't break it**). Generate external dir: `ros2 run rosbot_utils create_config_dir <dst>`.
- **Naming:** packages `rosbot_*`; launch verbs/functions (`controller.yaml`, `simulation.yaml`, `microros.launch.py`); public topics — see [ROS_API.md](ROS_API.md). **Renaming a public topic = breaking change.**
- `/tmp/rosbot_*_<namespace>.{yaml,urdf}` are runtime-resolved configs — don't commit.

---

## 6. Hard rules (firm NO's)

1. **Don't change `FIRMWARE_VERSION`** in [rosbot_utils/firmware_version.py](rosbot_utils/rosbot_utils/firmware_version.py) without swapping `rosbot_utils/firmware/rosbot[_xl]-${FIRMWARE_VERSION}.bin` in lockstep. Driver ↔ firmware are tightly coupled; `configure_robot` rejects mismatched strings.
2. **Don't commit `build/`, `install/`, `log/`, `*.pyc`** — gitignored. Same for vcstool submodules.
3. **Don't disable pre-commit hooks** without approval — CI enforces.
4. **Don't change `controllers.yaml` "Based on real measurements" lines** without fresh measurements.
5. **Don't enter `manipulation`/`manipulation_pro` without reading [MANIPULATOR.md](MANIPULATOR.md)** — HW pitfalls (power loss → arm falls).
6. **Don't ship new topics/launch args without updating [ROS_API.md](ROS_API.md) + table in [README.md](README.md).**
7. **Don't touch `tf_namespace_bridge`** outside designated cases — it's the only bridge from namespaced TF to global `/tf` (nav2 / multirobot).

---

## 7. Adding new functionality

> **Automation:** the checklist is wired into `/feature`. Lighter: `/quick-fix` (≤3 files, no API change), `/spec` (draft only), `/review` (optimizer+security+tester on current branch). See [.claude/README.md](.claude/README.md).

Knowledge-base updates required for any public change:

- [ ] [ARCHITECTURE.md](ARCHITECTURE.md) — new inter-package contract / node / topic / plugin.
- [ ] [ROS_API.md](ROS_API.md) — public topic / node / launch.
- [ ] [README.md](README.md) — launch arg added/changed.
- [ ] [MANIPULATOR.md](MANIPULATOR.md) — arm-related.
- [ ] Tests (`test_xacro.py`, `test_bringup.py`, `test_launch_offline.py`).
- [ ] `pre-commit run -a`, `colcon build`, `colcon test`.
- [ ] [§9](#9-decision-history) — one bullet, one sentence, PR link.

---

## 8. Sensitive spots

- **micro-ROS:** ROSbot 2/3 → serial (`/dev/ttySERIAL`, 921600). ROSbot XL → udp4 (port 8888). Mode via `microros_mode` arg. `configure_robot` checks firmware version + sets namespace over serial before agent starts.
- **Namespace:** `ROBOT_NAMESPACE` env → `--ros-args`. Inside robot TF is namespaced (`/<ns>/tf`); `tf_namespace_bridge` re-publishes to global `/tf`.
- **XL `configuration` arg:** `basic|telepresence|autonomy|manipulation|manipulation_pro|custom`. Drives component list in `rosbot_description/config/rosbot_xl/<configuration>.yaml` + whether manipulator launches.
- **XL `controllers.yaml` placeholder:** `<manipulator_state>` is sed-replaced with `active`/`inactive` before `controller_manager` starts. See [controller.yaml](rosbot_controller/launch/controller.yaml).
- **Sim vs HW ros2_control:** same URDF, different `<plugin>` — `rosbot_hardware_interfaces/RosbotSystem` (HW) vs `gz_ros2_control/GazeboSimSystem` (sim). See [ros2_control.urdf.xacro](rosbot_description/urdf/common/ros2_control.urdf.xacro).
- **EKF** fuses `odometry/wheels` + `imu/data` → `odometry/filtered`. `enable_odom_tf: false` on drives — EKF publishes `odom→base_link` TF.
- **Laser filter** crops points inside robot body. Per-model: [rosbot_utils/config/{rosbot,rosbot_xl}/config.yaml](rosbot_utils/config/).
- **`rosbot_bringup`/`rosbot_gazebo` CI tests run offline** (fake HW topics / launch-schema only) — still do a real sim/HW run after non-trivial launch changes.

---

## 9. Decision history

Format: `date — one-line summary (link)`. Detail lives in code + [.claude/specs/](.claude/specs/) + [.claude/plans/](.claude/plans/).

- *2026-04-30 — initial CLAUDE.md / ARCHITECTURE.md.*
- *2025-04-21 — `led_strip` arg in [rosbot_xl.yaml](rosbot_bringup/launch/rosbot_xl.yaml) (commit `bbd741b`).*
- *2025-04-XX — firmware → `v1.1.0-jazzy`, improved PID (commits `b87b1a4`, `e5509b2`).*
- *2026-05-04 — `frame_filters` parameter exposed for `tf_namespace_bridge` via per-package config (default empty = pass-through).*
- *2026-05-15 — MoveIt topic namespacing for `joy2servo`/`servo_node` fixed via `MoveGroupInterface::Options` 3-arg ctor + relative `monitored_planning_scene_topic`. `/parameter_events` global by design.*
- *2026-05-15 — `rosbot_moveit` refactor: `joy2servo` moved from `rosbot_joy`; ~1000 LOC of dead code removed; `dock`/`home` deduplicated via `arm_pose_mover` shared lib; `ompl_planning.yaml` 192→41 lines; `servo.launch.py` typo + symmetry fix; `test_moveit_config.py` regression guards.*
- *2026-05-15 — manipulation HW noise classified — most accepted upstream (`planner_id` empty, `OccupancyMapMonitor`, `controller_manager` deprecated `-r`, RT scheduling, dynamixel matrix, `RosbotSystem` startup race). `validate_workspace_bounds` fixed via `MoveGroupInterface::setWorkspace(±0.5m)` in [arm_pose_mover.cpp](rosbot_moveit/src/arm_pose_mover.cpp) + [joy2servo.cpp](rosbot_moveit/src/joy2servo.cpp).*
- *2026-05-15 — Cartesian arm rerouted around `moveit_servo` POSE path; `joy2servo` runs IK locally (KDL + `position_only_ik`) and publishes JointJog (skips singularity guard). See [joy2servo.cpp](rosbot_moveit/src/joy2servo.cpp).*
- *2026-05-15 — sim-warning cleanup for `simulation.yaml configuration:=manipulation`: `manipulator_controller` `interpolate_from_desired_state` (replaces deprecated `open_loop_control`), `default_workspace_bounds: 1.0`, `default_planner_config: RRTConnectkConfigDefault`, [home.launch.py](rosbot_moveit/launch/home.launch.py) wrapper for kinematics injection. Sim residue (OccupancyMapMonitor always-on, `ParallelGripperCommand` unsupported in MoveIt simple manager) — accepted.*
- *2026-05-18 — `FIRMWARE_VERSION` deduplicated → single source in `rosbot_utils.firmware_version`.*
- *2026-05-19 — HW `gripper_right_joint`-missing-from-`/joint_states` fix attempted via xacro override → **reverted**: `dynamixel_hardware_interface` counts mimic joints against `number_of_joints=5`, separate `<ros2_control>` block also rejected. Cosmetic only; needs upstream Robotis + ros2_control changes.*
- *2026-05-19 — [dock.launch.py](rosbot_moveit/launch/dock.launch.py) added mirroring [home.launch.py](rosbot_moveit/launch/home.launch.py) — `MoveItConfigsBuilder` injects `semantic`/`kinematics`/`joint_limits` so `MGI` stops logging `No kinematics plugins defined`. `dock` exposes `namespace:=` arg (`home` doesn't — already auto-included from `manipulator.yaml`).*
- *2026-05-19 — duplicated `move()` calls in `joy2servo` `MoveToDockPose`/`MoveToHomePose` dropped → **reverted (band-aid restored)** on 2026-05-19→20: `MGI` from-Dock to-Home staleness reproduces; root cause likely in server-side PSM. See FOLLOWUP_PLAN.md C4.*
- *2026-05-19 — [rviz.launch.py](rosbot_moveit/launch/rviz.launch.py) namespace-aware: `namespace:=` arg + `/tf|/tf_static|/diagnostics` remaps + sed-resolved `moveit.rviz` to `/tmp/rosbot_moveit_<ns>.rviz` (panel reads ns from rviz config, not node `__ns:=`). HW-verified.*
- *2026-05-19 — `gripper_controller` switched to `JointTrajectoryController` (was `GripperActionController`); `joy2servo::ControlGripper` publishes single-point trajectories directly at 20 Hz (no MGI in hot path). **Breaking:** `gripper_cmd` action gone, use `follow_joint_trajectory`. `gripper_left_joint` limits → physical m/s, `max_velocity: 0.08`. Test `test_gripper_uses_follow_joint_trajectory` guards moveit_controllers.*
- *2026-05-20 — Cartesian self-collision regression fix in `joy2servo`: new `cartesian_max_joint_velocity` (rad/s, default 1.0) — scales whole velocity vector when any joint exceeds. `collision_check_rate: 10→30 Hz`, `self_collision_proximity_threshold` upstream default 0.02 m. [servo.launch.py](rosbot_moveit/launch/servo.launch.py) drops `joy2servo` default log_level to WARN, whitelists `joy2servo:=info` to silence `kdl_kinematics_plugin` "Using position only ik" spam. Per-logger filter for `moveit.kinematics.kdl_kinematics_plugin` doesn't work in jazzy (auto-named plugin nodes).*
- *2026-05-21 — namespace audit ([.claude/plans/namespace-audit.md](.claude/plans/namespace-audit.md)): sim CM services namespaced via 14 URDF `<remapping>` entries in [gazebo.urdf.xacro](rosbot_description/urdf/common/gazebo.urdf.xacro) (push_ros_namespace can't reach `gz_ros2_control`-hosted CM). `micro_ros_agent` defensive `namespace=` attempted then **reverted** — produced `__ns:=/<ns>/<ns>` double-stack (push_ros_namespace IS active inside `OnProcessExit` callback). EKF services land at `/<ns>/{set_pose,enable,toggle}` (relative, not `ekf_node/`-prefixed). Regression guards: [test_namespace_isolation.py](rosbot_bringup/test/test_namespace_isolation.py), [test_xacro::test_gazebo_urdf_namespace_remappings](rosbot_description/test/test_xacro.py), [test_launch_offline::test_microros_agent_node_has_no_explicit_namespace](rosbot_bringup/test/test_launch_offline.py). HW + sim retest: zero leaks (only `/tf`, `/tf_static`, `/parameter_events`, `/rosout` on root, plus sim-only `/gz_bridge/*` + `/launch_ros_*`). Snapshots in [.claude/scratch/](.claude/scratch/). **Caveat:** xacro comments going to gz_sim must have NO `:` (gz's URDF→SDF uses yaml.safe_load).*
- *2026-05-21 — log-noise audit Phase A/B/C ([spec](.claude/specs/log-noise-reduction.md), [classification](.claude/scratch/logs/classification.md), [proposals](.claude/scratch/logs/proposals.md)): 4 baseline captures, 86 unique groups, 21 proposals tiered. **Surprises:** (1) Dynamixel SDK raw `cout` ≈105 lines hw-manipulation, NOT silencable via `--log-level` (only `output='log'`, high UX cost — see [[feedback-log-level-misses-cout]]). (2) `led_strip_{ready,rainbow}` KeyboardInterrupt traceback (18 lines on Ctrl+C) fixed via `try/except + try_shutdown`. (3) `process has died exit code -2` is launch-core ERROR on benign Ctrl+C — needs launch hook. (4) `gz_ros_control` plugin INFO lives inside gazebo process — `--log-level` may not propagate. **60% target revised → ~26% hw-m / ~21% sim-m** with wariant A alone.*
- *2026-05-21 — log noise Phase D partial: pretty-header `log:` actions in [rosbot.yaml](rosbot_bringup/launch/rosbot.yaml)+[rosbot_xl.yaml](rosbot_bringup/launch/rosbot_xl.yaml) (jazzy YAML frontend action is `log` not `log_info`, attribute is `message` not `msg`). `tf_namespace_bridge` `signal_handler` INFO fixed **at source** (sibling repo `f6f32bc`): `SignalHandlerOptions::None` + `std::signal` quiet handler → exit code 0 (was -2), launch core no longer prints red `process has died`. `led_strip_{ready,rainbow}` Python `KeyboardInterrupt` traceback wrapped (try/except + `try_shutdown`). micro_ros_agent silencing skipped (`-v 3` flag tried, reverted).*
- *2026-05-21 — MAVLink as default `backend:=mavlink|microros` arg on `rosbot[_xl].yaml` (paired with `rosbot_mavlink_bridge`). Renamed legacy `microros` bool → `hardware_bridge`.*
- *2026-05-22 — runtime-switch firmware handshake: `configure_robot --backend` emits `BACKEND:` line before `NS:`; missing ACK is fatal so legacy single-protocol firmware fails loud rather than silently defaulting the namespace. `link_layer` arg renamed to `backend` to match firmware-side `CommBackend` (breaking, no alias).*
- *2026-05-22 — added [justfile](justfile) with `just <recipe>` helpers (build, test, hw, sim, precommit) — handles workspace auto-detect for both main checkout and `.claude/worktrees/<name>/` worktrees, with idempotent `vcs import` of sibling repos. Firmware quirk worth remembering: ROSbot 2/3 firmware leaves `sensor_msgs/BatteryState.power_supply_status` at `UNKNOWN` regardless of charger state; only ROSbot XL fills it in (`CHARGING` / `FULL` / `DISCHARGING`). Any future code touching `power_supply_status` must either restrict to `rosbot_xl` or include `UNKNOWN` in the allowlist.*
- *2026-05-29 — pinned `RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"` via `set_env` in [rosbot.yaml](rosbot_bringup/launch/rosbot.yaml), [rosbot_xl.yaml](rosbot_bringup/launch/rosbot_xl.yaml), [simulation.yaml](rosbot_gazebo/launch/simulation.yaml) (next to existing `RCUTILS_COLORIZED_OUTPUT`). Value equals the ROS default — purpose is to override any shell-exported format so logs render consistently. **Dead end confirmed:** `RCUTILS_LOGGING_SEVERITY_THRESHOLD` does NOT exist in rcutils on any distro (incl. Rolling); no env var sets the default log *level* — only `--ros-args --log-level`. So no `log_level` arg was added.*
- *2026-07-23 — `husarion_asset_server` folded into `rosbot_bringup`'s own launch (`node:` under `push_ros_namespace` in [rosbot.yaml](rosbot_bringup/launch/rosbot.yaml)/[rosbot_xl.yaml](rosbot_bringup/launch/rosbot_xl.yaml), new `asset_server` arg, default `True`) instead of running as a separate snap daemon — gets the correct namespace at startup, no more restart-to-re-announce hack. Vendored as a new [husarion_asset_server](husarion_asset_server/) package that fetches + sha256-verifies the upstream prebuilt release binary (not built via `colcon-ros-cargo`/`cargo-ament-build` — avoids a Rust/clang toolchain in dev/CI/the `rosbot-snap` build). `husarion_asset_msgs` added to `rosbot_hardware.repos`. CI test_bringup passes `asset_server:=False` to stay offline.*
- *2026-06-22 — `led_strip/enable` `std_srvs/SetBool` service added to both `led_strip_{ready,rainbow}` nodes — `data:=false` cancels the 40ms timer (no image compute, no publish), `data:=true` resets it (resumes from current phase/position). Documented in [ROS_API.md](ROS_API.md) "Available Services"; regression guard `test_led_strip_enable_service` in [test_led_strip.py](rosbot_utils/test/test_led_strip.py).*
- *2026-08-11 — `led_strip/enable` `SetBool` restored on `animation_publisher`. The service was documented in [ROS_API.md](ROS_API.md) but had been lost when `led_strip_{ready,rainbow}` were retired in favour of the PNG-driven node, so the docs described an API nothing served. Publishing is now gated by `enabled_ && active_ != none` in a single `RestartTimer()`, which both the service and the `current_animation` callback go through — so switching animation while disabled stores the selection without resuming, and `enable=true` brings back whatever was picked. `std_srvs` was already a `package.xml` depend (leftover from the retired nodes), only the `ament_target_dependencies` line needed it. Guard: `test_led_strip_enable_service`.*
- *2026-08-11 — animations are now selectable at runtime from `config_dir`, not just from the startup scan. `current_animation` re-reads `<name>.png` + `<name>.yaml` from disk on every set (`TryLoadFromDisk`, user dir first) and only falls back to the cached copy, so dropping a PNG into `<config_dir>/rosbot_utils/animations` and selecting it needs no driver restart; the rejection message now names the directories that were searched. [rosbot_xl.yaml](rosbot_bringup/launch/rosbot_xl.yaml) gained an `animations_dir` let deriving that path from `config_dir` — the snap needs no change, `create_config_dir` already copied `rosbot_utils/animations` and `daemon_start.sh` already passes `config_dir:=`. **Gotcha found while testing:** `create_config_dir` used bare `shutil.copytree`, which aborts on a dangling symlink — routine in a `--symlink-install` tree whenever a source file is deleted without a rebuild, and it left a half-populated `config_dir`. Now `ignore_dangling_symlinks=True` + per-folder `shutil.Error` handling. Guard: `test_animation_loaded_from_user_dir_at_runtime`.*
- *2026-08-11 — `animation_publisher` follows `twist_mux_controller/source`, so the strip shows who is driving: `autonomous` → `navigation_animation` (default `navigation`), `manual`/`unknown`/`not_published` → `ready_animation` (default `ready`). The subscription must be **`transient_local` + `reliable`** to match the controller's latched publisher — it republishes only on handover, so a volatile subscriber would show nothing until the next change. The switch goes through `set_parameter("current_animation", …)` rather than `Activate()` directly, so `ros2 param get` keeps reporting what is actually on the strip and the disk re-read path (hence `config_dir` animations) is reused; `Node::set_parameter` returns a rejection in `SetParametersResult` instead of throwing, verified against `rclcpp/node.hpp`. All three parameters are read live in the callback, so `follow_cmd_vel_source:=False` hands the strip back at runtime with no subscription juggling. Follow mode owns `current_animation` while enabled — including over a hand-set `none`; use `led_strip/enable` to silence instead. Guard: `test_animation_follows_cmd_vel_source`. The three parameters plus `led_count` live in [rosbot_utils/config/rosbot_xl/config.yaml](rosbot_utils/config/rosbot_xl/config.yaml) next to `battery_alert`, loaded via an `animation_publisher_config` let that mirrors [battery_alert.yaml](rosbot_utils/launch/battery_alert.yaml)'s `config_dir`-or-share resolution. The `from:` entry is listed **before** the `name:`/`value:` params in the node block — later entries win in the launch frontend, so `led_animation` and the `config_dir`-derived `user_animations_dir` stay authoritative over the file.*
- *2026-05-25 — bumped firmware to `v2.0.0-jazzy` (runtime-switch — one binary covers both micro-ROS and MAVLink, picked at boot via the `BACKEND:` handshake line). Bundled `rosbot[_xl]_mavlink-v0.1.1.bin` and `rosbot[_xl]-v1.1.0-jazzy.bin` dropped; `--variant` flag and `--expected-firmware` arg removed (single binary, `FIRMWARE_VERSION` is the only source of truth).*
- *2026-08-10 — `twist_mux_controller` (chainable, from the already-pinned `husarion_controllers`) added to both models' `controllers.yaml` + spawner, mirroring `husarion_ugv_ros`. Priority inputs `manual/cmd_vel` (100) > `autonomous/cmd_vel` (10) > `cmd_vel` (1), 0.2 s fallthrough; `rosbot_joy`'s `joy_vel` default moved to `manual/cmd_vel` so the pad overrides nav2. **Gotcha:** base controllers disagree on reference-interface naming — `diff_drive_controller` exports `<name>/linear|angular/velocity` while `mecanum_drive_controller` / `omni_wheel_drive_controller` / `husarion_mecanum_drive_controller` keep the axis (`<name>/linear/{x,y}/velocity`, `<name>/angular/z/velocity`). Rather than encode that per robot, `twist_mux_controller` gained a `drive_controller` param (husarion_controllers `6181d6d`, **pin bumped in `rosbot_hardware.repos` — the two must land together**) that derives the names from the controller name + `holonomic`; empty keeps the legacy explicit `command_interface_*` path. `controllers.yaml` therefore only substitutes `<drive_controller>` + `<mecanum>`, both from the `mecanum` arg via a shared `drive_controller` let that also feeds the spawner. `holonomic` takes the raw `mecanum` arg — the ROS yaml parser accepts capitalised `True`/`False` as bool, verified via `rclpy.parameter.parameter_dict_from_yaml_file`. **Jazzy-only:** Humble's `diff_drive_controller` is not chainable, so the humble branch keeps the direct `cmd_vel` path. Sim-verified on both drive types (chained interfaces + manual-overrides-autonomous + fallthrough on release); `cmd_vel` stays functional at priority 1. Follow-up outside this repo: `rosbot-snap`'s `teleop_launcher.sh` still publishes to `cmd_vel`, so keyboard teleop now loses to nav2 — point it at `manual/cmd_vel`.*
- *2026-08-12 — Attempted composing `tf_namespace_bridge` + `robot_state_publisher` into one `rclcpp_components` `node_container` in `rosbot_description` (skip a serialize/DDS hop on the highest-frequency `/tf` publisher + its direct subscriber) — **reverted, do not retry without reading this first.** `LoadComposableNodes`/`ComposableNodeContainer` lazily creates a process-wide singleton `ROSAdapter` (`launch_ros/ros_adapters.py`, `get_ros_node()`) the first time anything needs a service client, and registers an `OnShutdown` handler that calls `ros_adapter.shutdown()`. `rosbot_controller/controller.yaml`'s `ros2_control_node` has `on_exit: shutdown` (deliberate fail-fast — see below on why that stays) — every exit of that node, including a completely normal shutdown, fires an *extra* `Shutdown` action that races the planned one. Two `Shutdown`s in flight → `OnShutdown` fires twice → the second `ros_adapter.shutdown()` raises `RuntimeError: Cannot shutdown a ROS adapter that is not running` → `[ERROR] [launch]: Caught exception in launch` → the SIGINT→SIGTERM escalation for the *remaining* processes gets interrupted, so unrelated ones (`husarion_asset_server`, the controller `spawner`) die via raw signal instead of a clean exit. 100% reproducible (8/8 `test_bringup` parametrizations + `test_namespace_isolation` failed in CI, run [31591651133](https://github.com/husarion/rosbot_ros/actions/runs/31591651133)), not flaky — happens on *every* bringup shutdown once any `node_container` exists anywhere in the tree, not just on an actual controller crash. Same symptom (`Cannot shutdown a ROS adapter that is not running` + `component_container`) independently confirmed in [navigation2#4722](https://github.com/ros-navigation/navigation2/issues/4722) — a `launch_ros` framework interaction, not something fixable from our YAML. **Why `on_exit: shutdown` stays despite this:** without it, a controller_manager crash leaves the rest of the stack (RSP, EKF, joy, LED, asset_server, HW/MAVLink bridge) running in a silent degraded state — topics still flow, robot looks alive, nothing actually drives it. The snap's `daemon` service (`snapcraft_template.yaml.jinja2`) has `restart-condition: always` + `restart-delay: 5s` specifically so a full fail-fast shutdown is cheap to recover from — the two mechanisms are meant to work together. Kept from this attempt (harmless on their own, no `node_container` involved): `rosbot_description` is now the single owner of `tf_namespace_bridge.yaml` (was duplicated identically across `rosbot_bringup`/`rosbot_gazebo`/`rosbot_description`); its path is an internal `let` (matching `components_config`'s pattern), not a public arg. Don't retry composable nodes anywhere `on_exit: shutdown` (or any other `OnProcessExit`-triggered `Shutdown`) is in the same launch tree until upstream `launch_ros` fixes the double-shutdown race.*
- *2026-08-17 — CPU-cost analysis of the manipulator stack on the Jetson Orin Nano identified `moveit_servo`'s `collision_monitor` loop as the single hottest thread (~91% of one core, live-measured) and `dynamixel_hardware_interface`'s `read()` as a real-time-budget risk (2.7–4.9 ms of a 10 ms/100 Hz cycle, ~200–300x costlier than the base's HW interface, per `/controller_manager/statistics/full`) rather than a CPU-% cost (blocking serial I/O). Fixed [moveit_servo.yaml](rosbot_moveit/config/moveit_servo.yaml): `is_primary_planning_scene_monitor` `true→false` (move_group.launch.py already runs the canonical primary PSM — servo was duplicating its own world/scene monitor against the yaml's own documented guidance) and `collision_check_rate` `30→20 Hz` (still 1.33x margin under the 20 mm self-collision threshold at the 1 rad/s joy2servo clamp, ~33% cut on the always-on dual self+scene FCL check loop). Needs an HW re-verify (fast jog toward self-collision/an obstacle, confirm it still stops before contact) before trusting in the field — not yet HW-tested.*

---

## 10. Quick command reference

| what | command |
|---|---|
| Build single package | `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <pkg>` |
| Build with dependents | `--packages-up-to <pkg>` |
| Test single package | `colcon test --packages-select <pkg> && colcon test-result --verbose` |
| Pre-commit | `pre-commit run -a` |
| List launch args | `ros2 launch <pkg> <file> -s` |
| Flash firmware | `ros2 run rosbot_utils flash_firmware --robot-model rosbot[\|_xl]` |
| Activate manipulator | `ros2 run rosbot_controller arm_control active` |
| External config dir | `ros2 run rosbot_utils create_config_dir <dst>` |
| Spawn another robot in sim | `ros2 launch rosbot_gazebo spawn_robot.yaml robot_model:=… namespace:=robotN x:=… y:=…` |
