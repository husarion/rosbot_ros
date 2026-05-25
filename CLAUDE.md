# CLAUDE.md — working guide for `rosbot_ros`

This file is the first one you read when starting work in this repo. It describes *how* to work here — steps, tools, rules. Code architecture and inter-package relations are in [ARCHITECTURE.md](ARCHITECTURE.md).

> **Top rule:** before you start writing code — **draft a short specification** of the change and discuss the sensitive spots. Never jump straight into implementing complex functionality.

---

## 1. Project context

- **What it is:** a set of ROS 2 (jazzy) packages for Husarion **ROSbot 2/3** and **ROSbot XL** robots (with optional OpenMANIPULATOR-X arm).
- **Distro:** ROS 2 **jazzy** (Ubuntu 24.04 Noble). Main branch: `jazzy`.
- **PR target branch:** `jazzy` (not `main`).
- **Targets:** real robot (over micro-ROS) **or** Gazebo simulation (Harmonic). Shared URDF / ros2_control for both.
- **Workspace (locally):** `~/Husarion/Workspaces/rosbot_ws/`, repo as `src/rosbot_ros`.
- **Packages in the repo:** `rosbot` (meta), `rosbot_bringup`, `rosbot_controller`, `rosbot_description`, `rosbot_gazebo`, `rosbot_hardware_interfaces`, `rosbot_joy`, `rosbot_localization`, `rosbot_moveit`, `rosbot_utils`. Per-package details — see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## 2. Specification before implementation (mandatory)

For every non-trivial task (new launch, public argument, hardware interface, controller, node, URDF change, firmware change) **first**:

1. State a one-sentence **goal** of the change ("add LED Strip car-wave for XL").
2. List the **impact** (which packages, which public arguments change, whether it breaks compatibility with firmware / snap).
3. Point out the **sensitive spots**, e.g.:
   - whether you touch `rosbot_hardware_interfaces` (hardware plugin ABI change → must be synced with firmware),
   - whether you change `controllers.yaml` (parameters with the comment "Based on real measurements" — do not overwrite without measurements),
   - whether you change `URDF`/`xacro` (collision matrix regeneration via MoveIt Setup Assistant when touching the manipulator),
   - whether you change topics/services in `ROS_API.md`.
4. Propose a **test plan** (pre-commit, `colcon test`, manual run in sim/HW).
5. Only now start writing.

In short: discuss the spec with the user before coding. Better to spend 15 min aligning than to refactor after PR review.

---

## 3. Workflow: change → build → pre-commit → test

### 3.1 First-time setup (once)

```bash
# git submodule deps (via vcstool)
cd ~/Husarion/Workspaces/rosbot_ws
vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos
vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos

# rosdep
export PIP_BREAK_SYSTEM_PACKAGES=1
sudo rosdep init   # only if not previously initialized
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# pre-commit (required for committing)
pip install pre-commit
cd src/rosbot_ros && pre-commit install
```

### 3.2 Daily loop

All commands at the workspace root: `cd ~/Husarion/Workspaces/rosbot_ws`.

```bash
# 1) source ROS and the existing overlay (if any)
source /opt/ros/jazzy/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

# 2) build a single package (preferred while iterating)
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select <pkg_name>

# 3) build a package and everything it depends on (safer after URDF/dep changes)
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to <pkg_name>

# 4) run tests for a package
colcon test --packages-select <pkg_name>
colcon test-result --verbose

# 5) source the new overlay
source install/setup.bash
```

> **Symlink-install** symlinks `launch/`, `config/`, `urdf/` and Python `scripts/` files — editing in `src/` works immediately **without** a rebuild. After C++ changes you need to rebuild.

### 3.3 Pre-commit and commit

```bash
# manually run all hooks across the whole repo
pre-commit run -a

# pre-commit also runs automatically on `git commit`
```

Hooks in `.pre-commit-config.yaml` include: `black` (line-length=99), `isort` (profile=black), `flake8` (ignore E501,W503), `clang-format`, `cmake-format`, `codespell`, `doc8` (max 100), `markdownlint-fix`, `prettier-package-xml`, `sort-package-xml`, `ament_copyright`, `yamlfmt` (mapping=2, sequence=4, offset=2, width=100, with exclusions for some launch/config yamls). Do not add new formatters without agreement.

### 3.4 Commits and PRs

- **Do not commit directly to `jazzy`.** Create a feature branch and a PR.
- Every PR has a `## Changelog description` section (see `.github/pull_request_template.md`) — a short description of the change goes there.
- Backports to other branches (e.g. `humble`) happen automatically via `backport.yaml` after the PR is labeled appropriately.

### 3.5 Running after a build

```bash
# Real robot
ros2 launch rosbot_bringup rosbot.yaml         # ROSbot
ros2 launch rosbot_bringup rosbot_xl.yaml      # ROSbot XL
ros2 launch rosbot_bringup bringup.yaml robot_model:=rosbot   # alias

# Simulation
ros2 launch rosbot_gazebo simulation.yaml robot_model:=rosbot_xl

# Arguments: see README.md ("Launch Arguments") or `ros2 launch <pkg> <file> -s`
```

---

## 4. CI / Docker / Devcontainer cheat sheet

- **Devcontainer (VS Code):** `.devcontainer/compose.yaml` mounts this repo at `/home/husarion/ros2_ws/src/rosbot_ros`. `runtime: nvidia` — requires NVIDIA Container Toolkit. Choose "Reopen in Container".
- **Production images:** `docker/Dockerfile.hardware` and `docker/Dockerfile.simulation`. Built by the `build-docker.yaml` workflow on push to `jazzy`. Hardware drops `rosbot_gazebo`, simulation drops `rosbot_bringup` before building.
- **CI (GitHub Actions, [.github/workflows/](.github/workflows/)):**
  - `integration.yaml` — orchestrator: `pre-commit` → `run-tests.yaml` → (on `jazzy`) `build-docker.yaml`.
  - `run-tests.yaml` — `colcon test --packages-skip rosbot_bringup rosbot_gazebo` (those two only run locally — they need real Gazebo / HW launch).
  - `backport.yaml` — automatic backport after merging a PR with the right label.
- **VS Code tasks (`.vscode/tasks.json`):** `build`, `debug`, `test`, `clean`, `purge`, `fix` (ament_uncrustify) — they use scripts from `~/ros2_ws/.vscode/scripts/`. **Heads-up:** those scripts are inherited from another repo (`panther`) — `build.sh` uses `--packages-up-to panther`, so the manual `colcon build` from section 3.2 is more reliable.

---

## 5. Code conventions

### Everywhere

- **Apache 2.0** header in every new source file (checked by `ament_copyright`). For the template see any existing `.py`/`.cpp` file from 2024.
- Maintainer: `support@husarion.com`. Current authors are visible in each `package.xml` (mainly Rafał Górecki, Jakub Delicat, Maciej Stępień, Krzysztof Wojciechowski, Dominik Nowak).

### Python

- `black` line-length **99**, `isort` profile=black, `flake8` ignore `E501,W503`.
- Type hints encouraged (per the user's global CLAUDE.md).
- Executable scripts go to `<pkg>/scripts/`, installed via `install(PROGRAMS ...)` in `CMakeLists.txt` to `lib/${PROJECT_NAME}` (runnable via `ros2 run <pkg> <name>`).
- Python modules (e.g. `rosbot_utils/rosbot_utils/`) installed via `ament_python_install_package`.

### C++

- `clang-format` (default ROS config), compiled with `-Wall -Wextra -Wpedantic` (+ `-Wshadow -Wold-style-cast` in `rosbot_joy`).
- C++17 minimum (`rosbot_hardware_interfaces`).
- Hardware plugins registered via `pluginlib` in `rosbot_hardware_interfaces.xml`.

### Launch

- **Prefer the YAML launch frontend** (`*.yaml`) — most launches in this repo are YAML. Use Python (`*.launch.py`) only when logic is required (example: [rosbot_bringup/launch/microros.launch.py](rosbot_bringup/launch/microros.launch.py)).
- Recurring arguments (`namespace`, `robot_model`, `config_dir`, `mecanum`, `use_sim`) — **keep the same shape** as in existing files; tests, snaps, and other Husarion packages anchor to them.
- Files in `/tmp/rosbot_*_<namespace>.{yaml,urdf}` are **resolved** versions of config/URDF — generated at runtime (sed) and read by nodes. Do not commit them.

### Configurations

- Every package that has a config accepts the `config_dir` arg. By default it uses its own `share/<pkg>/config/`. When `config_dir` is set, it reads `<config_dir>/<pkg>/config/...`. This mechanism is used by the Husarion snap — **do not break this convention**.
- Create a new external config dir: `ros2 run rosbot_utils create_config_dir <dst>`.

### Naming

- packages: `rosbot_*`
- launch: verbs/functions, e.g. `controller.yaml`, `simulation.yaml`, `microros.launch.py`
- public topics: see [ROS_API.md](ROS_API.md). **Renaming a public topic = breaking change** — coordinate with the team.

---

## 6. Hard rules (firm NO's)

1. **Do not change `FIRMWARE_VERSION`** in [rosbot_utils/firmware_version.py](rosbot_utils/rosbot_utils/firmware_version.py) without swapping `rosbot_utils/firmware/rosbot[_xl]-${FIRMWARE_VERSION}.bin` in lockstep, and vice versa. The driver verifies the MCU's reported firmware string against this constant in `configure_robot`; mismatch blocks the launch.
2. **Do not commit `build/`, `install/`, `log/`, `*.pyc`** — they are in `.gitignore`. The vcstool submodules (`husarion_components_description`, `husarion_controllers`, `husarion_gz_worlds`, `micro-ROS-Agent`, `tf_namespace_bridge`) are also ignored.
3. **Do not disable `ament_copyright`** or any other pre-commit hook without approval — CI enforces them.
4. **Do not change `controllers.yaml` parameters that carry the `Based on real measurements` comment** without fresh measurements. Those are empirically tuned values.
5. **Do not put a robot into `manipulation`/`manipulation_pro` without reading [MANIPULATOR.md](MANIPULATOR.md)** — there are hardware pitfalls (power loss → arm falls under inertia).
6. **Do not publish new topics/launch arguments without updating [ROS_API.md](ROS_API.md) and the table in [README.md](README.md).**
7. **Do not touch `tf_namespace_bridge`** outside of designated cases — it is the only path that bridges namespaced TF onto the global `/tf` (needed for nav2 / multirobot).

---

## 7. Adding new functionality

Every new (public) feature **must be written into the knowledge base**:

- [ ] update the package section in **[ARCHITECTURE.md](ARCHITECTURE.md)** (if it changes inter-package contracts or adds a new node/topic/plugin).
- [ ] add an entry in **[ROS_API.md](ROS_API.md)** (for public topics, nodes, or launch files).
- [ ] update the argument table in **[README.md](README.md)** (if you add/change a launch arg).
- [ ] if it concerns the manipulator — add it to **[MANIPULATOR.md](MANIPULATOR.md)**.
- [ ] add/update tests (`test_xacro.py`, `test_bringup.py`, `test_sim.py`).
- [ ] run `pre-commit run -a`, `colcon build`, `colcon test`.
- [ ] if the feature ships behind a flag / staged rollout — propose a `/schedule` agent for later cleanup.

A "new feature" entry template in **CLAUDE.md** (section [9](#9-decision-and-nuance-history)): one bullet, one sentence, link to the PR.

---

## 8. Sensitive spots (remember)

- **micro-ROS:** ROSbot (2/3) uses **serial** (`/dev/ttySERIAL`, baudrate 921600 — that is what `microros.launch.py` uses, even though README historically had 576000). ROSbot XL uses **udp4** on port 8888. Mode selection: arg `microros_mode` (`default`/`udp`/`serial`). Pre-communication (`configure_robot`) checks the firmware version and sets the namespace over serial before the agent starts.
- **Namespace:** a live mechanism. `ROBOT_NAMESPACE` env var → `--ros-args`. Inside the robot TF is namespaced (`/<ns>/tf`); `tf_namespace_bridge` bridges to the global `/tf`.
- **XL configuration via `configuration` arg:** `basic`, `telepresence`, `autonomy`, `manipulation`, `manipulation_pro`, `custom`. It determines the component list in `rosbot_description/config/rosbot_xl/<configuration>.yaml` and whether the manipulator launches (`manipulation*`).
- **Hardware initial state in XL `controllers.yaml`:** the `<manipulator_state>` placeholder is sed-replaced with `active`/`inactive` before `controller_manager` starts. See [rosbot_controller/launch/controller.yaml](rosbot_controller/launch/controller.yaml).
- **Sim vs HW in ros2_control:** the same URDF, different `<plugin>` in `<ros2_control>`: `rosbot_hardware_interfaces/RosbotSystem` (HW) vs `gz_ros2_control/GazeboSimSystem` (sim). See [common/ros2_control.urdf.xacro](rosbot_description/urdf/common/ros2_control.urdf.xacro).
- **EKF fuses `odometry/wheels` (from the drive controller) + `imu/data`** into `odometry/filtered`. `enable_odom_tf: false` on the drives — the `odom→base_link` TF is published by EKF.
- **Laser filter** crops points inside the robot's body. Per-model config in [rosbot_utils/config/{rosbot,rosbot_xl}/config.yaml](rosbot_utils/config/).
- **CI does not test `rosbot_bringup` or `rosbot_gazebo`** — local only. After non-trivial launch changes run them by hand.

---

## 9. Decision and nuance history

(Short entries: date + one sentence. Append fresh insights discovered during work to the bottom of this section.)

- *2026-04-30: Initial version of CLAUDE.md / ARCHITECTURE.md.*
- *2025-04-21: Added `led_strip` arg in [rosbot_xl.yaml](rosbot_bringup/launch/rosbot_xl.yaml) (commit `bbd741b`).*
- *2025-04-XX: Firmware bumped to `v1.1.0-jazzy`, improved PID (commits `b87b1a4`, `e5509b2`).*
- *2026-05-04: Exposed `frame_filters` parameter for `tf_namespace_bridge` via per-package config files in `rosbot_bringup/config/` and `rosbot_gazebo/config/` (default empty = pass-through).*
- *2026-05-15: Namespaced MoveIt topics for `joy2servo` and `servo_node`. `MoveGroupInterface` builds its `trajectory_execution_event` / `attached_collision_object` publishers via `rclcpp::names::append(opt.move_group_namespace, TOPIC)` — that is an FQN expansion that **ignores** the parent node's namespace, so the 2-arg ctor leaves these topics at `/`. Fixed by passing `this->get_namespace()` as the 3rd arg of `MoveGroupInterface::Options`. `moveit_servo`'s `monitored_planning_scene_topic` defaults to absolute `/planning_scene` (leading slash strips namespace from the PSM's private node) — fixed by setting it to relative `planning_scene` in `rosbot_moveit/config/moveit_servo.yaml`. `/parameter_events` is global by design.*
- *2026-05-15: Refactored `rosbot_moveit`. (a) `joy2servo` moved from `rosbot_joy` to `rosbot_moveit` — it is purely a MoveIt Servo client; `rosbot_joy` is now config-only (no MoveIt deps). (b) Removed dead code: `joy_servo_node` / `joy_control` / `manipulation_controller` (~1000 LOC) — abandoned richer servo node that depended on the removed `cartesian_drift_dimensions` API ([moveit_msgs#185](https://github.com/moveit/moveit_msgs/issues/185)). (c) Deduplicated `dock.cpp` / `home.cpp` (~180 LOC → ~50 LOC) into `arm_pose_mover` SHARED lib + thin executables; replaced the detached-thread executor pattern with a joinable spinner + RAII cleanup. (d) Trimmed `ompl_planning.yaml` from 192 → ~45 lines (3 planners for the manipulator + RRTConnect for the gripper). (e) Removed `gripper_right_joint` block from `joint_limits.yaml` (it is `<passive_joint>` in the SRDF). (f) Fixed `servo.launch.py`: typo `"robot_xl"` → `"rosbot_xl"` in `MoveItConfigsBuilder`, dropped unused `load_yaml` helper, made the param set symmetric with `move_group.launch.py` (same xacro override). (g) Added `test_moveit_config.py` (regression guards).*
- *2026-05-15: Manipulation HW launch leftover noise classified after the singularity-guard fix. **Accepted (cannot fix from config):** (i) `[planning_pipeline] The planner plugin did not fill out 'planner_id'` — `ompl_interface` does not stamp `MotionPlanResponse.planner_id`; jazzy-only upstream omission, fixed only on `main`. (ii) `[occupancy_map_monitor] No 3D sensor plugin(s) defined` + `Resolution not specified for Octomap` — PSM always starts the monitor even without `sensors_3d.yaml`; no flag to disable. (iii) `[controller_manager] remapping arguments deprecated` (×5) — spawner CLI still uses bare `-r` for some forwards; orthogonal to `rosbot_moveit`, leave for the rosbot_controller layer. (iv) `[ros2_control] FIFO RT scheduling: Operation not permitted` + `[servo_node] Realtime kernel recommended` — env (no RT kernel / sudo). (v) `[dynamixel_hardware_interface] transmission_to_joint_matrix not provided` + `[laser_filters] No HW_ID was set` + `[RosbotSystem] Feedback message wasn't received yet` (one-shot startup race) + `[ekf_node] Failed to meet update rate` (one-shot peak under startup load) — all upstream noise. **Fixed:** `validate_workspace_bounds` warning — the adapter substitutes from `<pipeline_ns>.default_request_adapter_parameters.default_workspace_bounds` only if the request's own `workspace_parameters` are zero, and neither nested-dict-on-Node nor a yaml override under that key actually populates the param at the right move_group internal namespace in jazzy. The reliable fix is to set the workspace on the **client** side: `MoveGroupInterface::setWorkspace(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5)` in [arm_pose_mover.cpp](rosbot_moveit/src/arm_pose_mover.cpp) and [joy2servo.cpp](rosbot_moveit/src/joy2servo.cpp) (covers `dock`/`home` + `RT+Back`/`RT+Start` + analogue gripper moves). 1 m cube comfortably contains the OpenManipulator-X reach (~0.4 m).*
- *2026-05-15: Cartesian arm control rerouted around moveit_servo's POSE path. The threshold workaround for `velocityScalingFactorForSingularity` (`moveit_servo/src/utils/common.cpp`) is not sufficient on real hardware: σ_min from `JacobiSVD` on the OpenManipulator-X 6×4 Jacobian goes below ~1e-15 at non-singular EE poses, so cond ≈ 1e15 and `HALT_FOR_SINGULARITY` still fires after KDL with `position_only_ik` solves the goal. There is no config switch to disable the guard or point it at the position-only sub-Jacobian. We sidestep the path entirely: `joy2servo` runs IK itself (`RobotState::setFromIK` with the KDL plugin loaded from `kinematics.yaml`) and publishes the resulting joint velocities as **JointJog** to servo — the JointJog handler does not run the singularity guard. UI keeps the X/Y toggle but both modes now publish to `servo_node/delta_joint_cmds`; nothing publishes to `servo_node/pose_target_cmds` from us. Drops the `switch_command_type` service client and the `pose_target_cmds` publisher; drops `tf2_ros` (FK done via `RobotState::getGlobalLinkTransform`).*
- *2026-05-15: Cartesian control for the 4-DoF arm (servo POSE + `position_only_ik`). joy2servo's `Y` button now toggles into POSE mode: it integrates joystick "linear velocity" (param `pose_linear_velocity`, default 0.1 m/s) into a `PoseStamped` published on `servo_node/pose_target_cmds`, with `planning_frame` defaulting to `end_effector_link` so motion feels EE-relative. **Why this works:** servo's `processPoseCommand` → `jointDeltaFromIK` → `ik_solver->searchPositionIK(...)` — it goes through the configured KDL plugin, and KDL with `position_only_ik: true` zero-weights the three orientation rows of the IK task (`moveit_kinematics/kdl_kinematics_plugin.cpp`). There is no hard 6-DoF assertion in the POSE path (unlike TWIST, which still falls through to the pseudo-inverse Jacobian — see [moveit_msgs#185](https://github.com/moveit/moveit_msgs/issues/185)). TWIST code path is removed from joy2servo. The `X` button restores JOINT_JOG.*
- *2026-05-15: Sim-warning cleanup for `ros2 launch rosbot_gazebo simulation.yaml robot_model:=rosbot_xl configuration:=manipulation`. (a) `manipulator_controller` no longer uses deprecated `open_loop_control: true` — replaced with `interpolate_from_desired_state: true` + per-joint `gains.{p,i,d}=0` ([controllers.yaml](rosbot_controller/config/rosbot_xl/controllers.yaml)); identical trajectory behavior in sim (Home/Open SUCCEEDED), HW retest pending. (b) MoveIt parameters: `default_workspace_bounds: 1.0` on move_group ([move_group.launch.py](rosbot_moveit/launch/move_group.launch.py)) + `default_planner_config: RRTConnectkConfigDefault` per group ([ompl_planning.yaml](rosbot_moveit/config/ompl_planning.yaml)). (c) `joy2servo` and `home` now receive `robot_description_kinematics`/`semantic`/`joint_limits` — params added to [servo.launch.py](rosbot_moveit/launch/servo.launch.py), and `home` was extracted into [home.launch.py](rosbot_moveit/launch/home.launch.py) so the wrapper can pass MoveIt config via `MoveItConfigsBuilder`. (d) Controller-prefixed remaps removed from [gazebo.urdf.xacro](rosbot_description/urdf/common/gazebo.urdf.xacro) (already covered by `--controller-ros-args` on the spawner); `/tf`, `/tf_static`, `/diagnostics` stay because `controller_manager` is instantiated by `gz_ros2_control` inside the Gazebo process — outside the launch group that hosts `set_remap: /tf -> tf` in `spawn_robot.yaml`, so URDF `<remapping>` is the only way to namespace those absolute topics for it. `tf_namespace_bridge` itself does **not** apply `/tf:=tf` (it is launched before `set_remap` in the group, so it stays on the global topic to act as the bridge between `<ns>/tf` and `/tf`). (e) Known residue: `OccupancyMapMonitor` always-on inside `PlanningSceneMonitor` (no launch-level switch); `gripper_controller` keeps `position_controllers/GripperActionController` because `parallel_gripper_action_controller` exposes `control_msgs/action/ParallelGripperCommand` which MoveIt's `moveit_simple_controller_manager` cannot drive — both deprecation messages are accepted.*
- *2026-05-18: Deduplicated FIRMWARE_VERSION constant — single source in `rosbot_utils.firmware_version`.*
- *2026-05-19: Attempted to fix the HW `gripper_right_joint`-missing-from-`/joint_states` bug via a local override of upstream `open_manipulator_x_position.ros2_control.xacro` (declaring the mimic joint unconditionally so ros2_control's URDF auto-mimic — PR [ros-controls/ros2_control#1256](https://github.com/ros-controls/ros2_control/pull/1256) — would pick it up). **Reverted on HW retest:** `dynamixel_hardware_interface/DynamixelHardware` counts every `<joint>` inside its `<ros2_control>` block against `number_of_joints` (set to `5` in the upstream xacro), regardless of whether the joint has a `<command_interface>` or carries `<param name="mimic">`. Adding the joint trips `Error: number of joints 5, 6, 6` and `Failed to initialize hardware 'OpenManipulatorXSystem'`, so controller_manager never loads anything. Putting `gripper_right_joint` in a separate `<ros2_control>` block does not help either — ros2_control's component parser searches for the mimic target only inside the current block (`Mimic joint '...' not found in <ros2_control> tag`). The proper fix needs upstream: either (a) Robotis/ROBOTIS-GIT removes the `<xacro:if value="$(arg use_sim)">` guard in `open_manipulator_description` AND bumps the dynamixel plugin's joint-count check to skip mimic joints, or (b) ros2_control adds an auto-mimic path for URDF mimic joints absent from `<ros2_control>`. Until then, the right finger looks visually stuck on HW — cosmetic, robot operations are unaffected.*
- *2026-05-19: Added [dock.launch.py](rosbot_moveit/launch/dock.launch.py) mirroring [home.launch.py](rosbot_moveit/launch/home.launch.py). Both wrappers inject `robot_description_semantic` + `robot_description_kinematics` + `joint_limits` via `MoveItConfigsBuilder` so the `MoveGroupInterface` inside `dock`/`home` no longer logs `No kinematics plugins defined` (the executables load their own `RobotModelLoader` which needs the kinematics param at the node level — passing it through the launch wrapper sidesteps the warning). `dock.launch.py` also exposes a `namespace:=` arg (default empty) so the standalone invocation is `ros2 launch rosbot_moveit dock.launch.py namespace:=<ns>` instead of `ros2 run rosbot_moveit dock --ros-args -r __ns:=/<ns>`; `home.launch.py` keeps no namespace arg (auto-included from `manipulator.yaml` which already pushes the namespace — declaring it here would double-stack). MANIPULATOR.md updated.*
- *2026-05-19: Dropped the duplicated `gripper_group_->move(); gripper_group_->move();` / `manipulator_group_->move(); manipulator_group_->move();` in `joy2servo`'s `MoveToDockPose` and `MoveToHomePose` (commented "To make sure the action is finished", inherited from `cf9186b` in 2025-12). HW retest on a real ROSbot XL (`ROBOT_NAMESPACE=rosbot_xl configuration:=manipulation`) confirmed: `MoveGroupInterface::move()` forwards to `MoveGroupInterfaceImpl::move(wait=true)`, which loops on the action result callback (`async_send_goal` + done flag); the result fires only after `trajectory_execution_manager::waitForExecution()` returns; that returns when the controller reports completion. One `move()` per group already blocks until the arm physically reaches the named target -- faked `/joy` with `RT+Back` produced exactly one "Plan and Execute request accepted" per group and joy2servo's `JoyMutex::try_lock` held across the entire ~2 s motion. The `[INFO] Not waiting for trajectory completion` log message persists because `trajectory_execution.allowed_start_tolerance: 0.0` in [move_group.launch.py](rosbot_moveit/launch/move_group.launch.py) short-circuits `waitForRobotToStop()` (the post-controller verification). The controller's own waitForExecution still blocks on physical completion, so the log is cosmetic. Tuning `allowed_start_tolerance` is a separate experiment (raising it would re-enable the verification but requires verifying no controller stalls within tolerance band).*
- *2026-05-19: `rviz.launch.py` namespace-aware. Added `namespace:=` arg (default empty), `/tf -> tf` / `/tf_static -> tf_static` / `/diagnostics -> diagnostics` remaps to match every other namespaced node in the bringup group, and a sed-resolved copy of `moveit.rviz` written to `/tmp/rosbot_moveit_<ns>.rviz` (substitutes `Move Group Namespace: ""` with `"/<ns>"`). Without the substitution RViz's MotionPlanning panel ignores the launch's `__ns:=` and constructs MGI on `/` instead of `/rosbot_xl` (the panel reads the namespace from the rviz config, not from the node's ROS namespace). Pattern mirrors `rosbot_controller/launch/controller.yaml`'s resolved-config dance. HW-verified: MotionPlanning panel reaches `/rosbot_xl/move_group`, planning groups list both `manipulator` and `gripper`, Plan + Execute works on the real arm, TF tree connects without warnings.*
- *2026-05-19: Gripper smoothness fix — `gripper_controller` switched from `position_controllers/GripperActionController` to `joint_trajectory_controller/JointTrajectoryController` (matches `manipulator_controller`), with `moveit_controllers.yaml` now using `FollowJointTrajectory`/`follow_joint_trajectory` for the gripper. **Why:** the old controller accepts a single position setpoint per action goal and cancels-and-restarts on every new goal, so `ControlGripper`'s 20 Hz `MGI::move()` spam caused stair-step motion + Dynamixel `Profile Velocity` profile restarts. JTC interpolates at controller_manager update rate (100 Hz) and lets us drop MGI entirely for the joystick hot path: `joy2servo`'s `ControlGripper` now publishes single-point `trajectory_msgs/JointTrajectory` (`time_from_start = 100 ms`) directly to `gripper_controller/joint_trajectory`. Named-pose moves (`MoveToDockPose`/`MoveToHomePose` from `dock`/`home`/joy2servo) keep `MGI::setNamedTarget + move()`; MoveIt forwards the trajectory via `FollowJointTrajectory` (same as the manipulator). **Breaking:** `gripper_controller/gripper_cmd` action is gone; any external user (snap, custom node) that drove it must switch to `gripper_controller/follow_joint_trajectory`. **Configs:** `gripper_left_joint` limits in `joint_limits.yaml` converted from rad/s to physical m/s; `max_velocity: 0.08 m/s` validated on HW (peak 0.056 m/s under 0.05 cap, bumped to 0.08 per "ciut wyżej" HW feel test); `max_acceleration: 0.5 m/s²` still TODO (HW peak 1.88 m/s² observed). Regression test in `test_moveit_config.py` (`test_gripper_uses_follow_joint_trajectory`) guards the moveit_controllers entry. Closes Q1 in FOLLOWUP_PLAN.md. The `parallel_gripper_action_controller` route is still blocked by MoveIt's `moveit_simple_controller_manager` not supporting `ParallelGripperCommand` (per the 2026-05-15 entry above) — JTC sidesteps that.*
- *2026-05-19→20: **Correction to c287e65** ("joy2servo: drop redundant second `move()`"). HW retest exposed a real regression: with arm at Dock, the first `RT+Start` (`MoveToHomePose`) moves the arm only ~60% of the way to Home (distance 1.26 → 0.50 rad, JTC reports SUCCEEDED); the second press completes it. Isolation testing: a fresh `rclpy.ActionClient<MoveGroup>` does NOT reproduce — joy2servo's long-lived MGI does. Two fix attempts on 2026-05-19/20 both failed: (a) `setStartStateToCurrentState()` before each `setNamedTarget` — still observed regression; (b) explicit seed from joy2servo's own `latest_joint_state_` cache (which Cartesian IK relies on and IS fresh) — HW test showed `MoveToDockPose`'s manipulator move() not moving the arm at all on first press, and gripper sometimes also stuck. The staleness lives deeper in MGI/PSM than expected — possibly the planning scene monitor on the **server** side (move_group), not the client. **Reverted** to the c287e65 band-aid (`manipulator_group_->move()` ×2 in `MoveToDockPose`/`MoveToHomePose`) until the root cause is properly understood. The c287e65 HW test only covered `RT+Back` from Home (where stale="near-Home" matches reality → bug invisible). The case that triggers it is `RT+Start` from Dock (or any non-Home settled pose). FOLLOWUP_PLAN.md C4 reopened with updated empirical data.*
- *2026-05-20: Cartesian-mode self-collision regression: joy2servo's `velocities[i] = (target - current) / cartesian_step_dt` produced multi-rad/s spikes when KDL `position_only_ik` jumped between IK branches (the 1-DoF null space on the 4-DoF arm), so the arm overshot `self_collision_proximity_threshold` inside a single `collision_check_rate` tick (100 ms @ 10 Hz). **Fixed in joy2servo by uniform clamp:** new ROS param `cartesian_max_joint_velocity` (rad/s, default `1.0`); if any joint exceeds it, scale the whole velocity vector by `cap / max|v_i|` — preserves trajectory direction, just slower. **Servo collision config:** `collision_check_rate: 10 → 30 Hz` (33 ms tick), `self_collision_proximity_threshold` kept at the upstream default `0.02 m`. Iteration on HW: 0.05 m halted both modes on startup (at typical resting poses some non-SRDF-disabled link pairs sit inside 5 cm and `collision_velocity_scale_` clamped to ~0; `servo_node/status` reported `DECELERATE_FOR_COLLISION`). The deceleration ramp at 0.02 m + 30 Hz + 1 rad/s clamp still leaves the arm able to contact the body cover (linear scaling converges geometrically but never zeros inside the threshold window) — accepted as a known limitation; raising the threshold further would need the SRDF `Never`-set regenerated first because rest-pose distances on `link4`/`link5`/gripper vs body/wheels/rplidar sit close to 0.03-0.05 m. **Log filter:** [servo.launch.py](rosbot_moveit/launch/servo.launch.py) drops `joy2servo`'s default log level to WARN and whitelists the `joy2servo` logger back to INFO, to silence `moveit_kinematics/kdl_kinematics_plugin`'s "Using position only ik" spam (fires every IK call, 20-50 Hz in Cartesian mode). Per-logger filter on `moveit.kinematics.kdl_kinematics_plugin` does not work in jazzy because MoveIt attaches plugin loggers under an auto-named internal node, so the registered prefix is unpredictable. Reset singularity thresholds to upstream defaults (`lower: 17`, `hard_stop: 30`) — the inflated 1e+12/1e+13 was a defensive fallback for the POSE/TWIST handlers, but Cartesian goes through JointJog which skips the guard entirely. Removed `test_singularity_thresholds_effectively_disabled`. **Out-of-scope (SRDF):** `link2`/`link3` vs `body_link`/`rl_wheel_link`/`rr_wheel_link`/`rplidar_link` carry `reason="Never"` in [rosbot_xl.srdf](rosbot_moveit/config/rosbot_xl.srdf:75-76,106-108,110-112) — those pairs are not collision-checked at all regardless of `self_collision_proximity_threshold`. User confirmed the SRDF is OK as-is on 2026-05-20.*
- *2026-05-21: Added MAVLink as the default `backend:=mavlink|microros` arg on `rosbot[_xl].yaml` (paired with `rosbot_mavlink_bridge`). Renamed `microros` bool → `hardware_bridge`.*
- *2026-05-22: Runtime-switch firmware handshake — `configure_robot --backend` emits a new `BACKEND:` line before `NS:`; missing ACK is fatal so legacy single-protocol firmware fails loud rather than silently defaulting the namespace. `link_layer` renamed to `backend` to match firmware-side `CommBackend` / `--backend` (breaking, no alias).*

---

## 10. Quick command reference

| what | command |
|---|---|
| Build a single package | `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <pkg>` |
| Build with dependents | `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to <pkg>` |
| Test a single package | `colcon test --packages-select <pkg> && colcon test-result --verbose` |
| Pre-commit (whole repo) | `pre-commit run -a` |
| List launch arguments | `ros2 launch <pkg> <file> -s` |
| List topics | `ros2 topic list` |
| Flash firmware | `ros2 run rosbot_utils flash_firmware --robot-model rosbot[\|_xl]` |
| Activate the manipulator | `ros2 run rosbot_controller arm_control active` |
| Generate an external config dir | `ros2 run rosbot_utils create_config_dir <dst>` |
| Spawn another robot in sim | `ros2 launch rosbot_gazebo spawn_robot.yaml robot_model:=... namespace:=robotN x:=... y:=...` |
