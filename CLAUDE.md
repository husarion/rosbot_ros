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

1. **Do not change the firmware version** (`v1.1.0-jazzy` for micro-ROS, `v0.1.1-jazzy-mavlink` for MAVLink) in `flash_firmware`/`configure_robot`/`mavlink.launch.py` without swapping the matching binary in `rosbot_utils/firmware/`, and vice versa. The driver and the firmware are tightly coupled per track.
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
- *2026-05-21: Added MAVLink as the default `backend:=mavlink|microros` arg on `rosbot[_xl].yaml` (paired with `rosbot_mavlink_bridge`). Renamed `microros` bool → `hardware_bridge`. `configure_robot` gained `--expected-firmware` for per-track version pinning. `flash_firmware` gained `--variant micro-ros|mavlink` (default `mavlink`). Added `just release` + tag-driven workflow — bare-suffix `X.Y.Z` tags belong on `jazzy`.*
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
