# Husarion rosbot_ros dev helpers — `just <recipe>` from this repo root.
# Requires `just` (apt / snap / cargo install just).
#
# Workspace auto-detect:
#   - In `.claude/worktrees/<name>/` (a git worktree): `/tmp/<name>_ws`,
#     populated with symlinks to the worktree's packages. Lets you work
#     on a feature branch without contaminating the main checkout.
#   - Otherwise (main checkout):  `~/Husarion/Workspaces/rosbot_ws`.
# Override with `WS=/path/to/ws just <recipe>`.

set shell := ["bash", "-c"]
set quiet := true

ws := env_var_or_default("WS", `
    if pwd | grep -q "/.claude/worktrees/"; then
        echo "/tmp/$(basename "$(pwd)")_ws"
    else
        echo "$HOME/Husarion/Workspaces/rosbot_ws"
    fi
`)

# Default recipe — show available recipes.
default:
    @just --list

# Ensure {{ws}}/src has everything `colcon build` needs:
#   1. symlinks to every rosbot_* / rosbot package present in the current
#      directory (so a feature branch's edits flow into the worktree build
#      without copying);
#   2. vcstool-imported sibling repos (`husarion_components_description`,
#      `husarion_controllers`, `tf_namespace_bridge`, `micro-ROS-Agent`) —
#      same flow as README §"First-time setup", but scoped to {{ws}}/src.
# Both steps are idempotent.
_setup-ws:
    set -eo pipefail; \
    mkdir -p {{ws}}/src; \
    for p in rosbot rosbot_bringup rosbot_controller rosbot_description \
             rosbot_gazebo rosbot_hardware_interfaces rosbot_joy \
             rosbot_localization rosbot_moveit rosbot_utils; do \
        if [ -d "$(pwd)/$p" ] && [ ! -e "{{ws}}/src/$p" ]; then \
            ln -sfn "$(pwd)/$p" "{{ws}}/src/$p"; \
        fi; \
    done; \
    if [ ! -d "{{ws}}/src/husarion_components_description" ]; then \
        echo "[setup] importing sibling repos via vcstool (one-time)…"; \
        vcs import "{{ws}}/src" < "$(pwd)/rosbot/rosbot_hardware.repos"; \
        vcs import "{{ws}}/src" < "$(pwd)/rosbot/rosbot_simulation.repos"; \
    fi

# Build a package and everything it depends on (default: rosbot meta = whole repo).
build pkg='rosbot': _setup-ws
    set -eo pipefail; \
    source /opt/ros/jazzy/setup.bash; \
    cd {{ws}}; \
    colcon build --symlink-install --packages-up-to {{pkg}} \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run colcon test on one package.
test pkg='rosbot': _setup-ws
    set -eo pipefail; \
    source /opt/ros/jazzy/setup.bash; \
    cd {{ws}}; \
    [ -f install/setup.bash ] && source install/setup.bash || true; \
    colcon test --packages-select {{pkg}}; \
    colcon test-result --test-result-base build/{{pkg}} --verbose

# pre-commit run --all-files — black, isort, flake8, codespell, prettier, ...
precommit:
    pre-commit run --all-files

# Launch bringup on real hardware (auto-builds rosbot_bringup first).
# `just hw`                       → rosbot_xl basic
# `just hw model=rosbot`          → rosbot 2/3 basic
# `just hw config=manipulation`   → rosbot_xl with manipulator
hw model='rosbot_xl' config='basic': (build "rosbot_bringup")
    set -eo pipefail; \
    source /opt/ros/jazzy/setup.bash; \
    source {{ws}}/install/setup.bash; \
    LAUNCH_FILE=$([ "{{model}}" = "rosbot" ] && echo "rosbot.yaml" || echo "rosbot_xl.yaml"); \
    exec ros2 launch rosbot_bringup "$LAUNCH_FILE" configuration:={{config}}

# Launch Gazebo simulation (auto-builds rosbot_gazebo first).
# `just sim`                      → rosbot_xl
# `just sim model=rosbot`         → rosbot 2/3
sim model='rosbot_xl': (build "rosbot_gazebo")
    set -eo pipefail; \
    source /opt/ros/jazzy/setup.bash; \
    source {{ws}}/install/setup.bash; \
    if [ -e /usr/share/glvnd/egl_vendor.d/10_nvidia.json ] && command -v nvidia-smi >/dev/null 2>&1; then \
      export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json; \
    fi; \
    exec ros2 launch rosbot_gazebo simulation.yaml robot_model:={{model}}

# Wipe build + install artefacts for a single package.
clean pkg:
    rm -rf {{ws}}/build/{{pkg}} {{ws}}/install/{{pkg}}

# Show the resolved workspace path.
where:
    echo "workspace: {{ws}}"
