# SPEC — ROSbot LED animation publisher

Status: **implemented + validated on-host; pending on-robot LED test** (2026-07-05). Code landed in `rosbot_utils` (`src/animation_publisher.cpp`, `third_party/stb_image.h`, `animations/`), `rosbot_bringup/launch/rosbot_xl.yaml`, `rosbot_utils/test/test_led_strip.py`, and the `rosbot-snap` `driver.current-animation` wiring. Validated on a ROS Jazzy host: `colcon build --packages-select rosbot_utils` green; the node loads all 3 animations and publishes; `test_led_strip.py` 2/2 pass (frame contract + `current_animation` accept/reject/`none`-stops). Remaining: snap rebuild/republish (also carries agent 1.5.1) + an on-robot run to see the physical strip. Delete this SPEC once that's done and migrate any durable notes into `ARCHITECTURE.md` / the READMEs.

A parameter-selectable LED-animation node in `rosbot_ros` that publishes patterns to the ROSbot's LED strip by itself, so operators no longer hand-publish frames (or run a dev-only blanker) to turn the built-in firmware animation off. Patterns are user-definable PNG files (row = frame), discovered from a directory, and selected via `sudo snap set rosbot driver.current-animation=<name>` and/or a live ROS topic.

## Decisions locked (interview, 2026-07-05)

- **Authoring model:** full PNG registry (Panther/Lynx-style) — every animation is a PNG, including `turn-off-lights` (an `18×1` all-black PNG). `none` is the one reserved non-file state (publish nothing).
- **Metadata layout:** **sidecar YAML per PNG** — `<name>.png` + `<name>.yaml`; filename stem = animation name.
- **List management:** directory-driven auto-discovery — the valid set is derived by globbing the animation dirs (shipped + user), not a hard-coded enum. A user-added PNG becomes selectable with no snap edit.
- **Selection surface:** snap param `driver.current-animation` (persistent boot default) **plus** a validated ROS **service** `~/set_animation` (UGV-style) for live switching.
- **Live-switch persistence:** transient — a live switch is in-memory only; on any daemon restart the node reverts to the snap param. The snap param is the single source of truth.
- **`led_strip/enable` service:** superseded — `set_animation` with `none` is the only "go dark/idle" path.
- **Per-animation config:** frequency (Hz) + brightness (0..1) + optional solid-color tint.
- **Shipped defaults:** `turn-off-lights`, `rainbow`, `car_wave` (rainbow/car_wave pre-rendered from today's procedural nodes).
- **Default value:** `car_wave` — preserves today's out-of-box XL behavior; the feature is purely additive.

## The hardware interface (do not re-research — verified in firmware 2026-07-05)

| Fact | Value | Source |
| -- | -- | -- |
| Topic | `led_strip` (relative → `/<ns>/led_strip`) | `rosbot-firmware/src/rosbot_xl/ros.cpp:141`; both example nodes |
| Message | `sensor_msgs/msg/Image` | firmware subscriber `ros.cpp:140` |
| Encoding | **`rgb8`** (3×uint8 R,G,B; no alpha) — firmware rejects anything else | `ros.cpp:123`, `bridge_node.cpp:468` |
| Shape | `height=1`, `width ≤ 18`, `step = width*3` | `ros.cpp:99-115` |
| Max LEDs | **18** (`MAX_NUM_LEDS`); width>18 clamped | `led_strip.hpp:23` |
| QoS | **BEST_EFFORT**, KeepLast(1) — RELIABLE is QoS-incompatible, frames dropped | `ros.cpp:143` |
| Idle timeout | **1000 ms** — no frame for >1 s → MCU idle animation (gray/red sweep) resumes | `config.hpp:202`, `rtos.cpp:132-170` |
| "Off" | **No off opcode.** To keep the strip dark, stream all-zero frames continuously (>1 Hz) | `rtos.cpp` idle loop |
| MCU consumer | depth-1 overwrite queue, 30 ms task → publishing >~33 Hz just overwrites | `rtos.cpp:45,60` |
| Physical remap | firmware swaps LED indices {13,17} and {14,16} before display | `config.hpp:213-216` |
| Scope | **rosbot-xl only.** Plain rosbot has no strip; MAVLink bridge `enable_led_strip: true` for XL, `false` for rosbot | `bridge/.../config/rosbot_xl.yaml:10`, `rosbot.yaml:9` |

**Consequence:** the only special state is **`none`** = publish nothing → firmware idle plays (today's behavior). Everything else is an ordinary registry animation. `turn-off-lights` is NOT special-cased — it is just a shipped animation whose PNG is a single (or few) all-black `18×1` row; publishing it continuously at its frequency is what suppresses the idle animation. A node that merely stops publishing does **not** turn the strip off — you must keep streaming the black frames.

## Prior art to generalize (≈80% done)

`rosbot_utils` already ships two nodes that publish the exact `1×18 rgb8` frame at 25 Hz with a `led_strip/enable` SetBool service:

- `rosbot_utils/src/led_strip_rainbow.cpp` — procedural HSV scroll (built, not launched).
- `rosbot_utils/src/led_strip_car_wave.cpp` — red/white sweep; **launched** on XL via `rosbot_bringup/launch/rosbot_xl.yaml:115-118`, gated by the `led_strip` launch arg (from `driver.led-strip`).
- Contract test: `rosbot_utils/test/test_led_strip.py` (asserts 1×18 rgb8 BEST_EFFORT + the enable service).

The new `animation_publisher` node replaces/absorbs these two: same publish contract, but frames come from the registry instead of a hard-coded generator.

## Component design

### 1. Registry + PNG format

**Sidecar-per-PNG** (interview): each animation is a `<name>.png` + `<name>.yaml` pair in an animation dir. The **filename stem is the animation name** (== the snap value / topic string; kebab-case). Auto-discovery globs `*.png` across two roots, merged (a user file with the same name overrides shipped):

- Shipped defaults: `rosbot_utils/animations/` (installed to `share/rosbot_utils/animations/`).
- User dir: `$SNAP_COMMON/animations/` (writable, **survives `snap refresh`** — do NOT use `config_dir`, which is overwritten on update).

Sidecar schema (`<name>.yaml`, next to `<name>.png`):

```yaml
frequency: 25.0     # Hz: publish rate AND frame-advance rate. keep >1 to hold the strip (idle after 1 s gap).
brightness: 1.0     # 0..1 scale applied after decode.
# color: "#RRGGBB"  # optional: grayscale the PNG then tint (UGV-style). omit for a full-color PNG.
```

- A `<name>.png` with no sidecar loads with defaults (`frequency: 25`, `brightness: 1.0`, no tint) and a warning. **[open: missing-sidecar = defaults+warn (proposed) vs skip]**
- **Shipped defaults** (interview): `turn-off-lights` (black), `rainbow`, `car_wave`. `rainbow`/`car_wave` are procedural today (`led_strip_rainbow.cpp`/`led_strip_car_wave.cpp`) → **pre-render them to PNGs** (hand-author, or generate at build time from the existing algorithms) so today's launched default (`car_wave`) is preserved through the migration.
- `turn-off-lights` ships as `turn-off-lights.png` (an `18×1` all-black row) + sidecar (a low `frequency`, e.g. 2 Hz, is enough to hold it dark and spares the MAVLink/serial link).
- **`none`** is reserved — never a file. Selecting it stops publishing.
- PNG semantics: **row = frame, column = LED**, top row = frame 0. Loops forever. A 1-row PNG is a static pattern (still published continuously at `frequency`).
- Width handling (interview): **crop** — use the first `led_count` (18) columns of each row; a wider PNG's extra columns are dropped, a narrower PNG's missing LEDs are padded black. No resampling (1 px = 1 LED, predictable).
- Brightness scales the decoded RGB; `color` (if set) grayscales then tints, matching UGV's `ImageAnimation` pipeline.

### 2. `animation_publisher` node (package `rosbot_utils`)

- Params: `led_count` (int, default 18), `registry_paths` (list; shipped + user), **`current_animation`** (string, default `car_wave`) — the single knob for both boot-default and live switching.
- On start: load + merge registries, decode each PNG to an in-memory sequence of `led_count`-wide RGB frames (apply brightness + tint at load).
- Publisher: `led_strip`, `sensor_msgs/msg/Image`, BEST_EFFORT KeepLast(1). Wall timer at the **active** animation's `frequency`; each tick emits the next row (looping) as `height=1, width=led_count, encoding="rgb8", step=led_count*3`. `none` → timer stopped, nothing published.
- Selection = the **`current_animation` node parameter** (no custom srv, no interfaces package — `rosbot_ros` has none today). An **on-set-parameter callback** validates the requested name against the loaded registry (or `none`) and returns `SetParametersResult{successful=false, reason=…}` for an unknown name, so `ros2 param set /animation_publisher current_animation rainbow` is a validated request with feedback. Applying a valid value swaps the active animation live. **Transient:** the parameter is not persisted; on any daemon restart the launch arg re-seeds it from the snap value (the snap param is the single persistent source of truth).
- The old `led_strip/enable` SetBool is **superseded** — `current_animation=none` is the only "go dark/idle" path; `test_led_strip.py` drops the enable-service test and asserts the parameter path + frame contract instead.
- PNG decode: **[open: `stb_image` (single-header, no heavy dep — recommended) vs Boost.GIL (UGV parity) vs OpenCV/cv_bridge]**. Current node deps are only `rclcpp sensor_msgs std_srvs`; a PNG lib must be added in `CMakeLists.txt`.
- Reuse the firmware remap note only if a specific physical LED must be addressed; for whole-strip patterns the remap is transparent.

### 3. `rosbot_ros` launch (`rosbot_bringup/launch/rosbot_xl.yaml`)

- Add a `led_animation` launch arg (default fed from the snap `current_animation`).
- Swap the `led_strip_car_wave` node (lines 115-118) for `animation_publisher`, keeping the `if: $(eval '"$(var led_strip)" == "True"')` gate; pass `led_animation` as the node's `current_animation` param (param-block syntax as used by `tf_namespace_bridge` at `rosbot_xl.yaml:70-80`).
- Keep `rosbot.yaml` (plain rosbot) LED-free.

### 4. `rosbot-snap` wiring (repo `husarion-cockpit-deps/rosbot-snap`)

Mirror the existing `driver.led-strip` / `driver.backend` patterns:

| File | Change |
| -- | -- |
| `snap/hooks/install` | `snapctl set driver.current-animation=car_wave` |
| `snap/hooks/post-refresh` | re-seed guard for the same key |
| `snap/hooks/configure` | add `current-animation` to `VALID_DRIVER_KEYS`; **dynamic** validation: `yq` the merged registry (shipped `$SNAP/...` + `$SNAP_COMMON/animations/`) into the allowed set + `none`, then validate (snap-common ships `yq`; realizes the directory-driven decision) |
| `snap/local/daemon_start.sh` | `LAUNCH_ARGS+="led_animation:=$(snapctl get driver.current-animation) "` (near line 88) |
| `snapcraft_template.yaml.jinja2` | add `driver.current-animation` bullet in the `description:` list; regenerate `snap/snapcraft.yaml` via `render_template.py` |
| `README.md` | add row to the `driver.*` table |
| `snap/husarion-agent-extras/config-seed/manifests/drive.yaml` | *(optional)* add `CURRENT_ANIMATION` — a **dynamic enum** (husarion-agent manifests support `dynamic:` from a dir/glob) so the cockpit Manage dropdown auto-lists animations |
| `snap/husarion-agent-extras/config-seed/hooks/drive` | *(optional)* `set_drv current-animation "${HUSARION_AGENT_CURRENT_ANIMATION:-}"` |

Notes: the configure hook restarts `rosbot.daemon` on every `snap set`, so the param takes effect on the next launch with no extra wiring; `driver.*` is local to the rosbot snap (not chained to rplidar/depthai). A user dir under `$SNAP_COMMON` needs the snap to create it on install and the node + hook to read it.

## Open items (proposed resolutions — confirm or override)

Remaining are implementation-leaning; each has a proposed default so implementation isn't blocked:

1. **PNG decode dependency:** **`stb_image`** (vendored single-header, permissive, no system dep) — *decided*. Vendor `stb_image.h` into `rosbot_utils` (e.g. `rosbot_utils/third_party/`).
2. **Width mismatch:** if PNG width == `led_count` use as-is (1 px = 1 LED, pixel-exact); otherwise resample columns to `led_count` with a warning — *proposed* (serves both precise and casual authors).
3. **Missing sidecar:** load the PNG with defaults (`frequency 25`, `brightness 1.0`, no tint) + a warning — *proposed* (vs skip the animation).
4. **Non-XL ROSbot:** the `driver.current-animation` param validates but is a no-op (the animation node is only launched on XL, where the strip exists) — *proposed* (vs hide/reject on plain rosbot).
5. **User dir discovery, non-snap runs:** `$SNAP_COMMON/animations/` under the snap; for a bare `rosbot_ros` run, a `registry_paths`/`user_animations_dir` launch arg (default just the shipped dir). *Confirm the arg name.*

Resolved in interview: authoring model, sidecar layout, auto-discovery, validated **`current_animation` node parameter** (no new interfaces package) + transient persistence, `led_strip/enable` superseded, shipped set, default `car_wave`, freq+brightness+tint, `stb_image` decode.

## Cross-repo touch points

- `rosbot_ros` (this repo): the node, launch, tests, shipped animations. **Primary.**
- `husarion-cockpit-deps/rosbot-snap`: the `driver.current-animation` param + dynamic validation + optional Manage manifest.
- `husarion-cockpit` (cockpit): only if the Manage dynamic-enum dropdown is wired — otherwise no change.
