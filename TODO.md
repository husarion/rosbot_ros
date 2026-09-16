# TODO — rosbot_ros

Known work items that are understood but not yet done. Each entry says what is
wrong, what has to change, and — where the fix reaches beyond this repo — which
other repos move with it.

## Accept a leading `/` (and nested `a/b`) in `namespace:`

**Status:** deferred by decision, 2026-09-16. Not a regression — it has never
worked. Raised while trying to loosen the Cockpit's `ROS_NAMESPACE` validation,
which is blocked on this.

**What works already.** ROS 2 itself handles every spelling. Verified on Jazzy
with `PushRosNamespace`: `/abs_ns`, `rel_ns`, `a/b` and `/x/` all resolve to
correct node and topic names, with no `//` and no warning — the launch API adds
the leading slash, trims a trailing one, and nests fine. `Node(namespace=…)`
behaves the same. (`--ros-args -r __ns:=` is stricter and needs a fully
qualified name, but nothing here uses that form.)

**What breaks.** Places in this repo that build strings out of `namespace` by
hand, where a `/` inside the value lands in the middle of a path or doubles a
separator.

Hard failures — the namespace goes into a `/tmp` filename, so a `/` points at a
directory that does not exist and the write fails (`No such file or directory`),
taking bringup with it:

| File | Line | Expression |
| -- | -- | -- |
| `rosbot_description/launch/rosbot.yaml` | 73 | `/tmp/rosbot_description_$(var namespace).urdf` |
| `rosbot_description/launch/rosbot_xl.yaml` | 81 | `/tmp/rosbot_xl_description_$(var namespace).urdf` |
| `rosbot_controller/launch/controller.yaml` | 101 | `/tmp/rosbot_controller_$(var namespace).yaml` |
| `rosbot_gazebo/launch/spawn_robot.yaml` | 96 | `/tmp/rosbot_bridge_$(var namespace).yaml` |

Nested `a/b` fails in exactly these four, for the same reason.

Doubled separator — `rosbot_gazebo/config/rosbot_bridge.yaml:2` writes
`/<namespace>/cmd_vel` while the `ns` substituted into it
(`rosbot_gazebo/launch/spawn_robot.yaml:82`) already carries its own leading
slash, giving `//my_ns/cmd_vel`.

Names that come out malformed rather than failing: the `ns` prefix in
`rosbot_controller/launch/controller.yaml:79` (into `controllers.yaml`'s
`sensor_name`), and `ns` in `rosbot_description/urdf/common/ros2_control.urdf.xacro`
(6, 31, 72) and `gazebo.urdf.xacro` (51, 54, 57) — yielding `/my_ns/imu`,
`/my_ns/rosbot_system`. `rosbot_gazebo/launch/spawn_robot.yaml:85` passes a
leading-slash entity name to Gazebo. Whether `gz_ros2_control` normalises the
`<namespace>` it is handed (`gazebo.urdf.xacro:10`) was not checked.

**Approach.** Normalise once at the top of each launch rather than patching each
call site — strip the slashes into a `ns_clean`, use
`ns_clean.replace('/', '_')` wherever the value becomes a filename, and
`ns_clean + '/'` where it becomes a name prefix. Then drop the leading `/` from
`rosbot_bridge.yaml:2`, since the prefix already supplies it.
`rosbot_moveit/launch/rviz.launch.py:41` already does this with
`.perform(context).strip("/")` and is the pattern to copy.

**Also:** `rosbot_bringup/test/test_namespace_isolation.py:26` pins
`NAMESPACE = "test_ns"`, so no test covers a leading slash. Parameterise it over
`"/test_ns"` and `"a/b"` as the regression guard — without that the fix can
silently rot. The docs should state which spellings are supported once they are;
today they neither promise nor forbid a leading slash.

### What moves in the other repos once this lands

The Cockpit currently accepts namespaces the snap then refuses, and the snap
refuses ones ROS 2 is perfectly happy with. Both sides only become correct after
the fix above — loosening either one first just moves the failure later.

**`husarion-snap-common`** — `local-ros/configure_hook_ros.sh:94` enforces
`^[0-9a-z_-]{1,20}$`: no `/`, 20 characters, and it *allows* `-`, which ROS
forbids in a name and which the manifest help says is forbidden. This is the
binding constraint; it is vendored into each snap as
`_husarion-snap-common-local/`, so it needs a release there first, then a
`source-tag` bump in every consuming snap.

**Every shipped `network.yaml` manifest** carries the same
`pattern: '^([a-z_{][0-9a-z_/{}]{0,63})?$'` and must stay identical across all
six — they are copies, not one file:

- `rosbot-snap`, `husarion-rplidar-snap`, `husarion-depthai-snap` —
  `snap/husarion-agent-extras/config-seed/manifests/network.yaml`
- `husarion-cockpit` — `deploy/rosbot/config-seed/manifests/network.yaml` and
  `dev/panther/config-seed/manifests/network.yaml`
- `meta-husarion-ugv` —
  `recipes-cockpit/ugv-cockpit-system/files/bundle/config-seed/manifests/network.yaml`

The pattern itself is close to right once the snap side allows `/`; the `help:`
text is what is actually wrong today, since it claims hyphens are forbidden
while the snap accepts them. Settle which of the two is authoritative and make
both say it.

**Order:** fix this repo → release `husarion-snap-common` → bump `source-tag` in
each snap → align manifest pattern + help everywhere → rebuild the snaps.
