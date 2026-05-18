# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
for the `X.Y.Z` portion of each tag.

Tags on the canonical `jazzy` branch are bare `X.Y.Z` (e.g. `1.0.0`).
Tags cut from feature branches carry a track suffix
(`X.Y.Z-jazzy-mavlink`, etc.) and are parallel release lineages until
the feature merges back into `jazzy`.

The version numbers here are independent of `rosbot-firmware`'s — the
two repos co-evolve but version on different cadences. `rosbot-snap`
pins specific tagged versions of each at build time.

## [1.1.0-jazzy-mavlink] - 2026-05-18

### Added
- MAVLink bridge launch in `rosbot_bringup`, runnable alongside the micro-ROS agent — paves the way for the rosbot-firmware MAVLink lineage.
- `led_strip` launch arg in `rosbot_xl.yaml` bringup.
- `frame_filters` parameter exposed for `tf_namespace_bridge` via per-package configs in `rosbot_bringup` and `rosbot_gazebo` (default empty = pass-through).
- `just release` recipe and tag-driven GitHub Actions release workflow, with self-bootstrapping pre-commit.

### Changed
- Bumped rosbot-firmware pin to `v1.1.0-jazzy` (improved PID tuning).
- Bumped `tf_namespace_bridge` and cleaned up Gazebo launches.

### Fixed
- `rosbot_hardware_interfaces` now shuts down the driver when `ros2_control_node` exits, instead of lingering with stale state.
- Namespaced MoveIt topics in `joy2servo` and `servo_node` (`rosbot_moveit`) so multi-robot setups work correctly.
- Pre-communication / namespace handshake before the micro-ROS agent starts.
- Corrected `serial_baudrate` documentation in README (921600).

