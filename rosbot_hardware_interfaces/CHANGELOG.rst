^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbot_hardware_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-04-21)
------------------
* New firmware with microros namespace pre-communication support (`#168 <https://github.com/husarion/rosbot_ros/issues/168>`_)
  - **New firmware** (`v1.0.0-jazzy`) for ROSbot 3 and ROSbot XL enabling namespace
  configuration via serial pre-communication before microros agent starts
  - **`configure_robot` script** — serial pre-communication node: sends namespace to MCU
  over FTDI/UART, verifies firmware version, waits for ACK
  - **Separated bringup** — `bringup.yaml` dispatches to `rosbot.yaml` / `rosbot_xl.yaml`;
  ROSbot XL gets dedicated nodes: `battery_alert`, `led_strip_car_wave`
  - **`tf_namespace_bridge`** — bridges namespaced `/tf` to global `/tf` and `/tf_static`
  - **`microros_mode` arg** — allows overriding default communication mode (serial/udp)
  - Controller tuning: updated ICR, wheel params and acceleration limits for both robots
  - Renamed `FirmwareFlasherUSB/UART` → `McuManagerFTDI/UART` with added `reset_mcu()`
  - `battery_alert`: switched from `aplay` to `paplay` with configurable `audio_device`
  ROS parameter (snap-friendly via `audio-playback` interface);
  `pulseaudio-utils` dep commented pending [`ros/rosdistro#50811 <https://github.com/ros/rosdistro/issues/50811>`_](https://github.com/ros/rosdistro/pull/50811)
  - Fixed: argparse `required=True` ignoring `os.getenv()` default in `configure_robot`
  and `flash_firmware`
  - Fixed: namespace validation before serial write in `configure_robot`
  - Fixed: missing `hasattr(e, "stderr")` guard in `mcu_manager_uart`
  - Fixed: deprecated `on_init(HardwareInfo&)` → `on_init(HardwareComponentInterfaceParams&)`
  in `rosbot_hardware_interfaces`
* Add missing changelog descriptions for past releases (`#167 <https://github.com/husarion/rosbot_ros/issues/167>`_)
  * Remove automatic bump
  * Add missing changelog descriptions for past releases
* Contributors: Rafal Gorecki

0.18.8 (2026-03-02)
-------------------
* Remove conditional dependencies
* Migrate rest of the ament_python pkg to ament_cmake
* Contributors: rafal-gorecki

0.18.7 (2026-02-27)
-------------------
* Update public action version in workflow
* Contributors: rafal-gorecki

0.18.6 (2026-02-27)
-------------------
* Unnecesary auto release
* Contributors: rafal-gorecki

0.18.5 (2026-02-27)
-------------------
* prepare for apt release: execute catkin_generate_changelog manually
* Contributors: rafal-gorecki

0.18.4 (2026-02-27)
-------------------
* rosbot_bringup: migrate ament_python pkg to ament_cmake package
* Contributors: rafal-gorecki

0.18.3 (2026-02-26)
-------------------
* Use apt instead pip dependencies (python3-pyftdi-pip to python3-ftdi)
* Contributors: rafal-gorecki

0.18.2 (2026-02-26)
-------------------
* Migrate ROS2 launch from Python to YAML
* Contributors: rafal-gorecki

0.18.1 (2025-12-10)
-------------------
* Exceed arm_activate timeout
* Add Husarion packages of: open_manipulator_description, open_manipulator_joy, open_manipulator_moveit
* Use fixed mecanum controller
* Contributors: rafal-gorecki

0.18.0 (2025-12-08)
-------------------
* Add rosbot_hardware_interfaces into rosbot_ros
* Fix: wrong frame_id in simulated sensors when namespace is used
* Enable color logs
* Add stm32flasher dependency
* Reduce number of controller config files
* One spawner for all controllers (reduce number of spawn node)
* Add activate_arm arg
* Add a script to activate/deactivate the arm
* Rename topic: imu_broadcaster/imu to imu/data
* Contributors: rafal-gorecki
