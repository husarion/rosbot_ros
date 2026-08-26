^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbot_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.3 (2026-08-26)
------------------

1.2.2 (2026-08-25)
------------------
* moveit_servo: cut collision-check CPU cost, fix config_dir gap (`#193 <https://github.com/husarion/rosbot_ros/issues/193>`_)
  * moveit_servo: fix duplicate PSM + cut collision_check_rate 30->20Hz
  move_group.launch.py already runs move_group, the canonical primary
  PlanningSceneMonitor -- is_primary_planning_scene_monitor:true had
  servo duplicating its own world/scene monitor against the yaml's own
  documented guidance to set it false in that case.
  collision_check_rate 30Hz was the single hottest loop in the
  manipulator stack (~91% of one CPU core, live-measured on the Jetson)
  -- collision_monitor runs it as an always-on thread independent of
  publish_period, doing two full FCL checks every tick. Dropped to
  20Hz: still 1.33x margin under the 20mm self-collision threshold at
  the 1 rad/s joy2servo velocity clamp (was 2x at 30Hz), for a ~33% cut.
  Not yet HW-verified -- needs a fast jog toward self-collision/an
  obstacle to confirm it still stops before contact.
  * servo.launch.py: honor config_dir for moveit_servo.yaml
  ParameterBuilder("rosbot_moveit").yaml("config/moveit_servo.yaml")
  always resolved to the package's installed share dir at launch-graph
  build time, completely ignoring config_dir -- unlike joy_config right
  next to it, which already went through pkg_config_dir. Found while
  trying to live-tune collision_check_rate on .163: the config_dir copy
  create_config_dir already drops there was silently unused dead weight.
  Moved the moveit_servo param load into an OpaqueFunction (matching
  rviz.launch.py's existing pattern) so config_dir -- a LaunchConfiguration,
  only resolvable against a real LaunchContext -- can gate a plain os.path
  read of either <config_dir>/rosbot_moveit/config/moveit_servo.yaml or the
  package share default, wrapped under the moveit_servo: namespace the
  node's generate_parameter_library listener expects.
  Not yet build-verified (rosbot_moveit isn't a locally-built overlay
  here) -- needs a colcon or snap rebuild + relaunch to confirm the arg
  still resolves and servo_node still gets its params.
  * rosbot_moveit: honor config_dir for SRDF/kinematics/joint_limits/moveit_controllers
  Same class of gap as servo.launch.py's moveit_servo.yaml (fixed in the
  previous commit): every launch file building a MoveItConfigsBuilder
  (move_group, home, dock, rviz, and servo's own SRDF/kinematics/
  joint_limits calls) always read from rosbot_moveit's installed package
  share, config_dir or not.
  Each file now conditionally chains the builder's file_path overrides
  (robot_description_semantic/robot_description_kinematics/joint_limits/
  trajectory_execution -- all accept an absolute path per
  moveit_configs_utils, same Path-join behavior ParameterBuilder uses)
  when config_dir is set, falling through to the untouched default
  otherwise. Each override list matches only what that file's Node(s)
  actually consume as parameters -- e.g. rviz2 only takes
  robot_description_kinematics + planning_pipelines, so only kinematics
  is overridden there.
  ompl_planning.yaml is NOT covered: MoveItConfigsBuilder.planning_pipelines()
  has no file_path override and always scans <package_share>/config/
  *_planning.yaml with no way to redirect the scan directory -- a
  moveit_configs_utils limitation, documented inline rather than routed
  around with a fragile post-hoc dict merge.
  Not yet build-verified, same caveat as the servo.launch.py commit.
  * Trim Readme
* Contributors: Rafal Gorecki

1.2.1 (2026-08-14)
------------------

1.2.0 (2026-08-12)
------------------

1.1.1 (2026-06-29)
------------------
* Declare moveit_core dependency in rosbot_moveit
* Contributors: rafal-gorecki

1.1.0 (2026-06-15)
------------------
* Add MAVLink backend alongside micro-ROS (runtime-switch firmware) (`#175 <https://github.com/husarion/rosbot_ros/issues/175>`_)
* Namespace audit + bringup quieting + docs/test cleanup (`#173 <https://github.com/husarion/rosbot_ros/issues/173>`_)
* Tests: CI coverage for every rosbot\_* package (`#171 <https://github.com/husarion/rosbot_ros/issues/171>`_)
* Refactor rosbot_moveit: joy2servo migration + Cartesian arm control (`#170 <https://github.com/husarion/rosbot_ros/issues/170>`_)
* Namespace MoveIt topics in joy2servo and servo_node
* Contributors: Rafal Gorecki, rafal-gorecki

1.0.0 (2026-04-21)
------------------
* Fix rosbot_moveit dependencies
* Add missing changelog descriptions for past releases (`#167 <https://github.com/husarion/rosbot_ros/issues/167>`_)
  * Remove automatic bump
  * Add missing changelog descriptions for past releases
* Contributors: Rafal Gorecki, rafal-gorecki

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
