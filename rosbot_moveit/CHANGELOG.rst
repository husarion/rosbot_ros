^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbot_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
