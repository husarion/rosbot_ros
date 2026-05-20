cmake_minimum_required(VERSION 3.11)
project(rosbot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # Offline-only: real Gazebo cannot run reliably in CI (no GPU, slow), and the
  # previous launch_pytest tests held no value beyond what test_bringup already
  # covers for the driver chain. Run a full Gazebo simulation locally via `ros2
  # launch rosbot_gazebo simulation.yaml` when validating sim changes by hand.
  ament_add_pytest_test(test_launch_offline test/test_launch_offline.py TIMEOUT
                        60 WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

ament_package()
