cmake_minimum_required(VERSION 3.11)
project(rosbot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # test_bringup spins up the full bringup launch with microros:=False and
  # publishes fake _motors/feedback + _imu/data itself, so it needs neither HW
  # nor a micro-ROS agent — only the full vcstool dependency set
  # (husarion_controllers, ...).
  set(_pytest_tests test/test_launch_offline.py test/test_bringup.py
                    test/test_namespace_isolation.py)
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(
      ${_test_name}
      ${_test_path}
      APPEND_ENV
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      ENV
      ROS_DOMAIN_ID=87
      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      TIMEOUT
      600
      WORKING_DIRECTORY
      ${CMAKE_SOURCE_DIR})
  endforeach()
endif()

ament_package()
