cmake_minimum_required(VERSION 3.11)
project(rosbot_joy)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(test_launch test/test_launch.py)
  ament_add_pytest_test(
    test_namespace_remap
    test/test_namespace_remap.py
    ENV
    ROS_DOMAIN_ID=88
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    TIMEOUT
    120)
endif()

ament_package()
