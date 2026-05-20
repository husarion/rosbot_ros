cmake_minimum_required(VERSION 3.11)
project(rosbot_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_ekf_config test/test_ekf_config.py)
  ament_add_pytest_test(test_launch test/test_launch.py)
endif()

ament_package()
