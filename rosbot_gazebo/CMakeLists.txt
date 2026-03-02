cmake_minimum_required(VERSION 3.11)
project(rosbot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests test/test_multirobot.py test/test_sim.py)
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(
      ${_test_name}
      ${_test_path}
      APPEND_ENV
      PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT
      1800
      WORKING_DIRECTORY
      ${CMAKE_SOURCE_DIR})
  endforeach()
endif()

ament_package()
