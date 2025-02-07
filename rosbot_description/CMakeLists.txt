cmake_minimum_required(VERSION 3.11)
project(rosbot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_description TESTNAME tests WORKING_DIRECTORY
                        ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/sutup_envs.sh.in")
ament_package()
