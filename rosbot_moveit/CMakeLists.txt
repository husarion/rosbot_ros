cmake_minimum_required(VERSION 3.11)
project(rosbot_moveit)

set(PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    rclcpp
    rclcpp_action
    sensor_msgs
    trajectory_msgs)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(arm_pose_mover SHARED src/arm_pose_mover.cpp)
target_include_directories(
  arm_pose_mover PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(arm_pose_mover ${PACKAGE_INCLUDE_DEPENDS})

add_executable(dock src/dock.cpp)
target_link_libraries(dock arm_pose_mover)
ament_target_dependencies(dock ${PACKAGE_INCLUDE_DEPENDS})

add_executable(home src/home.cpp)
target_link_libraries(home arm_pose_mover)
ament_target_dependencies(home ${PACKAGE_INCLUDE_DEPENDS})

add_executable(joy2servo src/joy2servo.cpp)
target_include_directories(
  joy2servo PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(joy2servo ${PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS arm_pose_mover
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
install(TARGETS dock home joy2servo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py TIMEOUT
                        60 WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_moveit_extra test/test_moveit_extra.py TIMEOUT 60
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

ament_package()
