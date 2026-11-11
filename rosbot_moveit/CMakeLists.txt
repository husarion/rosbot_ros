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
    pluginlib
    rclcpp
    rclcpp_action
    sensor_msgs
    trajectory_msgs)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# moveit_* packages only export pkg::pkg targets from Lyrical on; *_TARGETS
# exists on both Jazzy and Lyrical.
set(PACKAGE_LINK_LIBRARIES
    control_msgs::control_msgs
    geometry_msgs::geometry_msgs
    ${moveit_core_TARGETS}
    moveit_msgs::moveit_msgs
    ${moveit_ros_planning_TARGETS}
    ${moveit_ros_planning_interface_TARGETS}
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    sensor_msgs::sensor_msgs
    trajectory_msgs::trajectory_msgs)

add_library(arm_pose_mover SHARED src/arm_pose_mover.cpp)
target_include_directories(
  arm_pose_mover PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)
target_link_libraries(arm_pose_mover ${PACKAGE_LINK_LIBRARIES})

add_library(clamp_start_state_bounds SHARED src/clamp_start_state_bounds.cpp)
target_link_libraries(clamp_start_state_bounds ${PACKAGE_LINK_LIBRARIES})
pluginlib_export_plugin_description_file(moveit_core
                                         planning_request_adapters.xml)

add_executable(dock src/dock.cpp)
target_link_libraries(dock arm_pose_mover)
target_link_libraries(dock ${PACKAGE_LINK_LIBRARIES})

add_executable(home src/home.cpp)
target_link_libraries(home arm_pose_mover)
target_link_libraries(home ${PACKAGE_LINK_LIBRARIES})

add_executable(joy2servo src/joy2servo.cpp)
target_include_directories(
  joy2servo PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(joy2servo ${PACKAGE_LINK_LIBRARIES})

install(
  TARGETS arm_pose_mover clamp_start_state_bounds
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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_clamp_start_state_bounds
                  test/test_clamp_start_state_bounds.cpp)
  target_link_libraries(test_clamp_start_state_bounds
                        moveit_core::moveit_test_utils)
  target_link_libraries(test_clamp_start_state_bounds ${PACKAGE_LINK_LIBRARIES})

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py TIMEOUT
                        60 WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_moveit_extra test/test_moveit_extra.py TIMEOUT 60
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

ament_package()
