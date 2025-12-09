cmake_minimum_required(VERSION 3.11)
project(rosbot_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    control_msgs
    geometry_msgs
    rclcpp
    sensor_msgs
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_servo)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

# add_executable(joy_servo src/joy_servo_node.cpp src/joy_control.cpp
# src/manipulation_controller.cpp) ament_target_dependencies(joy_servo
# ${PACKAGE_INCLUDE_DEPENDS})
add_executable(joy2servo src/joy2servo.cpp)
ament_target_dependencies(joy2servo ${PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)
install(TARGETS joy2servo DESTINATION lib/${PROJECT_NAME})

ament_package()
