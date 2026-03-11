cmake_minimum_required(VERSION 3.11)
project(rosbot_moveit)

set(PACKAGE_INCLUDE_DEPENDS ament_cmake moveit_msgs moveit_ros_planning
                            moveit_ros_planning_interface rclcpp)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(dock src/dock.cpp)
ament_target_dependencies(dock ${PACKAGE_INCLUDE_DEPENDS})
add_executable(home src/home.cpp)
ament_target_dependencies(home ${PACKAGE_INCLUDE_DEPENDS})

install(TARGETS dock home DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
