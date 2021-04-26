cmake_minimum_required(VERSION 3.5)
project(rosbot_description)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)

#################
## Executables ##
#################

set(dependencies
  rclcpp
  tf2_geometry_msgs
  geometry_msgs
  tf2_ros
  tf2
)

add_executable(rosbot_tf src/rosbot_tf_node.cpp src/rosbot_tf.cpp)
target_link_libraries(rosbot_tf ${Boost_LIBRARIES})
ament_target_dependencies(rosbot_tf rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

#############
## Install ##
#############

install(DIRECTORY config launch models worlds urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS rosbot_tf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS 
  scripts/rosserial_node.py
  scripts/serialclient.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  scripts/rosserial
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
