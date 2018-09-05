cmake_minimum_required(VERSION 2.8.3)
project(tutorial_pkg)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf
  grid_map_core
  grid_map_msgs
  grid_map_ros
  nav_msgs
  roscpp
  sensor_msgs
  frontier_exploration
)

find_package( OpenCV REQUIRED )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(tutorial_pkg_node src/tutorial_pkg_node.cpp)
add_executable(action_controller_node src/action_controller.cpp)
add_executable(mission_controller_node src/mission_controller.cpp)
add_executable(search_controller_node src/search_controller.cpp)
add_executable(drive_controller_node src/drive_controller.cpp)
add_executable(search_manager src/search_manager_node.cpp src/SearchManager.cpp)

target_link_libraries(tutorial_pkg_node
  ${catkin_LIBRARIES}
)

target_link_libraries(action_controller_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(mission_controller_node
  ${catkin_LIBRARIES}
)

target_link_libraries(search_controller_node
  ${catkin_LIBRARIES}
)

target_link_libraries(drive_controller_node
  ${catkin_LIBRARIES}
)

target_link_libraries(search_manager
  ${catkin_LIBRARIES}
)