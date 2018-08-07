cmake_minimum_required(VERSION 2.8.3)
project(tutorial_pkg)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf
)

find_package( OpenCV REQUIRED )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(tutorial_pkg_node src/tutorial_pkg_node.cpp)
add_executable(action_controller_node src/action_controller.cpp)
add_executable(mission_controller_node src/mission_controller.cpp)
add_executable(search_controller_node src/search_controller.cpp)

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
