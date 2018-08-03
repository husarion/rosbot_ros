cmake_minimum_required(VERSION 2.8.3)
project(tutorial_pkg)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(tutorial_pkg_node src/tutorial_pkg_node.cpp)

target_link_libraries(tutorial_pkg_node
  ${catkin_LIBRARIES}
)