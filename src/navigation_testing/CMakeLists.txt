cmake_minimum_required(VERSION 3.0.2)
project(navigation_testing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  move_base_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
   find_package(rostest REQUIRED)
  add_rostest(test/navigation.test)
endif()
