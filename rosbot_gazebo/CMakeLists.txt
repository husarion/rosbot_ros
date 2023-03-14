cmake_minimum_required(VERSION 3.10.2)
project(rosbot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()