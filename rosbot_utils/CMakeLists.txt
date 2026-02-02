cmake_minimum_required(VERSION 3.11)
project(rosbot_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS rosbot_utils/create_config_dir rosbot_utils/configure_robot
                 rosbot_utils/flash_firmware DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config firmware launch DESTINATION share/${PROJECT_NAME})

ament_package()
