cmake_minimum_required(VERSION 3.11)
project(rosbot_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/configure_robot scripts/create_config_dir
                 scripts/flash_firmware scripts/install_udev_rules
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config firmware launch udev DESTINATION share/${PROJECT_NAME})

ament_package()
