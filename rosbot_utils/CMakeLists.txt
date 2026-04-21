cmake_minimum_required(VERSION 3.11)
project(rosbot_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(generate_parameter_library REQUIRED)

ament_python_install_package(${PROJECT_NAME})

generate_parameter_module(battery_alert_parameters
                          src/battery_alert_parameters.yaml)

install(
  PROGRAMS scripts/battery_alert
           scripts/configure_robot
           scripts/create_config_dir
           scripts/flash_firmware
           scripts/install_udev_rules
           scripts/led_strip_car_wave
           scripts/led_strip_rainbow
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config firmware launch udev DESTINATION share/${PROJECT_NAME})

ament_package()
