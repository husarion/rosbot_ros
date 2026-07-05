cmake_minimum_required(VERSION 3.11)
project(rosbot_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

generate_parameter_module(battery_alert_parameters
                          src/battery_alert_parameters.yaml)

install(PROGRAMS scripts/battery_alert scripts/configure_robot
                 scripts/create_config_dir scripts/flash_firmware
                 scripts/install_udev_rules DESTINATION lib/${PROJECT_NAME})

# Parameter-selectable LED animation publisher (supersedes the retired
# led_strip_car_wave / led_strip_rainbow nodes). PNGs are decoded with the
# vendored header-only stb_image (third_party/).
add_executable(animation_publisher src/animation_publisher.cpp)
target_include_directories(animation_publisher PRIVATE third_party)
ament_target_dependencies(animation_publisher rclcpp sensor_msgs ament_index_cpp)
install(TARGETS animation_publisher DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY animations config firmware launch udev
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_firmware_version test/test_firmware_version.py)
  ament_add_pytest_test(test_configs test/test_configs.py)

  # test_led_strip launches the C++ animation nodes and subscribes to led_strip,
  # so it needs an isolated, localhost-only DDS domain to stay reliable on CI.
  ament_add_pytest_test(
    test_led_strip
    test/test_led_strip.py
    ENV
    ROS_DOMAIN_ID=87
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    TIMEOUT
    120)
endif()

ament_package()
