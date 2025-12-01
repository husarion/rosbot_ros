cmake_minimum_required(VERSION 3.11)
project(rosbot_hardware_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME} SHARED src/rosbot_system.cpp
                                   src/rosbot_imu_sensor.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  realtime_tools
  rclcpp_lifecycle
  rclcpp
  pluginlib
  std_msgs)

target_compile_definitions(${PROJECT_NAME}
                           PRIVATE "ROSBOT_HARDWARE_INTERFACES_BUILDING_DLL")

# target_compile_definitions(${PROJECT_NAME} PUBLIC
# "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"s)
pluginlib_export_plugin_description_file(hardware_interface
                                         rosbot_hardware_interfaces.xml)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(hardware_interface realtime_tools rclcpp_lifecycle
                          rclcpp std_msgs)

ament_package()
