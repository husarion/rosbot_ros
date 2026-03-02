cmake_minimum_required(VERSION 3.11)
project(rosbot_utils)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBFTDI REQUIRED libftdi1)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(firmware_flasher_ftdi src/rosbot_utils/firmware_flasher_ftdi.cpp)

target_include_directories(firmware_flasher_ftdi PUBLIC ${LIBFTDI_INCLUDE_DIRS}
                                                        ${LIBUSB_INCLUDE_DIRS})

target_link_libraries(firmware_flasher_ftdi ${LIBFTDI_LIBRARIES}
                      ${LIBUSB_LIBRARIES})

install(TARGETS firmware_flasher_ftdi DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS rosbot_utils/flash_firmware rosbot_utils/create_config_dir
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config firmware launch DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
