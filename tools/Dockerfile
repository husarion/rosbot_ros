ARG ROS_DISTRO=humble
ARG PREFIX=
ARG ROSBOT_FW_RELEASE=0.8.0

## =========================== ROS builder ===============================
FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-base AS ros_builder

ARG ROS_DISTRO
ARG PREFIX

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws
RUN mkdir src

COPY tools/healthcheck.cpp /

COPY rosbot src/rosbot
COPY rosbot_bringup src/rosbot_bringup
COPY rosbot_controller src/rosbot_controller
COPY rosbot_description src/rosbot_description
COPY rosbot_utils src/rosbot_utils

RUN apt-get update && apt-get install -y \
        python3-pip

RUN vcs import src < src/rosbot/rosbot_hardware.repos && \
    cp -r src/ros2_controllers/diff_drive_controller src/ && \
    cp -r src/ros2_controllers/imu_sensor_broadcaster src/ && \
    rm -rf src/ros2_controllers && \
    # without this line (using vulcanexus base image) rosdep init throws error: "ERROR: default sources list file already exists:"
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
	rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -y

# Create health check package
RUN cd src/ && \
    MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    source /opt/$MYDISTRO/$ROS_DISTRO/setup.bash && \
    ros2 pkg create healthcheck_pkg --build-type ament_cmake --dependencies rclcpp nav_msgs && \
    sed -i '/find_package(nav_msgs REQUIRED)/a \
            add_executable(healthcheck_node src/healthcheck.cpp)\n \
            ament_target_dependencies(healthcheck_node rclcpp nav_msgs)\n \
            install(TARGETS healthcheck_node DESTINATION lib/${PROJECT_NAME})' \
            /ros2_ws/src/healthcheck_pkg/CMakeLists.txt && \
    mv /healthcheck.cpp /ros2_ws/src/healthcheck_pkg/src/

# Build
RUN MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    source /opt/$MYDISTRO/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

## =========================== Final Stage ===============================
FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

ARG ROS_DISTRO
ARG PREFIX

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY --from=ros_builder /ros2_ws /ros2_ws

RUN apt-get update && apt-get install -y \
        python3-rosdep \
        python3-pip \
        stm32flash \
        ros-$ROS_DISTRO-teleop-twist-keyboard && \
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    apt-get remove -y \
        python3-rosdep \
        python3-pip && \
    apt-get clean && \
    echo $(cat /ros2_ws/src/rosbot/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') >> /version.txt && \
    rm -rf src && \
    rm -rf /var/lib/apt/lists/*

COPY tools/healthcheck.sh /

# copy firmware built in previous stage and downloaded repository
# COPY --from=stm32flash_builder /firmware.bin /root/firmware.bin
# COPY --from=stm32flash_builder /firmware.hex /root/firmware.hex
# COPY --from=stm32flash_builder /stm32flash /usr/bin/stm32flash

# copy scripts
# COPY tools/flash-firmware.py /
# COPY tools/flash-firmware.py /usr/bin/
# COPY tools/flash-firmware-usb.py /usr/bin/

HEALTHCHECK --interval=7s --timeout=2s  --start-period=5s --retries=5 \
    CMD ["/healthcheck.sh"]
