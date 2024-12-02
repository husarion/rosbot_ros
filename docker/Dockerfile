ARG ROS_DISTRO=humble
ARG BUILD_TYPE=hardware

FROM husarnet/ros:${ROS_DISTRO}-ros-core

ARG BUILD_TYPE

WORKDIR /ros2_ws

ENV HUSARION_ROS_BUILD_TYPE=$BUILD_TYPE

COPY .. src/rosbot_ros

RUN apt-get update && apt-get install -y \
        python3-pip \
        ros-dev-tools \
        stm32flash \
        ros-${ROS_DISTRO}-teleop-twist-keyboard && \
    # Setup workspace
    ./src/rosbot_ros/rosbot/install_dependencies.sh && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # Get version
    echo $(cat /ros2_ws/src/rosbot/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') >> /version.txt && \
    # Size optimization
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        ros-dev-tools && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
