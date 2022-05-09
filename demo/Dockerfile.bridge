FROM ros:galactic-ros1-bridge

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN apt-get update && apt-get install -y \
       ros-$ROS_DISTRO-rmw-fastrtps-cpp && \
    rm -rf /var/lib/apt/lists/*
