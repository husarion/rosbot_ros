ARG ROS_DISTRO=humble

FROM husarnet/ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY . src/

RUN apt-get update && apt-get install -y \
        python3-pip \
        stm32flash \
        ros-${ROS_DISTRO}-teleop-twist-keyboard

RUN vcs import src < src/rosbot/rosbot_hardware.repos && \
    # cp -r src/ros2_controllers/diff_drive_controller src/ && \
    # cp -r src/ros2_controllers/imu_sensor_broadcaster src/ && \
    # rm -rf src/ros2_controllers && \
    rm -r src/rosbot_gazebo

# Create a script to install runtime dependencies for final image
RUN apt update && \
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} --reinstall --simulate -y --dependency-types exec >> /rosdep_install.sh && \
    chmod +x /rosdep_install.sh

RUN apt update && \
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y  && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release  && \
    echo $(cat /ros2_ws/src/rosbot/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') >> /version.txt && \
    rm -rf build log src