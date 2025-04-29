FROM husarnet/ros:humble-ros-base

ARG USERNAME=husarion
ARG UID=1001
ARG GID=$UID

# Install development tools
RUN apt update -q && \
    apt upgrade -q -y && \
    apt install -y --no-install-recommends \
        python3-pip \
        ros-dev-tools \
        ros-$ROS_DISTRO-plotjuggler \
        ros-$ROS_DISTRO-plotjuggler-ros \
        ros-$ROS_DISTRO-rqt* \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-teleop-twist-keyboard

# Create and switch to user
RUN groupadd -g $GID $USERNAME && \
    useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Setup workspace
WORKDIR /home/$USERNAME/ros2_ws
RUN mkdir -p /home/$USERNAME/ros2_ws/src && \
    echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

ENV TERM=xterm-256color
ENV RCUTILS_COLORIZED_OUTPUT=1

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
