FROM ros:humble-ros-base

WORKDIR /ros2_ws

RUN mkdir src

COPY ./rosbot src/rosbot
COPY ./rosbot_bringup src/rosbot_bringup
COPY ./rosbot_controller src/rosbot_controller
COPY ./rosbot_description src/rosbot_description
COPY ./rosbot_gazebo src/rosbot_gazebo

SHELL ["/bin/bash", "-c"]

RUN apt update && \
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rm -r src/rosbot_gazebo && \
    vcs import src < src/rosbot/rosbot_hardware.repos && \
	rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep install --from-paths src --ignore-src -y
# rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y --ignore-src -y

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	colcon build --packages-skip \
        ackermann_steering_controller \
        admittance_controller \
        bicycle_steering_controller \
        tricycle_steering_controller \
        effort_controllers \
        force_torque_sensor_broadcaster \
        forward_command_controller \
        gripper_controllers \
        joint_trajectory_controller \
        position_controllers \
        range_sensor_broadcaster \
        ros2_controllers \
        ros2_controllers_test_nodes \
        rqt_joint_trajectory_controller \
        tricycle_controller \
        tricycle_steering_controller \
        velocity_controllers  && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

# COPY ros_entrypoint.sh /

RUN echo -e \
'#!/bin/bash\n \
set -e\n \
source "/opt/ros/$ROS_DISTRO/setup.bash" --\n \
source "/ros2_ws/install/setup.bash"\n \
exec "$@"' > /ros_entrypoint.sh
