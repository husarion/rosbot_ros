#!/bin/bash

# Check if env HUSARION_ROS_BUILD_TYPE is set
if [ -z "$HUSARION_ROS_BUILD_TYPE" ]; then
  echo "HUSARION_ROS_BUILD_TYPE is not set. Please set it to 'hardware' or 'simulation'"
  exit 1
fi

# Define a function to run commands with or without sudo
run_with_sudo() {
  if [ "$(id -u)" -ne 0 ]; then
    sudo "$@"
  else
    "$@"
  fi
}

# Install tools
run_with_sudo apt-get update
run_with_sudo apt-get install -y python3-pip ros-dev-tools stm32flash

# Import repositories
vcs import src < src/rosbot_ros/rosbot/rosbot_$HUSARION_ROS_BUILD_TYPE.repos
if [ "$HUSARION_ROS_BUILD_TYPE" == "simulation" ]; then
    cp -r src/ros2_controllers/imu_sensor_broadcaster src
    rm -rf src/ros2_controllers
fi

# Install dependencies
if ! [ -d /etc/ros/rosdep ]; then
  run_with_sudo rosdep init
fi
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
