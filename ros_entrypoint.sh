#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros_ws/devel/setup.bash"

GAZEBO_SETUP_PATH=/usr/share/gazebo/setup.sh
if [[ -f "$GAZEBO_SETUP_PATH" ]]; then
    source "$GAZEBO_SETUP_PATH"
fi

echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo ". /ros_ws/devel/setup.bash" >> ~/.bashrc

exec "$@"