#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/melodic/setup.bash"

source "devel/setup.bash"

# wait for ros master

export ROS_LOG_DIR="/results"

until rostopic list
do 
echo "waiting for master"
sleep 0.1
done

rostest --reuse-master --results-filename result.xml navigation_testing navigation.test

