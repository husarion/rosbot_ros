#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/melodic/setup.bash"

source "devel/setup.bash"

# wait for ros master

until rostopic list
do 
echo "waiting for master"
sleep 0.1
done

rostest --reuse-master navigation_testing navigation.test



