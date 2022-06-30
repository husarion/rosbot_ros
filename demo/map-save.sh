#!/bin/bash

docker exec -it slam-toolbox bash -c "source /opt/ros/galactic/setup.bash && ros2 run nav2_map_server map_saver_cli --fmt png -f current-map"