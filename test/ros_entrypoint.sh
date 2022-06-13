#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/melodic/setup.bash"

source "devel/setup.bash"

/wait4master.sh

exec "$@"