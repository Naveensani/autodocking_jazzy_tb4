#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/turtlebot4_ws/install/setup.bash"

if [ "$#" -gt 0 ]; then
    "$@" &
fi

exec /bin/bash
