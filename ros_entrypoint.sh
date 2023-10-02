#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/opt/ws_ralph/install/setup.bash" --
ros2 run motorcontroller listener
exec "$@"