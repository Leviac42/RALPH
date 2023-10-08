#!/bin/bash
set -e

# Start bluetooth
service dbus start
bluetoothd &

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/opt/ws_ralph/install/setup.bash" --
ros2 run motor_controller listener --
exec "$@"