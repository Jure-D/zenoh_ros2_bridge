#!/bin/bash
set -e

echo "Starting Zenoh-ROS2 bridge"

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /ros2_ws/install/setup.bash
export PYTHONPATH="/ros2_ws/venv/lib/python3.12/site-packages/:$PYTHONPATH"
exec "$@"
