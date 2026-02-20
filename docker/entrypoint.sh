#!/bin/bash
set -e

# Source the ROS2 systemic setup
source /opt/ros/jazzy/setup.bash

# Activate the Python virtual environment
source /opt/venv/bin/activate

# Source our built workspace
source /ros2_ws/install/setup.bash

# Excute the command passed in from docker-compose or docker run
exec "$@"
