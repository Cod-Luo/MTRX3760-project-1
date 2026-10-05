#!/usr/bin/env bash
# Run a command with this simulation's ROS environment and local discovery.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/project1_ws/install/setup.bash"
export ROS_DOMAIN_ID=76
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=mtrx3760_project1_76
export TURTLEBOT3_MODEL=waffle_pi
exec "$@"
