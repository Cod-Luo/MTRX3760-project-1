#!/usr/bin/env bash
# Run a command with this simulation's ROS environment and local discovery.
set -eo pipefail
project_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_source/scripts/workspace.sh"
if [[ ! -f "$project_workspace/install/setup.bash" ]]; then
    printf 'Workspace is not built: %s. Set PROJECT1_WORKSPACE and run scripts/build-ubuntu.sh.\n' "$project_workspace" >&2
    exit 1
fi
# A shared default must not silently select a binary from another checkout.
project_cache="$project_workspace/build/turtlebot3_gazebo/CMakeCache.txt"
if [[ ! -f "$project_cache" ]] || ! grep -Fxq \
    "CMAKE_HOME_DIRECTORY:INTERNAL=$project_source/turtlebot3_gazebo" "$project_cache"; then
    printf 'Workspace belongs to another checkout or has no build cache: %s. Rebuild this checkout first.\n' "$project_workspace" >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$project_workspace/install/setup.bash"
project_prefix=$(ros2 pkg prefix turtlebot3_gazebo)
if [[ "$(realpath "$project_prefix")" != "$(realpath "$project_workspace/install/turtlebot3_gazebo")" ]]; then
    printf 'Unexpected turtlebot3_gazebo package: %s\n' "$project_prefix" >&2
    exit 1
fi
printf 'Using turtlebot3_gazebo: %s\n' "$project_prefix" >&2
export ROS_DOMAIN_ID=76
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=mtrx3760_project1_76
# Simulated model; override with PROJECT1_SIM_MODEL. Must match the launch file default.
export TURTLEBOT3_MODEL="${PROJECT1_SIM_MODEL:-burger_cam}"
exec "$@"
