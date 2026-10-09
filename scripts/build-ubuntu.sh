#!/usr/bin/env bash
# Build the Gazebo package using this checkout and a Linux-filesystem workspace.
set -eo pipefail
project_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_source/scripts/workspace.sh"
project_workspace=${1:-"$project_workspace"}
source /opt/ros/jazzy/setup.bash
mkdir -p "$project_workspace"
cd "$project_workspace"
colcon build --base-paths "$project_source/turtlebot3_gazebo" \
    --packages-select turtlebot3_gazebo --symlink-install --cmake-force-configure \
    --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
bash "$project_source/scripts/test-ubuntu.sh" "$project_workspace"
bash "$project_source/scripts/test-scripts.sh"
printf 'Workspace ready: %s\n' "$project_workspace"
