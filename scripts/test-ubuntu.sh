#!/usr/bin/env bash
# Run the controller's actual CTest executable and save a JUnit result.
set -eo pipefail
project_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$project_source/scripts/workspace.sh"
project_workspace=${1:-"$project_workspace"}
source /opt/ros/jazzy/setup.bash
result_directory="$project_workspace/build/turtlebot3_gazebo/test_results/turtlebot3_gazebo"
mkdir -p "$result_directory"
ctest --test-dir "$project_workspace/build/turtlebot3_gazebo" --output-on-failure \
    --output-junit "$result_directory/wall_follower_core.junit.xml"
colcon test-result --test-result-base "$project_workspace/build" --verbose
PROJECT1_WORKSPACE="$project_workspace" bash "$project_source/scripts/run-ros.sh" \
    python3 "$project_source/tests/ros_node_test.py"
