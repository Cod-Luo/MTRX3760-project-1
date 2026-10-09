#!/usr/bin/env bash
# Build and run the ROS-independent controller tests on Linux or macOS.
set -eo pipefail
project_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
test_directory=$(mktemp -d "${TMPDIR:-/tmp}/project1-core.XXXXXX")
trap 'rm -rf "$test_directory"' EXIT

"${CXX:-c++}" -std=c++17 -Wall -Wextra -Wpedantic -Werror "$@" \
    -I "$project_source/turtlebot3_gazebo/include" \
    "$project_source/turtlebot3_gazebo/src/scan_reader.cpp" \
    "$project_source/turtlebot3_gazebo/src/wall_follower.cpp" \
    "$project_source/tests/wall_follower_test.cpp" \
    -o "$test_directory/wall_follower_test"

"$test_directory/wall_follower_test"
