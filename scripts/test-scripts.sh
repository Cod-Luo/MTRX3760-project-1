#!/usr/bin/env bash
set -eo pipefail
project_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
for script in "$project_source"/scripts/*.sh; do
    bash -n "$script"
done
python3 "$project_source/tests/scenario_config_test.py"
python3 "$project_source/tests/workspace_test.py"
python3 -m py_compile "$project_source/scripts/observe-maze.py" \
    "$project_source/scripts/plot-evidence.py" \
    "$project_source/turtlebot3_gazebo/launch/project1_maze.launch.py"
echo 'Shell syntax, Python syntax and scenario metadata checks passed.'
