#!/usr/bin/env bash
# One command: fresh world, verified traversal, evidence, scoped process cleanup.
set -eo pipefail
project_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
runner="$project_source/scripts/run-ros.sh"
scenario=${1:-branched}
mode=${2:-fast}
rate=${3:-2}
python3 "$project_source/scripts/scenario_config.py" "$scenario" >/dev/null
case "$mode" in
    fast) gui=false; rviz=false; fast=true ;;
    view) gui=true; rviz=true; fast=false ;;
    *) echo 'Mode must be fast or view'; exit 1 ;;
esac
existing_topics=$(bash "$runner" gz topic -l)
if [[ "$existing_topics" == *'/world/'* ]]; then
    echo 'Another Gazebo world is active in this project partition. Close its launch with Ctrl+C first.'
    exit 1
fi
mkdir -p "$project_source/evidence"
launch_log="$project_source/evidence/launch-$(date +%Y%m%d-%H%M%S)-$scenario.log"
launch_pid=''
cleanup()
{
    if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
        # Signal only the process group started by this script, not unrelated ROS jobs.
        kill -INT -- "-$launch_pid" 2>/dev/null || true
        for attempt in $(seq 1 10); do
            kill -0 "$launch_pid" 2>/dev/null || break
            sleep 1
        done
        kill -TERM -- "-$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
setsid bash "$runner" ros2 launch turtlebot3_gazebo project1_maze.launch.py \
    "scenario:=$scenario" "gui:=$gui" "rviz:=$rviz" "fast:=$fast" > "$launch_log" 2>&1 &
launch_pid=$!
echo "Starting $scenario ($mode); launch log: $launch_log"
sleep 7
if ! kill -0 "$launch_pid" 2>/dev/null; then
    cat "$launch_log"
    exit 1
fi
bash "$project_source/scripts/validate-maze.sh" "$scenario" "$rate"
if [[ "$mode" == 'view' ]] && [[ -t 0 ]]; then
    read -r -p 'World paused. Take RViz screenshots, then press Enter to close this scene: ' response
fi
