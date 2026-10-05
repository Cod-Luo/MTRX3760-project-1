#!/usr/bin/env bash
# Observe the stationary launch, drive its maze, then pause and capture evidence.
# Start the Gazebo/RViz launch separately using run-ros.sh before this script.
set -eo pipefail
project_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
runner="$project_source/scripts/run-ros.sh"
scenario=${1:-s_maze}
rate=${2:-1}
world=$(python3 "$project_source/scripts/scenario_config.py" "$scenario")
python3 -c 'import sys; value=float(sys.argv[1]); assert 0.1 <= value <= 4, "Rate must be between 0.1 and 4"' "$rate"
run_name="run-$(date +%Y%m%d-%H%M%S)-$scenario"
run_directory="$project_source/evidence/$run_name"
mkdir -p "$run_directory"
observer_pid=''
drive_pid=''

cleanup()
{
    if [[ -n "$drive_pid" ]]; then
        kill -TERM -- "-$drive_pid" 2>/dev/null || true
    fi
    if [[ -n "$observer_pid" ]]; then
        kill -TERM "$observer_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "Evidence directory: $run_directory"
bash "$runner" python3 "$project_source/scripts/observe-maze.py" \
    --output "$run_directory" --seconds 600 --scenario "$scenario" --rate "$rate" > "$run_directory/observer.log" 2>&1 &
observer_pid=$!
for attempt in $(seq 1 30); do
    if [[ -f "$run_directory/sensors-ready.json" ]]; then
        break
    fi
    if ! kill -0 "$observer_pid" 2>/dev/null; then
        cat "$run_directory/observer.log"
        exit 1
    fi
    sleep 1
done
if [[ ! -f "$run_directory/sensors-ready.json" ]]; then
    echo 'Laser, camera and odometry did not become ready; autonomous driving was not started.'
    exit 1
fi
cat "$run_directory/sensors-ready.json"
echo "Requested simulation rate: $rate (actual rate depends on hardware)"
rate_result=$(bash "$runner" gz service -s "/world/$world/set_physics" --reqtype gz.msgs.Physics \
    --reptype gz.msgs.Boolean --timeout 5000 --req "real_time_factor: $rate")
echo "$rate_result"
if [[ "$rate_result" != *'data: true'* ]]; then
    echo 'Gazebo did not accept the requested rate; driving was not started.'
    exit 1
fi
echo 'Sensors ready. Starting autonomous right-wall following.'
setsid bash "$runner" ros2 run turtlebot3_gazebo turtlebot3_drive \
    --ros-args -p use_sim_time:=true > "$run_directory/controller.log" 2>&1 &
drive_pid=$!
progress_ticks=0
while kill -0 "$observer_pid" 2>/dev/null; do
    sleep 1
    progress_ticks=$((progress_ticks + 1))
    if (( progress_ticks % 10 == 0 )) && [[ -f "$run_directory/progress.json" ]]; then
        cat "$run_directory/progress.json"
    fi
done
passed=false
if wait "$observer_pid"; then
    passed=true
fi
observer_pid=''

bash "$runner" gz service -s "/world/$world/control" --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: true'
kill -TERM -- "-$drive_pid" 2>/dev/null || true
drive_pid=''
cat "$run_directory/observer.log"
bash "$project_source/scripts/capture-evidence.sh" "$run_directory"
$passed
