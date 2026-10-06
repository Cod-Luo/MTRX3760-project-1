#!/usr/bin/env bash
# Capture the existing paused scene and plot an already recorded run.
set -eo pipefail
project_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
runner="$project_source/scripts/run-ros.sh"
run_directory=$(realpath "${1:?Supply an evidence run directory}")
case "$run_directory" in
    "$project_source"/evidence/run-*) ;;
    *) echo 'Expected a run directory inside this project/evidence.'; exit 1 ;;
esac
test -f "$run_directory/run-summary.json"
run_name=$(basename "$run_directory")
world=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("world", "project1_maze"))' "$run_directory/run-summary.json")
bash "$runner" gz service -s "/world/$world/control" --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: true'
native_screenshots="$HOME/project1_ws/evidence/$run_name"
mkdir -p "$native_screenshots"
services=$(bash "$runner" gz service -l)
if [[ "$services" == *'/gui/screenshot'* ]]; then
    bash "$runner" gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg \
        --reptype gz.msgs.Boolean --timeout 1000 --req "data: \"$native_screenshots\"" || true
    sleep 2
fi
shopt -s nullglob
screenshots=("$native_screenshots"/*.png)
if (( ${#screenshots[@]} > 0 )); then
    cp -- "${screenshots[-1]}" "$run_directory/gazebo-maze.png"
else
    echo 'No GUI screenshot available (expected in fast/headless mode); camera frames and recorded data are saved.'
fi
bash "$runner" python3 "$project_source/scripts/plot-evidence.py" --run "$run_directory"
echo "Evidence saved: $run_directory"
