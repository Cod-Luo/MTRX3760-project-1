# Additional Gazebo tests

Build from this checkout's root before using the harness. The commands below
require a camera-equipped model. This older branch selects a plain Burger, so
camera-readiness checks may time out; use `A3-refactor` for the maintained
camera-backed workflow. Historical results below used Waffle Pi.
Close any existing project Gazebo launch before starting one of these commands.

```bash
bash scripts/test-scenario.sh branched fast 2
bash scripts/test-scenario.sh open_track fast 2
```

Run commands sequentially. `test-scenario.sh` starts a fresh world, checks stationary
sensors/spawn, runs the wall follower, records evidence, verifies ordered landmarks
and the finish, then closes its own simulation processes.

## What each world tests

| Scenario | Geometry | Required observations |
| --- | --- | --- |
| `s_maze` | Original S-maze, unchanged | East end of first corridor, west end of middle corridor, upper corridor, east exit |
| `branched` | Connected T-shaped corridors with a north exit | Approach junction, choose right-hand branch, enter dead end, return toward junction, reach north exit |
| `open_track` | One connected stepped solid wall in open space | Follow initial wall, negotiate outside corners and concave corner, return along opposite side to the finish |

The controller receives only its original ROS inputs, not scenario coordinates,
landmarks, an exit location or ground truth. The separate observer uses these solely
to decide whether the test passed and to record the actual trajectory.

## Fast versus visible runs

`fast 2` requests a 2x real-time factor, disables Gazebo/RViz windows, and uses a
temporary camera model at 320 x 240 / 10 Hz. Actual acceleration is hardware-limited.
The robot still moves at 0.15 m/s in simulated time. Robot/collision geometry, physics
step and laser settings are unchanged. ROS control is 20 Hz of simulation time;
path sampling is up to 5 Hz of simulation time.

For visible full-camera testing and report screenshots:

```bash
bash scripts/test-scenario.sh branched view 1
bash scripts/test-scenario.sh open_track view 1
```

The scene pauses when the finish is verified. In an interactive terminal, take your
screenshots and press Enter to close it. Visible/full-camera mode can be slower.

For a direct benchmark on the original course:

```bash
bash scripts/test-scenario.sh s_maze fast 2
```

Evidence appears under `evidence/run-<timestamp>-<scenario>/`. Headless runs contain
camera frames, LiDAR CSV/plot, actual simulator poses, odometry, a trajectory plot,
the scenario configuration and measured wall/simulation durations. They do not
contain a Gazebo GUI screenshot, because no GUI is running.

## Historical Waffle Pi runs, 5 October 2026

- Open track: `run-20261005-215851-open_track`, all four ordered landmarks and finish
  passed. 79.4 wall seconds / 94.7 simulation seconds; measured average RTF 1.19.
- Branched maze: `run-20261005-220102-branched`, all five ordered landmarks and exit
  passed, including dead-end recovery. 100.4 wall seconds / 120.8 simulation seconds;
  measured average RTF 1.20.
- Original S-maze: `run-20261005-220406-s_maze`, all three ordered corridor checks
  and exit passed. 129.0 wall seconds / 159.0 simulation seconds; measured average
  RTF 1.23. This is the same world that previously took 314.6 and 381.5 wall seconds
  with GUI/full-camera operation: approximately 2.4-3x shorter wall time in fast mode.

All used the same steering settings as the successful original S-maze. No steering
retuning was needed. These runs used the reduced camera-load profile, so they are not
full-camera performance benchmarks. Minimum *recorded model-origin* distance to wall
surfaces was 0.293 m (open track) and 0.286 m (branched); these numbers do not certify
clearance of every part of the robot.

The requested rate was 2, but all three measured averages were about 1.2. Selecting
an even higher target cannot guarantee a further improvement when this machine is
already limited by physics/rendering/ROS processing. Visible/full-camera mode remains
available for presentation-quality evidence; use fast mode for repeat testing.

The trajectory plot separates simulator ground truth (blue) from drifting wheel
odometry (dashed orange). Do not claim the orange curve is the exact world trajectory.
The open track has a defined demonstration finish, not an enclosed-maze exit, and
does not test floating-room maze solving.

## Development checks

```bash
bash scripts/test-ubuntu.sh
bash scripts/test-scripts.sh
```

The recorded development snapshot passed Linux C++ build/CTest, Windows controller
checks, shell/Python syntax and three scenario-metadata tests. Metadata checks are not runtime navigation
tests; the measured Gazebo runs above are the runtime evidence.
