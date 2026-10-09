# MTRX3760 Project 1: A1 simulation work

This is a local checkout of the official ROBOTIS TurtleBot3 simulations Jazzy branch.
The initial upstream commit is `45633014a14e8f438495b532a723e4ad45cbbd31`.
The assignment's `turtlebot3_drive.cpp` was modified in place for A1, then split into
classes during the A3 refactor (see Code structure). Current work is on the group's
`A3-refactor` branch. The existing group setup history is preserved.

## Development history and current status

The dated October 5 runs below used the previous Waffle Pi configuration. They are
historical evidence, not runs of the current Burger refactor. See `COURSE_ALIGNMENT.md`
for the October 9 code-quality changes and their verification.

- Right-wall-following C++ controller implemented.
- Windows development tests pass, including a sensor-driven traversal of an idealised S-maze.
- Ubuntu 24.04 WSL2, ROS Jazzy desktop and Gazebo Harmonic installed.
- Actual ROS C++ build and CTest pass (1 test, 0 failures).
- Gazebo and RViz run through WSLg; simulated 360-sample LiDAR and 640 x 480 camera verified.
- Ground-truth-verified autonomous traversal passed: all three corridors visited and
  east exit reached in 314.6 seconds of wall time (run `20261005-211442`).
- Evidence saved: 1,532 laser messages, 3,905 camera frames, 1,388 RViz path poses,
  2,668 plotted ground-truth samples, scene/camera screenshots and LiDAR plot.
- WSLg initially had a shared-memory I/O failure and invisible `[WARN: COPY MODE]`
  windows. After saving evidence and restarting WSL, its log confirms graphics
  redirection is enabled (`use_gfxredir = 1`). You subsequently confirmed both windows
  and supplied screenshots of a second successful run (`20261005-213255`).
- Separate branched-maze and open-track worlds added; scenario-aware validation
  checks ordered landmarks before accepting each scenario's own exit region.
- Both new worlds and the original S-maze passed measured fast-mode Gazebo runs
  with unchanged steering settings (79.4 s, 100.4 s and 129.0 s respectively).
- A1 report assembly/captions, team review and A2 physical testing remain.

The native Windows ray-cast tests check controller logic with three explicit profiles:
ideal readings at the 20 Hz control rate, the earlier Waffle Pi (10 Hz laser, -0.064 m
sensor x offset), and the selected Burger camera model (5 Hz laser, -0.032 m offset).
Mounted profiles use seeded 0.01 m Gaussian noise. Scan age advances between updates
according to each profile. These tests omit Gazebo physics, actuator dynamics and
ROS communication; their success is not evidence for A1.

## Code structure

After the A3 refactor each class has one job (headers in
`turtlebot3_gazebo/include/turtlebot3_gazebo/`, sources in `turtlebot3_gazebo/src/`):

| Class | File | Job |
| --- | --- | --- |
| `CWallFollowerNode` | `wall_follower_node` | The ROS node and single top-level owner. Reads parameters, receives scans, runs the 20 Hz update. |
| `CScanReader` | `scan_reader` | Turns a raw laser scan into front, right and front-right distances. |
| `CWallFollower` | `wall_follower` | Turns those distances into drive commands using three prioritised rules. |
| `CVelocityPublisher` | `velocity_publisher` | Sends timestamped commands on `cmd_vel` as `TwistStamped`. |
| `CPathRecorder` | `path_recorder` | Records odometry as an RViz path. Not used for control. |

`main.cpp` owns ROS startup and shutdown. The executable is still called `turtlebot3_drive`.
`CScanReader` and `CWallFollower` do not depend on ROS, so the unit tests use them
directly. `CScanReader` reads angular sectors rather than assuming that sample indices
equal degrees, so it works on both the 360-sample simulated lidar and the real LD19
(0.72 degrees per sample). ROS uses positive angular velocity for left turns, unlike
the screen coordinates used in the Lab 2 simulator.
The ROS control timer and path sampling now use ROS time: 20 control updates and
up to five path samples per simulated second. The independent steady-clock sensor
receipt watchdog remains, alongside the ROS timestamp-age check. Steering settings
and the original S-maze geometry are unchanged.

The node reports invalid settings, invalid scans, invalid timestamps, stale scans
and recovery once per state change. These runtime checks remain active in release
builds. The initial waiting-for-laser message explains why startup produces zero velocity.

All control callbacks execute on the main thread. ROS's default context receives
Ctrl+C/SIGTERM; the node uses a separate context kept alive until its loop ends and
sends zero velocity. Only then does main shut down the control context. There is no
authored mutex or cross-thread publishing hook. A bounded reliable-subscriber
acknowledgement wait helps delivery, but cannot prove that physical wheels stopped.

The controller follows a wall on its right, turns left when the front is blocked,
and curves right when it loses the wall. Invalid or stale scans stop motion.
It receives no waypoints or maze geometry. The maze assumes a wall beside the start
and has no floating rooms. The drive controller does not detect the exit: the separate
validation harness observes Gazebo's model position, pauses the world at the exit and
stops its own drive process. Ground truth and maze coordinates are never sent to the controller.

The supplied Jazzy simulation bridge expects `TwistStamped`; the controller publishes
this type on `cmd_vel`. Before physical operation, check the actual robot with
`ros2 topic info /cmd_vel --verbose`. Physical operation still requires tuning and live tests.

## Run development tests now (PowerShell)

Run from the root of this `A3-refactor` checkout, not an older `Project1` copy:

```powershell
& '.\scripts\test-windows.ps1'
```

The test executable is ignored by Git. Tests cover steering signs, front-obstacle
hysteresis, wall loss, missing/invalid/stale scans, scan angle wrapping, different scan
resolutions and invalid settings. The closed-loop tests ray-cast the supplied maze's
wall centrelines and check exit traversal plus collision clearance. Observed minimum
centre-to-wall clearance was 0.259 m (ideal readings), 0.269 m (Waffle profile) and
0.272 m (Burger profile). This simplified clearance is not a full-footprint certificate.

## Installed environment

Ubuntu-24.04 is installed and the normal Linux account is `royce`.
To enter it from PowerShell:

```powershell
wsl --distribution Ubuntu-24.04
```

Keep your Linux password private. `scripts/install-ros-jazzy.sh` has already been
executed successfully; it is retained for reproducibility, not needed each time.
It follows the official ROS apt-source instructions. References:

- https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- https://gazebosim.org/docs/harmonic/ros_installation/
- https://learn.microsoft.com/en-us/windows/wsl/install

## Build and test in Ubuntu

Use the checkout on the Windows drive as the source, but keep build outputs on the
Linux filesystem for better performance:

```bash
cd /path/to/MTRX3760-project-1  # Replace with your A3-refactor checkout.
bash scripts/build-ubuntu.sh
```

The default workspace is `$HOME/project1_ws`. For a separate workspace, set the same
environment variable in every terminal used for this checkout:

```bash
export PROJECT1_WORKSPACE="$HOME/project1_a3_ws"
bash scripts/build-ubuntu.sh
bash scripts/run-ros.sh ros2 pkg prefix turtlebot3_gazebo
```

Build, test and runtime scripts share this setting through `scripts/workspace.sh`.
The build script still accepts a workspace argument, but that argument affects only
that invocation; export `PROJECT1_WORKSPACE` for subsequent runtime commands.
The runtime wrapper prints the package prefix and rejects workspaces built from
another checkout. Rebuild after source edits; the wrapper is not an automatic builder.

The script builds the actual ROS node, runs CTest and reports its JUnit result. It also
runs synthetic-scan ROS tests for diagnostics, recovery and final stop delivery under
SIGINT/SIGTERM. To rerun the installed build's tests, use `bash scripts/test-ubuntu.sh`.
Run non-ROS script/metadata checks with `bash scripts/test-scripts.sh`.
A Windows test alone is not a ROS build.

## Launch the A1 simulation

In an Ubuntu terminal:

```bash
cd /path/to/MTRX3760-project-1  # Replace with your A3-refactor checkout.
bash scripts/run-ros.sh ros2 launch turtlebot3_gazebo project1_maze.launch.py
```

This launches Gazebo and RViz with a stationary robot. The default model is `burger_cam`:
the lab's Burger with a Pi camera, so it provides both the simulated laser and camera.
Choose another model with `model:=burger` (no camera) or by exporting
`PROJECT1_SIM_MODEL`. Your own `TURTLEBOT3_MODEL=burger` (needed for the real robot)
does not change the simulated model. RViz uses `odom` and displays `/scan`,
`/camera/image_raw`, the robot and `/wall_follower/path`.

`run-ros.sh` sources ROS/workspace setup and selects model `burger_cam` (or `PROJECT1_SIM_MODEL`), ROS domain 76,
localhost discovery and Gazebo partition `mtrx3760_project1_76`. Use it for every
ROS/Gazebo command in this project so terminals connect to the same isolated scene.
Only run one copy of the maze launch at a time. Stop it with Ctrl+C before a fresh run;
do not reuse a completed/paused world as if the robot were still at the start.

For optional sensor diagnostics in another Ubuntu terminal at the same project directory:

```bash
bash scripts/run-ros.sh ros2 topic info /scan --verbose
bash scripts/run-ros.sh ros2 topic info /camera/image_raw --verbose
bash scripts/run-ros.sh ros2 topic info /cmd_vel --verbose
```

For a complete evidence run, use this in that second terminal:

```bash
bash scripts/validate-maze.sh
```

It checks stationary sensor data and the configured spawn position before driving,
records the run, and pauses the scene at the exit (600-second wall-time limit).
Do not start a second controller/teleoperation publisher during validation.
For an informal run without the evidence harness, append `controller:=true` to
the launch command. Headless diagnostics can also append `gui:=false rviz:=false`.

The start is (-2.4, -2.55), heading east. The route passes through three corridors
and the opening on the east side. These coordinates configure the test world;
the controller does not receive them. Odometry/path coordinates may start at the
robot's spawn origin rather than the Gazebo world origin.

## A1 evidence and remaining handoff

The harness creates `evidence/run-YYYYMMDD-HHMMSS/` with actual camera frames,
laser readings, Gazebo scene screenshot, ground-truth and wheel-odometry CSVs,
a computer-plotted trajectory and JSON summaries. Build outputs and new evidence
runs are ignored by Git; the saved runs listed in `evidence/README.md` are included
in this publishing snapshot.

Use `evidence/run-20261005-211442/` for the verified run. `run-summary.json` reports
the actual exit at (4.0005, 2.0094) m. Minimum recorded model-origin-to-wall-surface
distance was 0.2825 m. This is not the minimum clearance of every point on the robot.
The earlier `run-20261005-210427` used drifting odometry for its exit check; it is
development evidence only and should not be used as the accurate route figure.

This run's observer completed successfully, but editing the running shell harness
interrupted its final capture stage. Capture/plotting were then executed separately
and verified. The current harness calls that same capture helper after the observer.
Do not edit a shell script while it is running.

You have supplied RViz and Gazebo screenshots for run `20261005-213255` (381.5
wall seconds, successful exit); include them with its saved trajectory plot.
Add descriptive report captions and the start/right-wall assumption.
Preserve a team-reviewed working commit and genuine development history.

RViz's green `/wall_follower/path` line is recorded from wheel odometry, not drawn by hand.
The node retains the full path at up to five samples per second for a short maze run.
Wheel odometry drifts significantly during turns in this simulation. The saved plot
therefore uses Gazebo's actual model position for the route and shows odometry as a
dashed comparison. Ground truth is read-only test instrumentation, not a navigation input.
`trajectory-checks.json` also checks that all three corridors were visited and reports
model-origin clearance (not a full-footprint collision certificate).

Optional ROS bag recording (large, especially camera images):

```bash
bash scripts/run-ros.sh ros2 bag record -o "$HOME/project1_ws/a1_run" /scan /camera/image_raw /odom /wall_follower/path /cmd_vel /tf /tf_static /clock
```

Stop the bag when the robot exits. Stop the Gazebo launch with Ctrl+C when finished.
If stopping only the drive node while leaving Gazebo running, stop the node first,
then publish a zero command:

```bash
bash scripts/run-ros.sh ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'
```

The final report must acknowledge AI assistance as required by the assignment.
A2 still requires the real robot: simulation evidence cannot substitute for live testing.

## Additional worlds and faster tests

World selection is `scenario:=s_maze`, `scenario:=branched` or `scenario:=open_track`.
The original S-maze file is unchanged. `project1_branched.world` adds a T-junction,
a right-hand dead-end branch, a required return, and a north exit.
`project1_open_track.world` is one connected stepped solid wall in open space,
with exposed outside corners and a concave corner, without an enclosing opposite wall.
Its finish region terminates the wall-following demonstration; it is not an enclosed-maze exit.

Run these from the project directory inside Ubuntu (close any existing maze launch first):

```bash
bash scripts/test-scenario.sh branched fast 2
bash scripts/test-scenario.sh open_track fast 2
bash scripts/test-scenario.sh s_maze fast 2
```

Each command owns a fresh world, waits for camera/LiDAR/odometry at the configured
start, sets the requested Gazebo real-time factor, starts the unchanged wall-following
logic, records ordered landmarks and the finish, plots evidence, and closes only its
own launch process group. Unknown scenario names and an already active project world
are rejected. Keep these commands sequential, not parallel in the same partition.

`fast` disables both GUI windows and uses a temporary model with a 320 x 240 camera
at 10 Hz. The 1 ms physics step, physical robot geometry, 360-sample LiDAR at its
original rate, steering parameters and robot speed are unchanged. Full camera mode
remains 640 x 480 at the upstream rate. Both modes still produce genuine camera data.

`2` requests two simulated seconds per wall second; it is a target, not a guarantee.
The machine may achieve less, depending on rendering/physics/ROS load. The accepted
range is 0.1 to 4. The observer reports both elapsed wall and simulated seconds and
their measured average ratio. Do not confuse simulation acceleration with increasing
`forward_speed`: the latter changes the controller's physical behaviour and needs retuning.

To watch a full-camera run and keep the paused windows open for screenshots:

```bash
bash scripts/test-scenario.sh branched view 1
```

In an interactive Ubuntu terminal, press Enter after taking screenshots to close it.
For manual two-terminal control, launch with the desired `scenario:=...` and run
`bash scripts/validate-maze.sh branched 1` in the second terminal (matching the world).
`validate-maze.sh` without arguments still selects the baseline S-maze at rate 1.

Fast/headless runs save camera images, scan data, ground truth, odometry and plots,
but cannot return a Gazebo GUI scene screenshot because no GUI exists. Use `view`
mode for the report's visible Gazebo/RViz screenshots. Each run stores a copy of its
scenario metadata; ground truth/landmarks remain observer-only and are never a
navigation input. Fast-mode tests are robustness evidence, not physical-robot proof.

Confirmed: open-track fast run `20261005-215851-open_track` passed all four
ordered landmarks in 79.4 wall seconds / 94.7 simulated seconds (average RTF 1.19).
Branched run `20261005-220102-branched` passed all five landmarks, including dead-end
recovery, in 100.4 wall seconds / 120.8 simulated seconds (average RTF 1.20).
The original S-maze also passed fast run `20261005-220406-s_maze` in 129.0 wall
seconds / 159.0 simulated seconds (average RTF 1.23), compared with earlier GUI/full-
camera times of 314.6 and 381.5 wall seconds. See `TESTING.md` for commands/results.

`bash scripts/test-scripts.sh` checks shell/Python syntax and three metadata tests
(including world names, bounds, and collision-free spawn origins). This does not
replace running the robot in Gazebo.

Gazebo Harmonic timing implementation/reference:
https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/SimulationRunner.cc

## If WSLg shows invisible `[WARN: COPY MODE]` windows

In this session `/mnt/wslg/weston.log` showed shared-memory allocation failing with
an I/O error and `use_gfxredir = 0`, while Gazebo's own screenshot renderer still worked.
After saving the run and closing our launch, `wsl --shutdown` reset WSLg; its next log
showed `use_gfxredir = 1`. This stops **all** WSL sessions, so save work first and do
not run it during a simulation or while other WSL tasks are active.
No Mesa PPA, driver replacement or shared-memory mount workaround was applied.
