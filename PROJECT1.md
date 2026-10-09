# Setup and operation: earlier implementation

This branch retains the two-class wall-following implementation. The current
refactor and verified Burger camera tests are on `A3-refactor`.
Historical Waffle Pi runs are listed in [evidence notes](evidence/README.md).

## Code structure

`turtlebot3_gazebo/src/turtlebot3_drive.cpp` owns ROS publishers, subscribers,
scan timing, parameters and a computer-recorded odometry path.
`WallFollower` owns scan interpretation and steering. It reads angular sectors rather
than assuming that sample indices equal degrees. ROS uses positive angular velocity
for left turns, unlike the screen coordinates used in the Lab 2 simulator.
The ROS control timer and path sampling now use ROS time: 20 control updates and
up to five path samples per simulated second. The independent steady-clock sensor
receipt watchdog remains, alongside the ROS timestamp-age check. Steering settings
and the original S-maze geometry are unchanged.

The controller follows a wall on its right, turns left when the front is blocked,
and curves right when it loses the wall. Invalid or stale scans stop motion.
It receives no waypoints or maze geometry. The maze assumes a wall beside the start
and has no floating rooms. The drive controller does not detect the exit: the separate
validation harness observes Gazebo's model position, pauses the world at the exit and
stops its own drive process. Ground truth and maze coordinates are never sent to the controller.

The supplied Jazzy bridge uses `TwistStamped`; the controller matches it by default.
The parameter `use_stamped_velocity:=false` selects `Twist` if a later platform needs it.
Physical robot operation still requires checking its topic types, tuning and live tests.

## Windows development checks

Run from this checkout's root:

```powershell
& '.\scripts\test-windows.ps1'
```

The test executable is ignored by Git. Tests cover steering signs, front-obstacle
hysteresis, wall loss, missing/invalid/stale scans, scan angle wrapping, different scan
resolutions and invalid settings. The closed-loop tests ray-cast the supplied maze's
wall centrelines and check exit traversal plus collision clearance. Observed minimum
centre-to-wall clearance was 0.259 m (ideal readings) and 0.269 m (model sensor settings).

## Requirements

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and the dependencies declared in
`turtlebot3_gazebo/package.xml`. Visible operation requires a desktop display or WSLg.
`scripts/install-ros-jazzy.sh` provides the installation procedure.

References:

- https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- https://gazebosim.org/docs/harmonic/ros_installation/
- https://learn.microsoft.com/en-us/windows/wsl/install

## Build and test in Ubuntu

Use the checkout on the Windows drive as the source, but keep build outputs on the
Linux filesystem for better performance:

```bash
bash scripts/build-ubuntu.sh
```

The runtime wrapper uses `$HOME/project1_ws`. Build into that default workspace
when using this older wrapper. The script builds the actual ROS node,
runs CTest and reports its JUnit result. To rerun only the installed build's tests,
use `bash scripts/test-ubuntu.sh`. A Windows test alone is not a ROS build.

## Launch the A1 simulation

In an Ubuntu terminal:

```bash
bash scripts/run-ros.sh ros2 launch turtlebot3_gazebo project1_maze.launch.py
```

This launches Gazebo and RViz with a stationary plain `burger`, which has no camera.
For a camera-equipped manual launch, override the model after the wrapper sets its
environment:

```bash
bash scripts/run-ros.sh env TURTLEBOT3_MODEL=burger_cam ros2 launch turtlebot3_gazebo project1_maze.launch.py
```

RViz uses `odom` and displays `/scan`, `/camera/image_raw`, the robot and
`/wall_follower/path`. Use `A3-refactor` for current automated camera-backed tests;
this branch's scenario wrapper selects a plain Burger and camera-readiness checks
may time out with that default.

`run-ros.sh` sources ROS/workspace setup and selects model `burger`, ROS domain 76,
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

## Recorded evidence

Saved runs contain camera frames, laser data, ground-truth and wheel-odometry
CSVs, trajectory plots and JSON summaries. The saved October 5 runs used the
earlier Waffle Pi configuration, not this branch's current plain Burger default.
See [evidence notes](evidence/README.md) for configurations and capture provenance.
New runs remain ignored by Git until selected for inclusion.

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

Simulation evidence cannot establish physical-robot or live-demo performance.
See [attribution and assistance](README.md#attribution-and-assistance).

## Scenario harness

The following commands document the harness interface. They require a camera-equipped
configuration; the current plain Burger default does not provide camera messages.
Use `A3-refactor` for the maintained camera-backed scenario workflow.

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

## WSLg display troubleshooting

Invisible `[WARN: COPY MODE]` windows have been observed alongside shared-memory
I/O errors and `use_gfxredir = 0` in `/mnt/wslg/weston.log`. Restarting WSL restored
graphics redirection in recorded development sessions.

Save all WSL work and stop the simulation before running `wsl --shutdown` in
PowerShell, then reopen Ubuntu. This stops every WSL distribution and session.
