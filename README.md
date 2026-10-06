# MTRX3760 Project 1

TurtleBot3 wall-following project for MTRX3760.

This project is based on the ROBOTIS `turtlebot3_gazebo` package.
This `Royce` branch contains the sensor-driven right-wall-following implementation,
three Gazebo test worlds, RViz configuration, test scripts and saved simulation
evidence. The controller does not receive maze geometry or simulator ground truth.

The ROS package now lives in `turtlebot3_gazebo/` so the tested scripts and
ROS-independent tests can sit alongside it. The existing setup commits and ROS 2
Jazzy `TwistStamped` compatibility are preserved. Other repository branches are
not changed by this upload.

## Start here

- [Project setup and design](PROJECT1.md)
- [Test commands and measured results](TESTING.md)
- [Saved evidence and its limitations](evidence/README.md)

With Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic installed, run from this
repository's directory inside Ubuntu:

```bash
bash scripts/build-ubuntu.sh
bash scripts/test-scenario.sh s_maze fast 2
bash scripts/test-scenario.sh branched fast 2
bash scripts/test-scenario.sh open_track fast 2
```

Run scenarios sequentially. Replace `fast 2` with `view 1` for full-camera
Gazebo/RViz screenshots. The build script uses `$HOME/project1_ws` by default.
Examples in `PROJECT1.md` show Royce's original working directory; replace that
path with your own checkout location.

A1 report assembly and team review remain; A2 requires live physical-robot testing.
AI assistance was used for implementation, test tooling and documentation; see
the assignment's acknowledgement requirements before submitting.
