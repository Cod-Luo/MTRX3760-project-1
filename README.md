# MTRX3760 Project 1

Sensor-driven right-wall following with ROS 2 Jazzy, Gazebo Harmonic and RViz.
The controller uses laser measurements, not maze geometry or simulator ground truth.

## Branches

- `A3-refactor`: current refactored implementation and Burger camera simulation.
- `A1-wall-following-logic`: earlier wall-following implementation.
- `main`: earlier integrated project version.

This branch retains the earlier implementation. Use `A3-refactor` for the current
working refactor. The ROS package is in `turtlebot3_gazebo/`; scripts, tests and
evidence are at the repository root.

## Documentation

- [Setup and operation](PROJECT1.md)
- [Testing and measured results](TESTING.md)
- [Saved simulation evidence](evidence/README.md)

## Quick start

From the repository root in Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic installed:

```bash
bash scripts/build-ubuntu.sh
bash scripts/run-ros.sh ros2 launch turtlebot3_gazebo project1_maze.launch.py
```

The older wrapper selects a plain Burger without a camera. See the setup guide
for a camera-equipped manual launch; use A3 for current automated scenario tests.
Simulation evidence is separate from physical testing and the live demonstration.

## Attribution and assistance

The simulator is based on ROBOTIS TurtleBot3 simulations, Jazzy source snapshot
`45633014a14e8f438495b532a723e4ad45cbbd31`. Original copyright notices
and the repository licence are retained.

OpenAI ChatGPT/Codex assisted with implementation, code review, test tooling and
documentation. The assessment report should
describe this assistance accurately; edited material requires team review and validation.
