# MTRX3760 Project 1

Sensor-driven right-wall following with ROS 2 Jazzy, Gazebo Harmonic and RViz.
The controller uses laser measurements, not maze geometry or simulator ground truth.

## Branches

- `main`: current controller, including partial-scan recovery and lecture-style C++.
- `A3-refactor`: historical refactor snapshot before partial-scan recovery.
- `A1-wall-following-logic`: historical wall-following implementation.

The ROS package is in `turtlebot3_gazebo/`; scripts, tests and saved evidence
are at the repository root. Each branch documents its own configuration.

## Documentation

- [Setup and operation](PROJECT1.md)
- [Testing and measured results](TESTING.md)
- [Saved simulation evidence](evidence/README.md)
- [Controller design](DESIGN.md)

## Quick start

From the repository root in Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic installed:

```bash
bash scripts/build-ubuntu.sh
bash scripts/test-scenario.sh s_maze fast 2
```

Run scenarios sequentially. Use `view 1` instead of `fast 2` for full-camera
Gazebo/RViz operation. See the setup guide for workspace and model selection.
Simulation evidence is separate from physical testing and the live demonstration.

For controller-only checks on Linux or macOS, without ROS installed:

```bash
bash scripts/test-core.sh
```

This checks scan processing and steering. It does not build or run the ROS node.

## Attribution and assistance

The simulator is based on ROBOTIS TurtleBot3 simulations, Jazzy source snapshot
`45633014a14e8f438495b532a723e4ad45cbbd31`. Original copyright notices
and the repository licence are retained.

OpenAI ChatGPT/Codex assisted with implementation, code review, test tooling and
documentation. The A3 review included shutdown sequencing, validation diagnostics,
workspace selection and regression-test organisation. The assessment report should
describe this assistance accurately; edited material requires team review and validation.
