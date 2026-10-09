# Simulation evidence snapshot

These are historical Waffle Pi simulation runs, not results for the current plain
Burger default or physical robot. New runs remain ignored by Git until selected.

| Run | Purpose and interpretation |
| --- | --- |
| `run-20261005-210427` | Early S-maze development run. Its exit check used drifting odometry; do not use this as the accurate route figure. |
| `run-20261005-211442` | Ground-truth-verified S-maze pass, 314.6 wall seconds. Final capture was recovered separately after editing the running shell harness; see its `EVIDENCE.md`. |
| `run-20261005-213255` | Visible full-camera S-maze pass, 381.5 wall seconds. Includes recorded RViz/Gazebo/terminal screenshots. |
| `run-20261005-215851-open_track` | Fast/headless open-track pass, all four landmarks and finish, 79.4 wall seconds. |
| `run-20261005-220102-branched` | Fast/headless branched-maze pass, all five landmarks and exit, 100.4 wall seconds. |
| `run-20261005-220406-s_maze` | Fast/headless S-maze pass, all three corridor checks and exit, 129.0 wall seconds. |
| `run-20261005-221119-branched` | Additional fast/headless branched-maze pass, all five landmarks and exit, 97.5 wall seconds. |
| `run-20261005-221553-branched` | Visible full-camera branched-maze pass, all five landmarks including dead-end recovery and exit, 280.5 wall seconds. Includes recorded screenshots. |

`trajectory.png` distinguishes simulator ground truth (blue) from drifting wheel
odometry (dashed orange). The green RViz path is wheel odometry, not the exact
world trajectory. Ground-truth coordinates are observer-only and never control
the robot. Recorded model-origin clearance is not a full-body collision certificate.

Fast mode requests accelerated simulation without changing the robot's simulated
speed or steering settings. It disables GUI windows and reduces camera resolution
to 320 x 240 at 10 Hz; visible mode retains the 640 x 480 camera. A headless run
cannot include a Gazebo GUI screenshot. A final camera view of sky/ground can be
normal when the robot faces outward at an exit.

The open track has a defined test finish, not an enclosed-maze exit. No saved run
establishes floating-room maze solving or substitutes for the required live A2 demo.
