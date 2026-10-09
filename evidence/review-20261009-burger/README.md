# Burger verification, 9 October 2026

These are actual headless Gazebo runs of the local course-alignment changes based
on `A3-refactor` commit `e73f4e4`. They are simulation evidence, not physical results.

The model is `burger_cam`: a Burger body with a camera, a 360-sample 5 Hz laser and
a -0.032 m laser x offset. Fast mode uses 320 x 240 camera images at 10 Hz. The
requested real-time factor was 2.0, but the measured rate is recorded separately
in each summary. Steering settings were unchanged across scenarios.

Each run directory contains the observer summary, ordered-checkpoint and sampled
clearance checks, raw trajectory and ground-truth samples, laser returns, camera
images, plots and logs. Ground truth is observer-only; the controller receives no
world coordinates or prescribed route. Wheel odometry drifts and is not the exact
world trajectory. Model-origin clearance does not certify full-body collision freedom.

The S-maze run preceded a final startup-only correction to avoid initialising ROS
logging twice; its log retains that harmless warning. Steering, the single-threaded
stop sequence and validation were unchanged by that correction. The final normal
and Release builds and synthetic-scan safety tests were rerun after it. Branched
and open-track runs use the corrected binary.

The temporary validation harness used the explicitly selected Linux workspace,
ROS domain 78 and a separate Gazebo partition. It waited for camera, laser and
odometry readiness before starting the drive process, checked the scenario's
ordered landmarks/exit and then paused and cleaned up its own world/processes.
The normal project harness remains available as `scripts/test-scenario.sh` with
`PROJECT1_WORKSPACE` selecting the build.

See `../../COURSE_ALIGNMENT.md` for measured outcomes and build/test details.
