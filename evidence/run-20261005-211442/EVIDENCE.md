# A1 simulation evidence — 5 October 2026

The actual ROS C++ executable followed the right wall using `/scan`, with no
waypoints, map or simulator ground truth fed into the controller.
The observer verified the robot's Gazebo model position at the exit after 314.6
seconds of wall time. All three S-maze corridors were visited.

## Recorded outputs

- **trajectory.png:** Computer-recorded TurtleBot3 right-wall-following trajectory
  through the S-maze. Blue is Gazebo's actual model position; dashed orange is wheel
  odometry translated to the spawn origin, demonstrating accumulated drift.
  Green marks the start beside the right wall and red marks the verified east exit.
  Ground truth was used only for recording and exit verification, not navigation.
- **camera-start.png / camera-final.png:** Actual 640 x 480 RGB images from the
  simulated onboard camera before motion and at the verified exit, respectively.
- **laser-final.png:** Actual finite LiDAR returns at the verified exit, plotted in
  sensor coordinates. Positive x is forward and positive y is left. Empty/no-return
  readings are omitted from this visualisation but handled by the controller.
- **gazebo-maze.png:** Gazebo's rendered maze scene, captured after the completed
  traversal was paused. Its screenshot time is later than the observer's exit sample.

`ground-truth.csv` records actual simulator poses; `trajectory.csv` records odometry.
`run-summary.json` and `trajectory-checks.json` contain the measured checks.
The controller published 1,388 path poses, and the observer received 1,532 laser
messages and 3,905 camera frames. The recorded minimum robot-origin-to-wall-surface
distance is 0.2825 m; it does not certify complete-body collision clearance.

The observer completed successfully. The running shell harness's capture stage was
interrupted by a source edit; it was recovered using `scripts/capture-evidence.sh`.
Recorded sensor and trajectory values were retained.

This is historical Waffle Pi simulation evidence, separate from current Burger
verification and physical-robot testing.
