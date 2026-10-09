# Controller design

The A3 refactor separates ROS integration, scan processing, steering, velocity
publication and trajectory recording. Headers are in
`turtlebot3_gazebo/include/turtlebot3_gazebo/`, with implementations in
`turtlebot3_gazebo/src/`.

| Class | Responsibility |
| --- | --- |
| `CWallFollowerNode` | Owns components, parameters, scan callbacks and 20 Hz control updates. |
| `CScanReader` | Validates laser data and extracts front, right and front-right distances. |
| `CWallFollower` | Converts validated distances into right-wall-following commands. |
| `CVelocityPublisher` | Publishes timestamped velocity commands on `cmd_vel`. |
| `CPathRecorder` | Records odometry for RViz, separately from navigation. |

`main.cpp` manages ROS initialisation and shutdown. The executable remains
`turtlebot3_drive`. Scan processing and steering are ROS-independent and directly testable.

## Encapsulation and algorithms

State is private, inputs use const references where appropriate, and named constants
describe configuration. Scan sectors use angular metadata rather than assuming that
sample indices are degrees. Filtering uses standard containers and `std::sort`.
The path recorder has private, undefined copy declarations because its callbacks
depend on a stable object address. `CWallFollowerTests` encapsulates development tests.

## Timing, diagnostics and stopping

Control updates and path sampling use ROS time. Sensor validation checks ROS
timestamp age and steady-clock receipt age. Invalid settings, timestamps, missing
input and stale data command zero velocity. Fresh partial scans retain steering:
blocked-front turns keep priority, a missing right wall triggers rightward search,
and a missing diagonal still permits right-distance control. Forward recovery is
capped at 0.01 m/s; an unknown front allows rotation only. A previously blocked
front retains its left turn until a valid front reading clears it. Recovery has
no timer or attempt limit.
Usable scans restore the original wall-following behaviour, including its obstacle
turns and range filtering. Diagnostics report state changes.
These checks remain enabled in Release builds.

Control callbacks run on the main thread. The default ROS context handles
termination signals; a separate node context remains valid until the loop exits
and publishes zero velocity, then shuts down. A bounded subscriber-acknowledgement
wait assists delivery but does not prove that physical wheels have stopped.

The implementation uses ROS lifecycle/context APIs beyond the supplied lecture
examples. Their eligibility needs confirmation if course restrictions apply to
individual ROS APIs as well as C++ language features.

## Test boundaries

The controller receives no maze coordinates, ground truth, checkpoints or exit
location. The observer uses these only to validate and record a run.
Odometry paths are visualisation outputs, not control inputs.

Regression profiles cover ideal scans, the earlier Waffle Pi configuration, and
the Burger camera model's 5 Hz laser with a -0.032 m x offset. Synthetic-scan
ROS tests check diagnostics, stop/recovery behaviour and shutdown-command delivery.
These checks complement Gazebo traversal tests, not physical testing.

See [testing](TESTING.md) and [Burger verification](evidence/review-20261009-burger/README.md).
