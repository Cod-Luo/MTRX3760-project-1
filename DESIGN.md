# Controller design

The controller separates ROS integration, scan processing, steering, velocity
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

The recovery helper uses one final return, ordinary `if`/`else` branches and a named
speed limit. Its validity queries expose only the information the controller needs.
The test suite keeps the stateful corner-recovery sequence together and separates
independent recovery cases into small methods.

These choices follow the supplied lectures:

| Concept | Lecture reference |
| --- | --- |
| Small private helper methods | Lec 3A, Project Management, pages 9-11 |
| One return point per function | Lec 2B, Polymorphism, page 3 |
| Const references, const methods, constants and embedded types | Lec 4B, Object Oriented Design, pages 3-13 |
| STL minimum operation and vector sorting | Lec 6A, STL Iterators, Algorithms, pages 12-13 |
| Smart pointers and `auto` | Lec 4A, C++xx and Pointers, pages 19-23 |
| ROS callbacks, timers, publishers, subscribers and QoS | Lec 5B, ROS Publishers and Subscribers, pages 12-18 |

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

The context, executor and acknowledgement APIs are a deliberate ROS integration
extension beyond the supplied lecture examples. They are retained to send the
final stop before closing the publisher's context, without adding authored
cross-thread callbacks or locks. They are separate from the scan-processing and
steering algorithm. This lecture mapping does not claim that every ROS API appears
in the slides or that the implementation has formal course approval.

## Test boundaries

The controller receives no maze coordinates, ground truth, checkpoints or exit
location. The observer uses these only to validate and record a run.
Odometry paths are visualisation outputs, not control inputs.

Regression profiles cover ideal scans, the earlier Waffle Pi configuration, and
the Burger camera model's 5 Hz laser with a -0.032 m x offset. Synthetic-scan
ROS tests check diagnostics, stop/recovery behaviour and shutdown-command delivery.
These checks complement Gazebo traversal tests, not physical testing.

See [testing](TESTING.md) and [Burger verification](evidence/review-20261009-burger/README.md).
