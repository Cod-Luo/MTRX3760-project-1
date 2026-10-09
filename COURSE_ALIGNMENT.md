# Weeks 1-6 code-quality alignment

Changes prepared on `A3-refactor`, based on `e73f4e4`, on 9 October 2026.
This note explains design choices and verification, not a guarantee of tutor approval.

## Design and language choices

- Preserve the existing small classes and genuine node inheritance. Keep steering
  and scan processing independent of ROS, with private state, named constants,
  initialised values, const getters and const-reference inputs.
- Use a simple enumeration to explain validation results. The ROS node logs changes
  between invalid settings, invalid scan data, invalid time, stale data and usable
  data. Stops and validation remain active under `NDEBUG`; assertions are not used
  for motor safety.
- Remove the authored mutex, lock guards and cross-thread pre-shutdown publishing
  callback. A single-threaded executor processes callbacks in the control loop.
  The default ROS context handles termination signals, while a separate node context
  stays valid until that loop stops executing callbacks and publishes zero velocity.
  This uses ROS lifecycle APIs, not student-written threading or signal handlers.
  Those specific ROS APIs are not demonstrated in the supplied lecture slides;
  their use still needs tutor confirmation if restrictions extend beyond C++ topics
  to every ROS API. No claim of absolute syllabus certification is made.
- Preserve non-copyability of the path recorder using private, undefined copy
  declarations instead of `= delete`; its callbacks depend on a stable address.
- Keep smart pointers and standard algorithms already supported by the supplied
  Week 4 and Week 6 material. Do not replace `std::sort` with an unnecessary custom sort.
- Encapsulate development C++ tests in their own class/header, use named model
  profiles, keep the test groups small and use one exit per function. Failure
  checks are explicit and remain active in release builds.

The comparison used the Lab 2 quality summary, the submitted Lab 1/2 code, Week 4
pointer/ownership material and Week 6 STL/debugging material. The Python and shell
files are supporting build/simulation/test tools, not the C++ steering implementation.
Python error handling and observer threading are retained where needed. If the
course restricts those tools too, clarify that separately rather than removing
error handling without a safe replacement.

## Regression coverage

The controller tests retain idealised and older Waffle profiles and add the actual
Burger camera model's -0.032 m sensor x offset and 5 Hz cadence. Scan age increases
between measurements rather than resetting every control step. The 20 Hz controller
and existing steering settings are unchanged. These ray-cast tests are not A1 evidence.

ROS integration tests use synthetic scans on an isolated domain to verify startup
stops, stale/invalid/future-dated input stops, diagnostic suppression/recovery,
invalid-setting stops and delivered final zero commands on SIGINT and SIGTERM.
Acknowledgement and receipt of zero velocity are not physical braking measurements.

Build/run/test scripts share `PROJECT1_WORKSPACE`. The run wrapper also verifies the
build's source checkout and prints its installed package prefix. Workspace regression
tests cover the default, an override containing spaces, missing builds and a build
from another checkout. Source edits still require an explicit rebuild.

## Reproduce

```bash
export PROJECT1_WORKSPACE="$HOME/project1_a3_ws"
bash scripts/build-ubuntu.sh
bash scripts/test-scripts.sh
# Optional rerun of the compiled core and synthetic-scan ROS tests:
bash scripts/test-ubuntu.sh
```

PowerShell development-only test: `& './scripts/test-windows.ps1'` from this checkout.

## Verification record

- Normal ROS package build and CTest: passed, 1/1 CTest test.
- Full Release ROS build with `-Werror` and `NDEBUG`: passed, including CTest and
  all three synthetic-scan ROS safety tests.
- The same three ROS safety tests passed on the final normal build, including
  diagnostic transitions, recovery and received final stops for both signal types.
- Native Windows C++ development build with `-Wall -Wextra -Wpedantic -Werror`:
  passed all test groups and all three ray-cast profiles.
- Scenario metadata tests: 3 passed. Workspace regression tests: 4 passed.
  Shell/Python checks and `git diff --check`: passed.

Fresh headless Burger camera simulation results:

| Scenario | Required checkpoints | Wall time | Measured real-time factor | Sampled origin-to-wall clearance |
| --- | --- | --- | --- | --- |
| S-maze | 3/3 and exit | 169.2 s | 0.922 | 0.294 m |
| Branched maze | 5/5, including dead-end return, and exit | 127.6 s | 0.900 | 0.320 m |
| Open track | 4/4 and finish | 102.8 s | 0.888 | 0.301 m |

Runtime sensor checks passed in every scenario. These were accelerated-mode
requests, not achieved 2x acceleration: the computer remained below real time.
Clearance is sampled model-origin clearance, not a full-footprint collision
certificate. Evidence and raw samples are saved in `evidence/review-20261009-burger/`.
Its README notes the startup-only logging correction after the S-maze run and
the subsequent final-build safety checks. Older October 5 evidence remains
historical Waffle evidence. No physical test or demo result is claimed.

## Assistance disclosure

Codex assisted with the code-quality review and these edits: lifecycle/stop ordering,
validation-status diagnostics, workspace selection, sensor-profile regression tests,
test organisation and documentation. Tests were executed locally; no physical-robot
performance is claimed. Incorporate an accurate disclosure in the report as required
by the assignment, and have team members review and understand the changes.
