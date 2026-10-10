# QR quadruped precision locomotion

[中文](README.md) | **English**

Development branch: `RC2027`. This work extends Dog-control and its existing ROS 2 Humble/Gazebo/WBC stack. HIM training and RL deployment remain in their separate repository.

The robot has **IMU and joint feedback only; no sole contact sensors**. Contact is inferred from joint effort/model residuals, foot kinematics, hysteresis and persistence. Estimated force is not a measured sole force. Real actuator effort calibration and hardware validation remain outstanding. Unknown or invalid effort must not be treated as confirmed support.

The first release uses explicitly known horizontal surfaces. It does not implement lidar/camera perception, a complete obstacle course, navigation or return-to-start. Gazebo contact and pose truth are isolated under `/evaluation`; the precision controller does not consume them.

Precision mode is off by default and rejected with the real-hardware backend. No physical motor testing is authorized by the simulation launch.

Packages: `qr_interfaces` (contracts), `qr_planning` (geometry/reference primitives), `qr_course` (rules and fixture generation), `qr_bringup` (simulation startup), `qr_validation` (independent assessment). The existing controller owns estimation, execution and the precision WBC. Planning callbacks run outside the update thread. The original velocity controller remains available.

Build from the repository root after building the existing dependency and model underlays:

```bash
source /opt/ros/humble/setup.bash
source external/ocs2_ws/install/local_setup.bash
source external/model_ws/install/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --symlink-install \
  --packages-up-to custom_dog_control qr_bringup qr_validation \
  --executor sequential --cmake-args -DCUSTOM_DOG_CONTROL_BUILD_REAL_HARDWARE=OFF
source install/local_setup.bash
python3 src/qr_validation/scripts/run_precision.py \
  --height 0.0 --foot 0 --output artifacts/qr/trial-001
```

Foot indices are FR/FL/RR/RL. Heights are 0, 0.03 or 0.05 m. Each output directory must be new. `--gui` displays the simulation. The independent evaluator uses a private ROS domain and Gazebo master.

M1 begins at a standing simulation pose, not a validated prone-to-standing sequence. It accepts one prevalidated step at a time; it is not a general terrain planner. Full environment collision checking, NX profiling, physical deployment and complete competition autonomy are not claimed. See the [Chinese architecture/interface details](interfaces.md), [roadmap](roadmap.md), and [actual validation status](validation.md).


The MID360 / FAST-LIO ROS 2 sources are now pinned and retrieved from the owner's
repository; see [provenance and integration prerequisites](perception-source.md).
This optional dependency is not connected to precision control.

Run `src/qr_validation/scripts/run_matrix.py --trials-per-foot 25 --jobs 2
--output artifacts/qr/matrix-100` (as a single shell command) for the step matrix.
`run_precision.py --scenario` supports `cancel`, `unreachable`, `replay`,
`missed_touchdown` (with `--height 0.03`) and `stale_imu`.
All failed trials remain in the success-rate denominator. Foot error statistics
cover completed executions with an explicit sample count. Matrix acceptance is
not complete fault coverage, domain randomization, or competition acceptance.

Gazebo remains paused until controller activation has completed through bounded
single steps. The evaluator additionally observes leg and body collisions.

The precision runtime now orchestrates three separate components: a ROS-free
`PrecisionPlanner`, a ROS-free `PrecisionExecutionCore`, and a `PrecisionRosAdapter`.
Each dynamics consumer owns its mutable model caches. Precision mode loads the
shared `RobotModel` directly without initializing an NMPC optimization problem.
Contact types and model validation live in common headers.

Control telemetry uses fixed triple buffering with one producer and serialized
non-real-time readers; the producer does not wait on a reader mutex. Snapshot
freshness uses monotonic acquisition time, while messages/map observations retain
ROS timestamps. This is not a hard-real-time guarantee for the whole WBC loop.

Model, control, and evaluation settings are separate YAML files; startup rejects
invalid or contradictory values. Status/plan/action results have stable numeric
error codes, with textual diagnostics retained. Rebuild consumers after this
message definition change. See the [implementation plan](refactoring.md) and
[measured refactoring results](refactoring-results.md).

After building and sourcing the workspace, run:

```bash
python3 tools/validate_qr.py --suite unit --output artifacts/qr/unit-001
python3 tools/validate_qr.py --suite regression --output artifacts/qr/regression-001
```

`regression` runs unit tests, all four feet at 0/30/50 mm, the five existing fault
scenarios and legacy motion. `matrix` runs the separately qualified 100-trial
matrix. A short regression does not claim full M1 fault acceptance. Development
pushes and pull requests target `origin/RC2027`; RL runtime stays in its separate
repository.

## M1 closeout

The closeout adds acquisition-time guards, sustained swing-error detection, support-loss
fallback, and independent contact-point slip integration (`qr_simulation`). Physical
fault injection is separate from execution-core timestamp/deadline tests. Added physics
profiles cover mass/inertia ±5% and sole friction 0.35; they do not constitute sensor
noise validation or hardware calibration. See [closeout evidence](m1-closeout.md).

M1 closeout reached 99/100. M2 now includes body-following 16-step sequences, four-foot 30/50 mm platform transfers, bounded whole-sequence preview, a separate planning executor, spherical contact consistency and dense mechanical-candidate replay. The known IMU/joint-only sensor contract and hardware prohibition remain. Full nominal-obstacle dynamic execution is not certified; the wall path remains unresolved. See [current M2 record](m2-progress.md) and the [historical entry baseline](m2-entry.md).

Latest evidence: 9 repeated runs (144 steps), future-step rejection, a mid-sequence support-loss abort and M1 compatibility checks passed. The final 25-entry regression and a fresh 16-step platform run also passed. These are scoped results; full M2 acceptance remains false. [Results and open gates](m2-progress.md).

[New bounded coordinated-reference interface and remaining gates](coordinated-reference.md).
