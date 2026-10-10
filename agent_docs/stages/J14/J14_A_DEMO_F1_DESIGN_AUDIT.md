# J14-A-Demo-F1 design and evidence boundaries

Baseline: user Feedforward active-function PASS, shutdown FAIL/HOLD. Original raw evidence is `results/j14/demo/feedforward_20261010_165632_313928`; no raw file was rewritten. Closing the two previously orphaned IMUs naturally appended their rclcpp signal_handler messages to their already-open original logs.

## Directly observed shutdown failure

Residuals are Platform H30 node PID 49596 and Base H30 node PID 49597. They owned ttyACM1 and ttyACM0 respectively. Launcher-owned `ros2 run` wrapper PIDs were 49586/platform and 49585/base. Each received leader-only SIGINT, waited 3 seconds, then wrapper-only SIGTERM; wrapper returncode -15, leaf exit code NOT RECORDED. Local ros2run API confirms it assumes the subprocess also receives Ctrl+C through the process group; the Demo isolated each role with start_new_session but signalled only its leader. The leaves remained alive, independently of IK/control's exit-code-1 context exceptions. One SIGINT to each identity-checked, historical Demo-owned leaf released it in approximately 0.151 seconds. New evidence is `historical_owned_imu_release.json`; this closed sensors only, with no motor/UART actuator action.

Original cleanup had no HOME target: controller -> relay -> entire robot launch (Parent close -> Child pause) -> IMU wrappers. Its launch leader exit code 0 did not mean all ROS nodes or descendants exited cleanly.

## Reuse rather than second motion implementation

J7 standalone HOME/PARK opens and owns UART, writes servo targets directly and ends in Pause. Running it alongside the current driver would introduce a second owner and bypass the current trajectory shaper. It is audited but not invoked.

J8 Cartesian HOME pose is zero XYZ lateral offsets, geometry nominal height, identity orientation. J10 uses that same frozen J8/J9 chain, exposes `WorkspaceMeasurementHarness.publish_target_pose`, `_home_feedback_in_band`, `_home_timeout_from_feedback` and a five-consecutive-new-feedback default. Demo imports these existing methods/predicates/default rather than copying a controller or tuning constants. The unbound publish method receives the existing runner node: its original target publisher is retained, no new publisher or UART owner is created.

HOME completion: all six actual raw feedback steps in the existing 995..1005 inclusive band, five consecutive updated packets, every per-axis receive timestamp newer than HOME entry. Timeout is the imported J10 formula: max(1.0, maximum observed step distance/60 + 1.5) seconds. HOME entry also reuses the existing 1-second feedback hard-stop constant and 1500-mA overcurrent constant. No HOME on unsafe/missing feedback. These are Demo stop admission checks, not changes to the underlying driver thresholds.

Normal SIGINT after healthy ACTIVE: cancel runner timer -> HOME through same Cartesian publisher/IK/shaper/Parent/Child -> fresh feedback confirmed -> signal existing Parent for original Child Pause/joins -> stop controller host -> relay -> direct IMU leaves -> remaining launch nodes -> serial/process audit. HOME timeout aborts motion requests, requests Pause and records a failed Demo, then completes resource cleanup. Second interrupt during HOME requests safety stop.

Fault/unexpected exit/SIGTERM/startup abort: no HOME request; stop controller target output and request existing Parent close/Pause before waiting on other components. Child retains its own independent safety stop/lease logic.

## Stage-owned lifecycle repair

Original production console main loops can shut their context through ROS signals while the executor is waiting; IK/control then unconditionally call shutdown again, and Parent can escape with ExternalShutdownException. F1 does not edit these production sources or patch global rclpy functions. The existing launch receives a supported launch prefix: Demo-owned node host instantiates the same existing classes/factory with exactly the original ROS argument/parameter stack, uses SingleThreadedExecutor, and owns signals through a stop event. Nodes are destroyed while context is still alive, then the executor and context shut down once. Parent still uses the original create_driver factory and ProcessClient/Child/Bridge; no new driver implementation.

Actual IMUs are launched by their existing installed executable path directly, using identical ROS remaps/parameters. The Popen PID now identifies the sensor node, avoiding ros2 run ownership indirection. No H30 source modification.

## Test boundaries

All F1 executions are explicit hardware_mode mock. Normal return tests deliberately inject only software IMU amplitude .02 rad, to begin outside the existing HOME band. Mock failure injection uses a guarded test-only Child bootstrap, before any serial access, and substitutes FakeBus.write: hardware_fault sets ID1's FakeBus fault; home_timeout holds FakeBus measured positions at 1020. No fabricated real measurements and no threshold relaxation. Unexpected controller/relay exits are externally injected SIGTERM on those owned Mock processes, not blanket native-crash coverage.

The original smoke already passed HOME and context cleanup but started within tolerance. That attempt remains insufficient proof of an actual return. The early request-level injection failed because normal servo feedback arrives via Mock write ACKs; corrected write-level injection is recorded separately. One targeted-test attempt collided with an active Demo's exclusive lock, was correctly rejected, and remains preserved; subsequent tests run sequentially.

`safe_shutdown` reports Pause/resource/context cleanup, allowing the controller's intentional exit 1 when HOME/fault handling failed. `demo_shutdown_pass` additionally requires successful normal HOME/session outcome. Failure sessions stay returncode 1 and Demo shutdown FAIL; policy/cleanup verification PASS does not turn failed HOME into a successful HOME.
