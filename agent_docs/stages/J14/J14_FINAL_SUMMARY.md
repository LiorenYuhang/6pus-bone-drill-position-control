# J14-A Final Summary: Long-Duration Runtime & Communication Reliability Qualification

**Stage Identity**: J14-A (Long-Duration Runtime & Communication Reliability)
**Status**: **CLOSED / FROZEN**
**Final Release Tag**: `v1.0-j14-a-three-process-runtime-reliability-qualified`
**Date**: 2026-10-10
**Authoritative Evidence Baseline**: Real Hardware Qualified (Q3-R1D Classical PASS, Q3-R2 LADRC PASS, Demo Human Hardware Confirmation PASS)

---

## 1. Stage Identity & Objectives

J14-A was initiated following the completion of J13 (First-Order Linear ADRC Residual Control). During J13 long-duration testing, sporadic communication stalls and scheduling jitter over the RS485 bus were observed (max scheduler gap $\\approx 370.6\\text{ ms}$, max feedback age $\\approx 352.4\\text{ ms}$), limiting continuous multi-minute operation.

**Primary Objectives**:
1. Diagnose and eliminate low-level communication blocking and scheduler starvation;
2. Design and deploy a production-grade, multi-process decoupled driver architecture;
3. Qualify long-duration static and active operation on both mock and real 6PUS hardware;
4. Establish fully verified, one-click demonstration entrypoints with safe, controlled lifecycles;
5. Freeze the production runtime baseline and maintain all established safety invariants.

---

## 2. Original Runtime Problem

In pre-J14 architectures, motor control, RS485 serial communication, and ROS 2 middleware callbacks operated within a single multi-threaded process. Under prolonged execution:
- Serial I/O system calls and thread pool scheduling contention delayed the event loop;
- Blocking I/O accumulated downstream feedback latency, inflating feedback age beyond safety limits;
- Intermittent OS scheduler latency and serial transaction jitter caused cyclic communication stalls;
- Watchdog trips triggered sudden motor pause, interrupting long-duration operations.

---

## 3. Measurement Boundary Investigation

Early diagnostics revealed that naive telemetry instrumentation introduced its own measurement distortion:
- Synchronous file writes and diagnostic queues during control loops induced observable timing jitter;
- Wrapping serial read/write calls with microsecond profiling exacerbated thread switching overhead;
- Determining the true bus timing required isolating measurement probes from the critical real-time execution path.

---

## 4. Q1 Instrumentation Effect

During initial 120s and 300s static testing (Q1), a periodic latency spike ($\\approx 67.6\\text{ s}$ interval, reaching up to $180\\text{ ms}$) was observed. Forensic analysis isolated this artifact to the serial diagnostic wrapper itself:
- Periodic queue observation and queue-after diagnostic processing in serial wrappers introduced post-write long tails;
- The stall was an instrumentation artifact rather than physical RS485 bus failure.

---

## 5. Q1F Fix (Diagnostic Isolation)

To eliminate the instrumentation effect, Q1F implemented strict diagnostic isolation:
- High-frequency diagnostic counters were separated from serial I/O threads;
- Diagnostic state tracking was bounded and separated from serial I/O threads;
- Diagnostic flushes were decoupled from critical 50 Hz control ticks.
Result: The diagnostic long-tail was confirmed absent in subsequent runs.

---

## 6. Three-Process Architecture (Production Implementation)

J14-A established a production-grade three-process decoupled architecture to permanently isolate serial I/O and feedback streaming from ROS 2 node execution:

1. **Parent Node (`ProductionDriverNode` in `production_driver.py`)**:
   - Manages high-level ROS 2 lifecycle, parameter configuration, and target ingress;
   - Executes via **`SingleThreadedExecutor`** to guarantee deterministic inline execution;
   - Communicates with child processes via non-blocking domain sockets and named pipes;
   - Enforces the 1000 ms Parent lease.
2. **UART Child (`j14_process_uart.py` / `j14_uart_child_main.py`)**:
   - **Sole owner of the physical RS485 UART port** (`/dev/ttyUSB0` at 921600 baud);
   - Operates a dedicated real-time 50 Hz control loop;
   - Receives target positions from Parent; applies 80-step reconnect limits, position envelopes, and 1500 mA current protection;
   - Directly executes 6-axis hardware Pause on fault or lease expiration.
3. **Feedback Bridge (`j14_feedback_bridge.py` / `j14_feedback_bridge_main.py`)**:
   - **Sole motor feedback IPC consumer** (consumes decoded feedback frames from UART Child over `AF_UNIX SOCK_SEQPACKET` at 50 Hz);
   - Formats and publishes ROS 2 `/joint_states`, `/pus6/joint_feedback/timed`, and `/pus6/joint_command/timed` topics with microsecond-level hardware and DDS timestamping;
   - Does not own any serial port, does not directly read IMUs (managed independently by `h30_imu` ROS 2 nodes), and does not use POSIX SHM or RingBuffer.

---

## 7. Startup Coordination & Coordinated Handshake

To prevent communication deadlocks during initialization:
- Child processes launch and initialize serial ports before Parent commands are admitted;
- Inter-process handshake tokens verify child process readiness;
- Startup handshake packet-loss fixes ensure reliable state transition from `DISARMED` to `READY` to `ACTIVE`.

---

## 8. Q2 Static Qualification (120s, 300s, 600s)

Static qualification evaluated multi-minute communication stability without Cartesian motion:
- **120s Static Run (`j14_a2_q2_r1_static_120s`)**: PASS, 0 timeouts, 0 serial disconnects.
- **300s Static Run (`j14_a2_q2_r2_static_300s`)**: PASS, 0 timeouts, 0 serial disconnects.
- **600s Static Run (`j14_a2_q2_r3_static_600s`)**:
  - Executed exactly 29,995 cycles (`cycle_seq` 0 to 29994, ~10 minutes continuous runtime);
  - Recorded 3 deadline reset / re-anchor events ($\approx 5$ estimated skipped slots: 1 + 2 + 2);
  - Cycle wall max: 68.375 ms (at Cycle 23487);
  - Cycle interval max: 68.541 ms (between Cycle 23487 and 23488);
  - 0 communication timeouts, 0 motor faults, 0 IPC packet drops (missing child to bridge: 0), 0 IPC duplicate deliveries;
  - Old Q1 $\approx 67.6\text{ s}$ diagnostic long-tail was absent;
  - **Verdict**: Engineering Qualified (PASS). The run verified robust recovery and multi-minute stability, while transparently documenting the minor re-anchor events.

---

## 9. Static vs. Active Divergence

Passing static qualification did not automatically guarantee active qualification:
- Under static conditions, motor position commands remain constant, and the command pipeline operates with minimal data variance;
- Under active Cartesian motion, high-rate dynamic Cartesian pose updates, IK solver loops, Dual-IMU subscriptions, and feedforward calculations heavily load the host process;
- Active mode introduced concurrent subscription workloads that exposed architectural sensitivities in the Parent executor.

---

## 10. Q3-R1C Parent Lease Failure

During the first active real-hardware test of Classical control (Q3-R1C):
- Preflight, Dual-IMU, and IK successfully engaged;
- Motors began tracking dynamic trajectory (`[1000, 999, 1000, 999, 998, 1000]`);
- At cycle 459, the UART Child latched emergency stop: `parent_lease_expired`;
- Child/Bridge cycle count froze at 459 / 459, triggering safety pause;
- **Diagnosis**: The Parent process failed to renew its IPC lease to the UART Child within the 1000 ms lease window.

---

## 11. Q3J Executor Investigation

Forensic analysis of the Parent process isolated the root cause of lease expiration:
- **CONFIRMED**: The Parent node was executing under ROS 2 `MultiThreadedExecutor(num_threads=3)`;
- **CONFIRMED**: Under active command subscription workloads, incoming topic traffic provoked continuous ROS 2 wait-set rescanning within the thread pool, starving the IPC lease transmission thread;
- **STRONGLY SUPPORTED**: This wait-set scanning contention directly precipitated the R1C `parent_lease_expired` trip;
- **UNKNOWN**: Deep CPython GIL thread contention versus Fast DDS native wake leaf causes could not be independently disentangled, nor was disentangling necessary once the architecture was addressed.

---

## 12. SingleThreadedExecutor Fix

The architectural fix resolved the qualified Parent IPC starvation mechanism by executing inline on SingleThreadedExecutor:
- `production_driver.py` enforces **`SingleThreadedExecutor`** for `ProductionDriverNode`:
  ```python
  executor = (SingleThreadedExecutor() if isinstance(node, ProductionDriverNode)
              else MultiThreadedExecutor(num_threads=3))
  ```
- All ROS subscription callbacks execute inline to update the latest target buffer;
- IPC lease renewal executes deterministically without thread scheduling competition;
- Parent CPU utilization dropped by ~48%, and IPC gaps normalized to $< 20\text{ ms}$.

---

## 13. Active Mock Validation

Following the SingleThreadedExecutor fix, active mock regression was conducted:
- **Active Mock 60s (`j14_a2_q3j_active_mock_60s`)**: PASS, 0 lease expirations, 0 timeouts.
- **Active Mock 300s (`j14_a2_q3j_active_mock_300s`)**: PASS, 0 lease expirations, 0 timeouts.
- **Static Mock 60s Regression**: PASS.

---

## 14. Q3-R1D Classical Hardware Qualification

Real-hardware verification of Classical P/PD control:
- **Run Directory**: `results/j14/j14_a2_q3_r1d_classical_60s/`
- **Duration**: 60 seconds nominal runtime:
  - Driver session: 3,529 cycles (`child_cycles.jsonl`)
  - Controller total: 2,999 cycles (`telemetry.csv`)
  - Controller ACTIVE: 2,945 cycles (WAITING_FOR_PREFLIGHT: 3, CAPTURING_REFERENCE: 50, READY: 1)
- **Results**:
  - Hard stops: 0
  - Motor faults: 0
  - Communication timeouts: 0
  - Parent lease expirations: 0
  - Base IMU age: min 0.26 ms, max 56.18 ms, avg 1.36 ms (0 samples > 100 ms)
  - Platform IMU age: min 0.14 ms, max 57.83 ms, avg 2.46 ms (0 samples > 100 ms)
- **Verdict**: **ACTIVE HARDWARE PASS**.

---

## 15. Q3-R2 LADRC Hardware Qualification

Real-hardware verification of 3-axis First-Order LADRC:
- **Run Directory**: `results/j14/j14_a2_q3_r2_ladrc_60s/`
- **Duration**: 60 seconds nominal runtime:
  - Driver session: 3,539 cycles (`child_cycles.jsonl`)
  - Controller total: 2,997 cycles (`telemetry.csv`)
  - Controller ACTIVE: 2,942 cycles (WAITING_FOR_PREFLIGHT: 4, CAPTURING_REFERENCE: 50, READY: 1)
- **Results**:
  - Hard stops: 0
  - Motor faults: 0
  - Communication timeouts: 0
  - Parent lease expirations: 0
  - Base IMU age: min 0.15 ms, max 65.88 ms, avg 2.22 ms (0 samples > 100 ms)
  - Platform IMU age: min 0.38 ms, max 66.80 ms, avg 2.56 ms (0 samples > 100 ms)
  - LESO observer states ($z_1, z_2$): Bounded and numerically stable ($z_1, z_2$ within valid observation range throughout)
- **Verdict**: **ACTIVE HARDWARE PASS**.

---

## 16. Final Three Demo Entrypoints

Three standardized, user-facing shell scripts provide one-click execution without requiring manual ROS environment sourcing:
1. `bone_ws/scripts/run_j14_demo_feedforward.sh`: Base $SO(3)$ Pure Feedforward;
2. `bone_ws/scripts/run_j14_demo_feedforward_pd.sh`: Base $SO(3)$ Feedforward + Classical Task-Space P/PD;
3. `bone_ws/scripts/run_j14_demo_feedforward_ladrc.sh`: Base $SO(3)$ Feedforward + 3-Axis First-Order LADRC.

---

## 17. Startup HOME / Shutdown HOME Lifecycle

To prevent mechanical step loss and ensure safe operator workflow:
- **Startup Sequence**:
  1. Driver / Feedback Bridge initialization;
  2. Timed 6-axis feedback verification;
  3. Pre-motion position check: if motors deviate from `[1000]*6` within tolerance, Startup HOME automatically executes a jerk-limited return;
  4. Dual-IMU preflight verification ($< 100\\text{ ms}$ age check);
  5. Controller state transition to `ACTIVE`.
- **Shutdown Sequence (on Ctrl+C / Operator Stop)**:
  1. Operator interrupt captured gracefully;
  2. Trajectory shaper smoothly commands robot back to `OPERATING_HOME` (`[1000]*6`);
  3. Controlled 6-axis Pause issued (ACK 6/6 confirmed);
  4. Serial port cleanly released;
  5. Child and bridge processes cleanly terminated via SIGINT;
  6. Residual processes verified: **0**.

---

## 18. Final Demo Hardware Confirmation

All three official Demo entrypoints have been executed on the physical 6PUS robot:
- `run_j14_demo_feedforward.sh`: **PASS**
- `run_j14_demo_feedforward_pd.sh`: **PASS**
- `run_j14_demo_feedforward_ladrc.sh`: **PASS**
- **Evidence Type**: **HUMAN HARDWARE CONFIRMATION** (confirmed directly on physical hardware by the operator).

---

## 19. F3 Base IMU USB Incident

During an intermediate execution of `run_j14_demo_feedforward_ladrc.sh` (`results/j14/demo/feedforward_ladrc_20261010_180702_906278`), the system triggered a safety stop with `Base IMU age timeout (>100ms)`.

**Forensic Audit Findings**:
- **CONFIRMED**: Device serial `5B32028000` (Base IMU) experienced a physical USB disconnect at monotonic 6103.650 s, exactly 82.6 ms after its last data packet.
- **CONFIRMED**: Linux kernel logged downstream Hub Port 3 hardware disablement: `usb 3-3.4-port3: disabled by hub (EMI?), re-enabling...`.
- **CONFIRMED**: Device re-enumerated $\\approx 320\\text{ ms}$ later as `ttyACM2` (previously `ttyACM0`).
- **CONFIRMED**: The 100 ms age watchdog functioned strictly as designed, tripping 17.4 ms after physical disconnect and executing immediate 6-axis hardware pause.
- **CONFIRMED**: Platform IMU (`5B0B029000`), motor driver, and controller scheduling operated nominally throughout the run.
- **UNKNOWN**: The precise physical/electrical trigger that induced the external hub port disablement (connector vibration, cable strain, transient power dip, or external noise). Kernel text `(EMI?)` is a diagnostic heuristic printk, not confirmed root cause.
- **Categorization**: **NON-BLOCKING HARDWARE RELIABILITY FOLLOW-UP**.

---

## 20. Frozen Production Architecture

The frozen production codebase embodies:
- **Parent Node**: `SingleThreadedExecutor` inline event execution;
- **UART Child**: Dedicated process, single UART owner;
- **Feedback Bridge**: Dedicated process, single feedback IPC consumer;
- **Compatibility Relay**: External adapter only, zero direct UART ownership.

---

## 21. Frozen Safety Parameters

| Parameter | Frozen Value | Verification Location |
| :--- | :--- | :--- |
| Parent IPC Lease Timeout | `1000 ms` | `j14_process_uart.py` |
| Servo Cycle Rate | `50 Hz` ($20\\text{ ms}$) | `j14_process_uart.py` |
| Overcurrent Protection Limit | `1500 mA` | `pus6_motor_driver/safety.py` |
| Reconnect Jump Limit | `80 steps` | `j7_real_uart_adapter.py` |
| Base / Platform IMU Watchdog | `100 ms` | `j13_ladrc_shadow_runner.py` |
| Position Travel Bounds | Hardware & software envelope | `pus6_control` / `pus6_motor_driver` |
| Emergency Stop Protocol | 6-axis Pause (ACK 6/6) | `j7_real_uart_adapter.py` |

---

## 22. Known Non-Blocking Follow-ups

1. **USB Topology Isolation**: Connect Base IMU directly to a host root port, bypassing the daisy-chained external hub;
2. **C++ Driver EOF Handling**: Update `h30_imu_node.cpp` to recognize `read() == 0` as EOF and trigger auto-reconnection via stable udev symlinks;
3. **Pytest Context Isolation**: Separate ROS 2 context lifetimes across unit test suites to eliminate concurrency interference;
4. **Mock Child Long-Tail Observation**: Investigate $\\approx 110\\text{ ms}$ mock scheduling tail under simulated heavy load.

---

## 23. Final Verdict

**J14-A STATUS: CLOSED / FROZEN**
All functional, architectural, safety, and experimental requirements have been fully verified and locked. J14-A is formally frozen as the foundational runtime reliability baseline.
