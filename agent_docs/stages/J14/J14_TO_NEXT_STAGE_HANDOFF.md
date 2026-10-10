# J14-A to Next Stage Engineering Handoff Specification

**From**: J14-A (Long-Duration Runtime & Communication Reliability Qualification)
**To**: Next Research & Application Stage (e.g., Eccentric-Payload Dynamic Tracking / Disturbance Rejection)
**Status**: **FROZEN / BASELINE LOCKED**
**Date**: 2026-10-10
**Release Tag**: `v1.0-j14-a-three-process-runtime-reliability-qualified`

---

## 1. Frozen Baseline Invariants (Do-Not-Change Rules)

The following architectural and operational invariants are strictly frozen. Modifying any of these parameters requires a formal regression re-qualification:

### 1.1 Process Architecture Invariants
- **Parent Node**: Must execute using `SingleThreadedExecutor` for `ProductionDriverNode`. Do NOT re-introduce threaded executor variants without re-qualifying IPC lease continuity under dynamic topic loads.
- **UART Child**: Sole owner of physical RS485 UART port (`/dev/ttyUSB0` at 921600 baud). No other process, node, or thread may open, read, or write to the motor serial port directly.
- **Feedback Bridge**: Sole motor feedback IPC consumer (`AF_UNIX SOCK_SEQPACKET` at 50 Hz). Does not own UART, does not directly read IMUs, and does not use POSIX SHM.
- **Compatibility Relay**: Strictly an external adapter for legacy topic translation; prohibited from directly acquiring UART or command authorities.

### 1.2 Safety Envelope Invariants
- **Parent IPC Lease**: Exactly `1000 ms`. Failure to renew within this window triggers immediate child-side autonomous latch stop (`parent_lease_expired`).
- **Servo Control Rate**: Exactly `50 Hz` ($20\text{ ms}$).
- **Overcurrent Protection**: Host limit `MAX_CURRENT_MA = 1500` (strictly aligned with actuator hardware protection register).
- **Reconnect Jump Limit**: Maximum `80 steps` / axis.
- **IMU Age Watchdog**: Maximum `100 ms` for both Base and Platform IMU streams.
- **Emergency Stop Action**: 6-axis hardware Pause (ACK 6/6 confirmed).

### 1.3 Controller Algorithm Invariants
- **Classical P/PD Gain Set**:
  - $K_p = [0.05, 0.05, 0.025]$
  - $K_d = [0.0, 0.0015, 0.0015]\,\text{s}$
  - $K_i = [0.0, 0.0, 0.0]$
  - $f_c = 5.0\,\text{Hz}$
  - Residual Clamp = $\pm 0.5^\circ$ / axis
- **3-Axis First-Order LADRC Parameters**:
  - Roll: $b_0 = 20.0$, $\omega_c = 0.8\,\text{rad/s}$, $\omega_o = 2.5\,\text{rad/s}$
  - Pitch: $b_0 = 10.0$, $\omega_c = 1.0\,\text{rad/s}$, $\omega_o = 4.0\,\text{rad/s}$
  - Yaw: $b_0 = 6.0$, $\omega_c = 0.25\,\text{rad/s}$, $\omega_o = 1.2\,\text{rad/s}$
  - Residual Clamp = $\pm 0.5^\circ$ / axis

---

## 2. Official Demonstration Entrypoints

All operational runs must utilize the verified J14 Demo scripts, which manage the complete preflight, Startup HOME, Dual IMU qualification, active running, and safe controlled Shutdown HOME lifecycle:

```bash
# 1. Base SO(3) Pure Feedforward
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward.sh

# 2. Base SO(3) Feedforward + Classical Task-Space P/PD
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward_pd.sh

# 3. Base SO(3) Feedforward + 3-Axis First-Order LADRC
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward_ladrc.sh
```

---

## 3. Known Non-Blocking Technical Debt (Follow-Up Checklist)

The following items are documented engineering debts handed off for future refinement:

1. **Base IMU External Hub Isolation**:
   - Base IMU is currently connected via an external Genesys Logic 4-Port USB Hub.
   - *Recommendation*: Connect directly to a native USB Root Port on the host industrial PC and apply cable strain relief.
2. **`h30_imu_node.cpp` EOF Disconnect Handling**:
   - Current driver treats `read() == 0` (EOF) as silent no-op.
   - *Recommendation*: Explicitly mark `fd = -1` on EOF, close descriptor, and attempt dynamic reconnection via stable udev by-id paths.
3. **ROS 2 Context Concurrency in Unit Tests**:
   - Concurrent test execution can collide on ROS 2 Jazzy global context lifecycle.
   - *Recommendation*: Run unit tests in isolated subprocesses.

---

## 4. Re-Qualification Criteria (When Re-Testing is Mandatory)

If subsequent development alters any of the following, a full re-qualification suite must be executed before deployment:

| Change Scope | Mandatory Verification Suite |
| :--- | :--- |
| **Driver IPC or Executor** | 600s Static Mock + 300s Active Mock + 60s Real Hardware ACTIVE |
| **Safety Thresholds** | Bounds verification + E-stop Pause ACK validation |
| **Kinematics / Geometry** | IK/FK regression + 6-axis preflight workspace check |
| **Controller Gains / ESO** | Shadow mode passive run + 60s hardware trial with telemetry recording |
