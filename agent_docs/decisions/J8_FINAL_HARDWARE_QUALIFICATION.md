# 6PUS Stage J8: Continuous Cartesian Servo & Design-B Feedback Hardware Qualification Report

> **Authoritative Baseline Record**: `v1.0-j8-active-servo-hardware-qualified`  
> **Status**: `J8 functional hardware qualification: PASS`  
> **Date**: 2026-08-28  

---

## 1. J8 Architecture Overview

Stage J8 establishes continuous Cartesian trajectory streaming on the 6PUS parallel manipulator at 50 Hz, utilizing Design-B accompanying telemetry feedback for closed-loop kinematic observation.

```text
Cartesian Target Pose (50 Hz)
  │  [/pus6/target_cartesian_pose]
  ▼
Analytical Inverse Kinematics (pus6_kinematics_node)
  │  [/pus6/target_joint_steps]
  ▼
Stateful Jerk-Limited Trajectory Shaper (pus6_control_node)
  │  [/pus6/command_joint_steps, LVW]
  ▼
Pus6MotorDriverNode (Design-B 50Hz Servo Mode)
  │  [Batch-atomic 6-axis 0x29 write + inline RX feedback]
  ▼
RS485 Half-Duplex Bus (921600 8N1)
  │  [6x 0x29 Target/Feedback Packets per 20ms cycle]
  ▼
TimedJointFeedback (pus6_msgs/msg/TimedJointFeedback)
  │  [Dedicated SingleThreadedExecutor]
  ▼
FixedLagJointSynchronizer (pus6_kinematics)
  │  [Strict zero-extrapolation timeline reconstruction, tau = 160ms]
  ▼
Pus6FkNode (Levenberg-Marquardt Analytical Jacobian FK @ 50Hz)
  │  [/pus6/fk/state]
```

---

## 2. Design-B Accompanying Feedback Contract

- **ACTIVE Mode Execution**: The primary runtime feedback in ACTIVE mode is derived directly from the `0x29` write-response frames (`StatusFeedback`), which are returned by each actuator after writing the target position.
- **Critical Path Hygiene**: Routine blocking `0x30` query scans are completely excluded from the 50 Hz active control loop, eliminating interleaving UART transaction jitter.
- **Protocol Baseline**: [`protocol.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py) remains byte-for-byte identical to the `v1.0-j7-servo-hardware-frozen` baseline.

---

## 3. Two-Tier Watchdog Architecture

To decouple UART packet loss from non-real-time Linux scheduler timing jitter, the safety watchdog is separated into two distinct tiers:

1. **Tier 1: Per-Executed-Cycle 6/6 Feedback Completeness (Primary Communication Loss Protection)**
   - Every executed 0x29 Servo cycle expects exactly 6 valid responses (ID1 to ID6).
   - Tracks consecutive incomplete cycles across axes.
   - If $N = 4$ consecutive executed cycles fail to receive complete feedback on any axis, the driver safely trips communication fault (`0x1A STOP`).

2. **Tier 2: Servo Scheduler & Deadline Protection (Fail-Safe Scheduler Ceiling)**
   - Monitors the wall-clock gap between executed Servo cycles.
   - The threshold ($263.324\text{ ms}$) serves as an emergency upper ceiling for operating system stalls, ensuring the scheduler has not frozen indefinitely while tolerating benign Python GC or executor jitter ($\sim 20\text{--}40\text{ ms}$).
   - **Important Note**: No hard real-time guarantees or RT-preempt certification are claimed for the standard Linux host.

---

## 4. Centralized PARK / HOME Startup Contract

Located centrally in [`src/pus6_motor_driver/pus6_motor_driver/safety.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py):

| Parameter | Value | Semantics |
| :--- | :--- | :--- |
| `STORAGE_PARK_RAW_STEP` | $0\text{ steps}$ ($25.000\text{ mm}$) | Mechanical bottom resting index |
| `OPERATING_HOME_RAW_STEP` | $1000\text{ steps}$ ($50.000\text{ mm}$) | Mid-stroke kinematics operating origin |
| `FEEDBACK_POSITION_TOLERANCE_STEPS` | $\pm 5\text{ steps}$ ($\pm 0.125\text{ mm}$) | Provisional zero-reference measurement band |
| `PARK_MEASUREMENT_BAND` | $[-5, +5]\text{ steps}$ | Valid measurement band for cold-start and PARK |
| `HOME_MEASUREMENT_BAND` | $[995, 1005]\text{ steps}$ | Valid measurement band for settled HOME |
| `COMMAND_SAFETY_BOUNDS` | $[0, 2000]\text{ steps}$ | Commanded targets strictly $\ge 0$ (no negative commands) |

### State Classification Logic
- **`VALID_PARK`**: All 6 axes healthy (`fault == 0`), multi-sample stable (drift $\le 2$ steps), and all feedback positions $\in [-5, +5]\text{ steps}$.
  - Raw feedback (e.g. `[-4, -5, -2, -3, -4, 0]`) is preserved unmodified in telemetry.
  - Motion planning seed is set to **strictly `[0, 0, 0, 0, 0, 0]`**, guaranteeing Cycle 0 commands $q \ge 0$.
- **`VALID_HOME`**: All 6 axes $\in [995, 1005]\text{ steps}$, seed set to `[1000, 1000, 1000, 1000, 1000, 1000]`.
- **`VALID_IN_ENVELOPE`**: All 6 axes strictly $\in [0, 2000]\text{ steps}$, seed set to measured $q$.
- **`MIXED_UNCLASSIFIED` / `INVALID_OUT_OF_RANGE`**: Arbitrary mixed negative states are **strictly rejected** with `SafetyViolation`; generic clamping is forbidden.

---

## 5. 3-Round Autonomous Hardware Qualification

Executed on real hardware across 3 consecutive rounds (READ_ONLY $\to$ Authorized HOME $\to$ 20s Active Hold $\to$ Authorized PARK):

- **Total 0x29 Motion Cycles**: $9,459\text{ cycles}$
- **0x29 Feedback Responses**: $56,754 / 56,754$ (**100.0% Completeness**)
- **Hardware Faults**: 0
- **UART Timeouts**: 0
- **Driver STOP Triggers**: 0
- **Cold-Start PARK Distribution**: Bounded within $[-5, +1]\text{ steps}$ ($[-0.125, +0.025]\text{ mm}$).

---

## 6. Final 9-Phase Real-Hardware Video Demonstration

Demonstration trajectory executed via [`scripts/run_j8_final_video_demonstration.py`](file:///home/lyapunov/bone_drill/bone_ws/scripts/run_j8_final_video_demonstration.py):

| Phase | Description | Target / Amplitude | Result |
| :--- | :--- | :--- | :--- |
| **Phase 0** | **Cartesian HOME Hold** | $z_0 = 132.612\text{ mm}$ (3.0s) | **PASS** |
| **Phase 1** | **X Translation Showcase** | $\pm 8.0\text{ mm}$ | **PASS** ($\Delta X = -0.138\text{ mm}$) |
| **Phase 2** | **Y Translation Showcase** | $\pm 8.0\text{ mm}$ | **PASS** ($\Delta Y = -0.258\text{ mm}$) |
| **Phase 3** | **Z Heave Translation Showcase** | $\pm 8.0\text{ mm}$ | **PASS** ($\Delta Z = -0.044\text{ mm}$) |
| **Phase 4** | **Roll Rotation Showcase** | $\pm 8.0^\circ$ | **PASS** ($\Delta\text{Roll} = -0.092^\circ$) |
| **Phase 5** | **Pitch Rotation Showcase** | $\pm 8.0^\circ$ | **PASS** ($\Delta\text{Pitch} = -0.087^\circ$) |
| **Phase 6** | **Yaw Rotation Showcase** | $\pm 8.0^\circ$ | **PASS** ($\Delta\text{Yaw} = -0.404^\circ$) |
| **Phase 7** | **Combined 6-DoF Continuous Motion** | $XYZ \pm 5.0\text{ mm}$, $\text{Roll/Pitch} \pm 3.0^\circ$, $\text{Yaw} \pm 5.0^\circ$ (20.0s @ 0.1Hz) | **PASS** |
| **Phase 8** | **Final HOME Return & Repeatability** | $z_0 = 132.612\text{ mm}$ (4.0s) | **PASS** ($dX{=}+0.115, dY{=}+0.237, dZ{=}+0.045\text{ mm}$) |

### Final Metrics:
- **Total Duration**: 137.128 s
- **Total Executed Servo Cycles**: 6,898 cycles
- **Configured / Measured Rate**: 50.0 Hz / 50.30 Hz sustained
- **Feedback Completeness**: 41,388 / 41,388 (100.0%)
- **Worker Batch Duration**: Mean = 9.76 ms, P95 = 15.82 ms, Max = 29.40 ms
- **Faults / Timeouts / Latched STOP**: 0 / 0 / None (HEALTHY)

---

## 7. Scope and Explicit Non-Claims

- **J8 functional hardware qualification: PASS**
- **Explicit Limitations**:
  - Hard real-time certification is **NOT** claimed.
  - Analytical passive spherical/universal (U/S) mechanical joint-limit collision verification is **NOT** included in J8.
  - Forward kinematics output provides pipeline state reconstruction and does **NOT** constitute independent external optical ground truth.
