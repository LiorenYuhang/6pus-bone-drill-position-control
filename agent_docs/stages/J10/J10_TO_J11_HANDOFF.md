# J10 -> J11 Handoff Document

Date: 2026-09-09
Authoritative Baseline: Frozen J8 (`v1.0-j8-active-servo-hardware-qualified`) + J9 Dual IMU + J10 Axis-Wise Operational Workspace Facts
Working Branch: `j9.1-clean`
Repository: `/home/lyapunov/bone_drill`
Workspace: `/home/lyapunov/bone_drill/bone_ws`

---

## 1. Executive Summary & J10 Final State

Stage **J10 (Task-Space Operational Safe Workspace Measurement)** is officially **COMPLETE**.

All six independent degrees of freedom (\(X, Y, Z, \text{Roll}, \text{Pitch}, \text{Yaw}\)) have completed single-axis empirical characterization on real hardware starting from the physical `VALID_HOME` state. All measurement records, CSV logs, and JSON metadata are finalized under `results/J10*` and synthesized in [`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](./J10_FINAL_SUMMARY.md).

### J10 Final Axis-Wise Measurement Facts (Audited)
- **X**: `[-45.75, +45.75] mm` *(pre-calibration absolute-endpoint-proximity censored; limiter ID3/ID4, margin 29 steps)*
- **Y**: `[-45.50, +49.00] mm` *(pre-calibration absolute-endpoint-proximity censored; limiter ID2/5 margin 31 steps, ID1/6 margin 30 steps)*
- **Z (Cartesian Direct)**: `[-20.00, +24.50] mm` relative to nominal HOME \(z_0 \approx 132.612\text{ mm}\) *(upper bound censored at 20 steps margin; direct command to 2000 avoided)*
- **Z (Including STORAGE_PARK)**: Approximately `[-24.983, +24.50] mm` *(lower endpoint verified via J7 joint-space PARK routine at \(Z_{\text{FK}} = 107.629\text{ mm}\))*
- **Roll**: `[-35.0, +37.5] deg` *(operational current/load failure before stroke limit; negative bracket `[-35.0, -37.5]°`, positive bracket `[+37.5, +40.0]°`, `overcurrent_ID4`)*
- **Pitch**: `[-30.0, +35.0] deg` *(operational current/load failure before stroke limit; negative bracket `[-30.0, -35.0]°`, positive bracket `[+35.0, +40.0]°`, `overcurrent_ID5/ID6`)*
- **Yaw**: `[-90.0, +90.0] deg` *(fully-qualified observed; +95.0° FK numerical execution failure / timeout; +105.0° configuration entanglement hazard; negative exploration stopped at -90.0° for geometric safety)*

---

## 2. Explicit Prohibitions for J11 and Downstream Work

> [!CAUTION]
> **MANDATORY ENGINEERING DIRECTIVES**:
> 1. **DO NOT MISTAKE J10-A NOMINAL VALUES FOR CALIBRATED GEOMETRIC BOUNDARIES**:
>    All task-space poses (\(X, Y, Z, \alpha, \beta, \gamma\)) reported in J10 are computed via nominal uncalibrated kinematic parameters (`Kinematics/params.m`). They are empirical operational indicators of the current assembly, **not** ground-truth geometric workspace boundaries.
> 2. **DO NOT CLAIM J10 VALUES AS CLINICALLY CERTIFIED LIMITS**:
>    J10 boundaries reflect test-harness stopping criteria and motor-driver operational trips. They must not be deployed as certified surgical constraints without clinical qualification.
> 3. **STRICT PROHIBITION ON YAW +105° RERUN**:
>    Yaw +105.0° produces physical 3D strut crossing and closed-chain mechanical binding. Direct command to Yaw \(\ge +97.5^\circ\) remains strictly barred.
> 4. **DO NOT ASSUME 6D COUPLED EQUIVALENCE**:
>    J10 characterized single-axis excitation from HOME only. Operational limits under combined translation and tilt (e.g. Roll at \(Z_{\text{max}}\)) were not measured in J10.

---

## 3. Scope of Stage J11: PID 3DOF World-Attitude Stabilization / Anti-Vibration

> [!NOTE]
> **STAGE RE-DEFINITION & DEFERRED CALIBRATION**:
> Actuator & Kinematic Parameter Calibration has been **DEFERRED** to a later dedicated stage (no current stage number assigned).
> Stage J11 is authoritatively redefined as:
> **$\mathbf{PID\ 3DOF\ World\text{-}Attitude\ Stabilization\ /\ Anti\text{-}Vibration}$**.

Stage J11 is strictly focused on:
- **Primary Objective**: Base receives manually applied Roll / Pitch / Yaw attitude disturbance; Platform maintains initial world attitude using Platform H30 feedback and 3-axis task-space PID.
- **Sensor Roles**:
  - **Platform H30**: Primary feedback sensor (`/pus6/imu_platform/data`).
  - **Base H30**: Disturbance measurement, logging, OFF/ON performance evaluation only (never enters J11 PID loop).
- **Control Structure**: Platform H30 -> $R_{WP, \text{meas}}$ -> $\mathrm{SO}(3)$ attitude error -> 3-axis task-space PID -> $\delta\phi_{\text{cmd}}$ -> $\mathrm{SO}(3)$ residual composition -> `/pus6/target_cartesian_pose` -> J8 frozen pipeline.
- **Translational Scope**: XYZ translation stabilization is explicitly out of scope.
- **Production Boundary**: J7/J8 production code (`pus6_motor_driver`, `pus6_control`, `pus6_kinematics`) remains strictly frozen (0 diff).

---

## 4. Deferred Actuator & Kinematic Calibration (Post-J11)

The physical identification of link lengths, anchor geometry, and slider zero offsets is deferred:
- **Base Anchor Geometry**: Calibrate base radius \(c\) and base pair angular separation \(\lambda\).
- **Platform Anchor Geometry**: Calibrate platform radius \(d\) and platform pair angular separation \(\mu\).
- **Strut Effective Lengths**: Calibrate individual effective rod lengths \(e_1 \dots e_6\).
- **Actuator Zero Offsets**: Calibrate individual prismatic slider zero offsets (\(q_{0,1} \dots q_{0,6}\)).
- **Post-Calibration Re-validation (J10B)**: Triggered only after the deferred calibration stage is executed and frozen.

