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

## 3. Scope of Stage J11: Actuator & Kinematic Parameter Calibration

Stage J11 is strictly focused on **identifying and validating the real kinematic and actuator parameters** of the assembled 6PUS parallel manipulator:

### 3.1 Core Calibration Objectives
- **Base Anchor Geometry**: Calibrate base radius \(c\) and base pair angular separation \(\lambda\) across all mounting points.
- **Platform Anchor Geometry**: Calibrate platform radius \(d\) and platform pair angular separation \(\mu\).
- **Strut Effective Lengths**: Calibrate individual effective rod lengths \(e_1 \dots e_6\) between spherical bearing centers.
- **Actuator Zero Offsets**: Calibrate individual prismatic slider zero offsets (\(q_{0,1} \dots q_{0,6}\)) relative to the mechanical bottom stops (`STORAGE_PARK`).
- **Step-to-Length Mapping & HOME Absolute Geometry**: Reconcile discrete step positions with true Cartesian metrics; determine true physical resting height \(z_0\).
- **Calibration Validation**: Validate forward kinematics residuals and closed-loop position repeatability against external precision metrology.

### 3.2 Explicitly Deferred Items (Not in J11 Scope)
The following tasks are **NOT** bundled into J11 and remain explicitly deferred:
- Dual-H30 IMU extrinsic mounting rotation calibration (\(R_{P}^{IP}\))
- Platform IMU 3D lever-arm vector estimation
- Base IMU coordinate frame alignment
- Sensor fusion / unified state estimation
- External disturbance force/torque reconstruction

---

## 4. J10 Measurements Requiring Re-Verification Post-Calibration

Following parameter updates in J11, the following empirical findings from J10 must be re-evaluated:
1. **Nominal HOME Resting Height**:
   Re-measure true resting height \(z_0\) (currently \(132.612\text{ mm}\) in nominal model).
2. **True Cartesian Endpoints**:
   Map the audited motor step endpoints (e.g. X+ ID3=1970 steps, Y+ ID2/5=1968 steps, Z+ ID1..6=1980 steps) through the **calibrated forward kinematics** to obtain true physical Cartesian limits in SI millimeters.
3. **Spatial Clearance Models**:
   Recompute 3D strut-to-strut minimum clearance profiles using calibrated anchor positions and true physical link outer envelopes.

---

## 5. Post-Calibration Stage J10B: Definition & Scope

### 5.1 Definition & Trigger Criteria
Stage J10B is strictly defined as **Post-J11 Calibrated Workspace Re-validation**. It will be initiated **only** after:
- Stage J11 actuator and kinematic parameter calibration is complete, validated, and frozen.

### 5.2 Scope of Stage J10B
1. **Calibrated Axis-Wise Boundary Re-measurement**:
   - Re-evaluate key single-axis boundaries using the calibrated kinematic model and true parameter set.
2. **Re-confirmation of Endpoint-Adjacent Results**:
   - Re-confirm X, Y, and Z endpoint-adjacent limits with true link lengths and anchor geometry.
3. **Calibrated Task-Space Attitude Validation**:
   - Re-confirm Roll, Pitch, and Yaw calibrated task-space values against external ground truth.
4. **Targeted Coupled Workspace Slices**:
   - Measure a small, targeted number of coupled-workspace slices (e.g. tilt envelopes at varying vertical elevations \(Z_{\text{low}}, Z_{\text{mid}}, Z_{\text{high}}\)).

### 5.3 Explicit Non-Scope Exclusions for J10B
The following activities are **NOT** part of J10B:
- Dynamics parameter identification
- Trajectory acceleration profiling / optimization
- Anti-overcurrent controller development or current-limit tuning
- Clinical certification or clinical-grade operational gate definitions
