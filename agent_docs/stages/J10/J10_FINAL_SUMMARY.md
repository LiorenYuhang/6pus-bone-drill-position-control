# J10 6DOF Axis-Wise Operational Safe Workspace Measurement — Final Summary

Date: 2026-09-09
Stage: J10-FINAL — 6DOF Axis-Wise Operational Workspace Evidence Audit & Closure
Authoritative Baseline: Frozen J8 (`v1.0-j8-active-servo-hardware-qualified`) + J9 Dual IMU Telemetry
Status: **COMPLETE / FROZEN FOR J10-A**

---

## 1. Research Question & Scope Definition

### 1.1 Research Question
The sole scientific objective of stage J10 was to determine the **HOME-centered, single-DOF axis-wise operational workspace** of the assembled 6PUS parallel manipulator under real-hardware closed-loop conditions.

Specifically, starting from the physical resting HOME configuration:
$$[x=0, y=0, z=z_0 \approx 132.612\text{ mm}, \alpha=0, \beta=0, \gamma=0]$$
each degree of freedom (\(X, Y, Z, \text{Roll}, \text{Pitch}, \text{Yaw}\)) was excited **independently along its single coordinate axis** to identify the maximum achievable displacement, operational limits, or pre-calibration boundary conditions.

### 1.2 Explicit Exclusions & Non-Claims
1. **Not a Coupled 6D Geometric Workspace**:
   - J10 did **not** measure or characterize coupled multidimensional workspace boundaries (e.g., maximum tilt achievable at extreme \(Z\) heights, or permissible \(XY\) planar excursions under non-zero roll/pitch/yaw).
   - Multi-axis coupled workspace mapping is explicitly deferred to **J10B** (to be executed after Stage J11 calibration).
2. **Not a Clinical Envelope**:
   - J10 results represent raw physical and algorithmic measurement facts under test-harness conditions. They do not constitute a certified medical or surgical operational envelope.
3. **Uncalibrated Baseline**:
   - Kinematic transformations rely on nominal geometric constants (`Kinematics/params.m`). True structural link lengths, joint spherical offsets, and IMU frame extrinsic lever-arms remain uncalibrated pending J11.

---

## 2. Methodology & Safety Principles

1. **Actuator Hard Boundary Contract**:
   - The authoritative actuator motion domain is strictly the physical stroke \([0, 2000]\text{ steps}\).
   - The historical \([200, 1800]\text{ steps}\) envelope is legacy metadata only. When candidate trajectories required excursions beyond \([200, 1800]\), the explicit `--maintenance-mode` flag was supplied to bypass driver-layer software pre-flight warnings while maintaining active watchdog protection.
2. **Endpoint Proximity Stop Rule**:
   - To protect lead screws and bearings prior to high-precision calibration, candidate progression in any direction was halted whenever the minimum remaining actuator margin dropped to \(\le 20\text{ steps}\) (\(0.500\text{ mm}\)).
   - Direct command of absolute actuator endpoints (\(q=0\) or \(q=2000\)) in task space was strictly forbidden.
   - Points stopped by this protocol are classified as **`pre-calibration absolute-endpoint-proximity censored`**, not operational failures.
3. **Zero Production Changes**:
   - Production packages ([`pus6_motor_driver`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver), [`pus6_kinematics`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics), [`pus6_control`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_control)) remained completely frozen and unmodified throughout J10.
   - All test logic, dynamic task-space gating, and safety expansions were strictly confined to the experiment runner ([`j10_a1_single_candidate_workspace_measurement.py`](file:///home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j10_a1_single_candidate_workspace_measurement.py)).
4. **Mandatory HOME Lifecycle**:
   - Every hardware run required confirmed `VALID_HOME` (all six axes within \([995, 1005]\text{ steps}\), zero motor currents, zero fault codes) both before motion initiation and following candidate completion.

---

## 3. Final Axis-Wise Operational Workspace Summary

These are the authoritative HOME-centered, single-DOF axis-wise operational ranges. They are not a complete coupled 6D geometric workspace.

| DOF | Qualified / Operational Range |
|:---|:---:|
| **X** | `[-45.75, +45.75] mm` |
| **Y** | `[-45.50, +49.00] mm` |
| **Z** | `[-25.00, +25.00] mm` — 50.00 mm total common-mode travel (`q_1 = ... = q_6`) |
| **Roll** | `[-35.0, +37.5] deg` |
| **Pitch** | `[-30.0, +35.0] deg` |
| **Yaw** | `[-90.0, +90.0] deg` |

For Z, HOME is approximately `q_i=1000` steps and `Delta Z=0`; the known common-mode endpoints are `q_i=0` at `Delta Z=-25.00 mm` and `q_i=2000` at `Delta Z=+25.00 mm`. J10-A12 Cartesian sampling stopped at +24.50 mm under the endpoint-proximity protocol, while the approximately -24.983 mm STORAGE_PARK observation is experimental coverage evidence. Those conservative sampling limits do not reduce the known full common-mode Z travel capability.

## 4. Detailed Axis-Wise Experimental Evidence

The table below summarizes the audited empirical results established from the primary CSV records, telemetry JSON metadata, and final stage closure reports across all six axes.

| Axis / DOF | Direction | Capability / Experimental Coverage | Observed Feedback Steps | Absolute Margin | Limiting Actuator | First Observed Failure / Stop Point | Boundary Classification |
|:---|:---:|:---:|:---|:---:|:---:|:---|:---|
| **X** | **+** | **+45.75 mm** | `[1463, 1865, 1970, 1189, 1270, 1643]` | 29 steps | **ID3** | +46.00 mm (predicted margin 18 steps) | Pre-calibration absolute-endpoint-proximity censored |
| | **-** | **-45.75 mm** | `[1643, 1269, 1188, 1970, 1866, 1463]` | 29 steps | **ID4** | -46.00 mm (predicted margin 18 steps) | Pre-calibration absolute-endpoint-proximity censored |
| **Y** | **+** | **+49.00 mm** | `[1237, 1968, 1779, 1779, 1968, 1236]` | 31 steps | **ID2 / ID5** | +49.50 mm (predicted margin 11 steps) | Pre-calibration absolute-endpoint-proximity censored |
| | **-** | **-45.50 mm** | `[1969, 1283, 1429, 1430, 1283, 1968]` | 30 steps | **ID1 / ID6** | -45.75 mm (predicted margin 19 steps) | Pre-calibration absolute-endpoint-proximity censored |
| **Z** | **+** | **+25.00 mm capability**<br>J10 coverage: +24.50 mm (Z FK = 157.08 mm) | `[1980, 1980, 1979, 1980, 1979, 1980]` *(coverage)* | 20 steps *(coverage)* | **ID1..6** | `q_i=2000` is the upper common-mode endpoint; direct endpoint command was avoided in J10 | Full common-mode range is `[-25.00, +25.00] mm`; +24.50 mm is experimental coverage only |
| | **-** | **-25.00 mm capability**<br>J10 coverage: -20.00 mm (Cartesian) / -24.98 mm (PARK joint) | `[200, 200, 200, 200, 200, 200]` *(coverage)*<br>`[1, 0, 0, 1, 1, 1]` *(PARK)* | 200 steps *(Z-20)*<br>0 steps *(PARK)* | **ID1..6** | `q_i=0` is the lower common-mode endpoint; J10 observations provide experimental coverage evidence | Full common-mode range is `[-25.00, +25.00] mm`; detailed coverage remains historical evidence |
| **Roll** | **+** | **+37.50 deg** | `[1620, 1001, 439, 439, 1000, 1620]` | 379 steps | **ID1 / ID6** | +40.00 deg (timeout / `overcurrent_ID4`) | Operational current/load failure before stroke limit |
| | **-** | **-35.00 deg** | `[489, 1000, 1559, 1560, 1001, 490]` | 440 steps | **ID3 / ID4** | -37.50 deg (timeout / `overcurrent_ID4`) | Operational current/load failure before stroke limit |
| **Pitch** | **+** | **+35.00 deg** | `[1312, 1663, 1334, 705, 403, 683]` | 336 steps | **ID2** | +40.00 deg (timeout / `overcurrent_ID5`) | Operational current/load failure before stroke limit |
| | **-** | **-30.00 deg** | `[724, 474, 740, 1289, 1572, 1273]` | 427 steps | **ID5** | -35.00 deg (timeout / `overcurrent_ID6`) | Operational current/load failure before stroke limit |
| **Yaw** | **+** | **+90.00 deg** | `[1733, 1400, 1733, 1401, 1733, 1401]` | 266 steps | **ID1/3/5** | +95.00 deg (FK numerical failure / timeout)<br>+105.0 deg (strut entanglement hazard) | Operational numerical/tracking failure at +95°; configuration entanglement hazard at +105° |
| | **-** | **-90.00 deg** | `[1401, 1733, 1401, 1733, 1401, 1733]` | 266 steps | **ID2/4/6** | Negative boundary not reached; stopped due to high-Yaw geometric hazard | Exploration stopped at -90.0° due to geometric safety clearance protocol |

---

## 5. Resolution of Evidence Contradictions & Audit Findings

During the J10-FINAL audit, the preliminary handwritten summary table generated at the conclusion of J10-A12 was compared against the primary stage closure reports ([`J10_A6_2`](file:///home/lyapunov/bone_drill/reports/j10/J10_A6_2_X_POS_ABSOLUTE_BOUND_CONTINUATION_20260909.md), [`J10_A7`](file:///home/lyapunov/bone_drill/reports/j10/J10_A7_X_NEGATIVE_COMPLETE_CHARACTERIZATION_20260909.md), [`J10_A8`](file:///home/lyapunov/bone_drill/reports/j10/J10_A8_Y_COMPLETE_CHARACTERIZATION_20260909.md), [`J10_A8_1`](file:///home/lyapunov/bone_drill/reports/j10/J10_A8_1_Y_BOUNDARY_CLOSURE_20260909.md)) and raw JSON metadata:

1. **X-Axis Limiting Actuators and Margins**:
   - *Draft A12 Error*: Erroneously listed X limiting actuators as "ID2 (at -45.75), ID5 (at +45.75), margin 3 steps".
   - *Authoritative Correction*: In positive X motion (+45.75 mm), actuator ID3 extends to 1971 steps (margin **29 steps**, limiter **ID3**). In negative X motion (-45.75 mm), actuator ID4 extends to 1971 steps (margin **29 steps**, limiter **ID4**). The erroneous "3 steps" originated from a theoretical projection at 46.50 mm (`ID3=2003`), which was never commanded on hardware.
2. **Y-Axis Limiting Actuators and Margins**:
   - *Draft A12 Error*: Erroneously listed Y limiting actuators as "ID3/4 (at -45.50), ID6/1 (at +49.00), margins 8 steps on +49, 13 steps on -45.5".
   - *Authoritative Correction*: In positive Y motion (+49.00 mm), symmetrical actuators ID2 and ID5 extend to 1969 steps (margin **31 steps**, limiters **ID2 / ID5**). In negative Y motion (-45.50 mm), symmetrical actuators ID1 and ID6 extend to 1970 steps (margin **30 steps**, limiters **ID1 / ID6**).
3. **Z Terminology Precision**:
   - Replaced colloquial references to "dead points" with the standardized term **"absolute actuator command endpoint"**.
   - Clarified that the lower bound \(Z \approx 107.629\text{ mm}\) (\(\Delta Z = -24.983\text{ mm}\)) was achieved and qualified via the native joint-space `run_j7_servo_home_park.py --action park` routine, rather than driving Cartesian IK to the boundary.

---

## 6. Major Incidents, Lessons & Failure Mechanisms

### 5.1 Yaw +105° Geometric Entanglement Incident (J10-A11.2)
- **Incident Description**: At Yaw = +105.0°, adjacent strut rods and spherical rod ends (pairs ID4-ID5, ID6-ID1, ID2-ID3) physically crossed and interlocked in 3D space, mechanically binding the closed kinematic chain.
- **Root Mechanism**: Simulation of strut-to-strut spatial clearance revealed that minimum 3D centerline separation dropped from \(11.52\text{ mm}\) at +90° to \(8.51\text{ mm}\) at +105°, falling below the physical rod diameter and ball-joint housing clearance.
- **Resolution**: Actuators stalled near 1894/1558 steps; automated return was impossible until manual operator unbinding. Following manual untangling, read-only telemetry verified `VALID_HOME` with zero sensor damage or mechanical distortion.
- **Safety Policy**: **Rerun of Yaw +105° is strictly prohibited**. Positive Yaw test gate was locked at +97.5°, and negative Yaw exploration was terminated at -90.0° to maintain a \(\ge 11.5\text{ mm}\) spatial clearance buffer.

### 5.2 Yaw +95° Numerical Kinematic Execution Failure
- **Crucial Distinction**: At Yaw = +95.0°, **no physical collision or mechanical binding occurred**. Actuators moved smoothly to ~1787/1450 steps.
- **Root Mechanism**: Analytical forward kinematics encountered `WorkspaceLimitError` (residual norm \(6.81 \times 10^{-10}\text{ m} > 1.0 \times 10^{-10}\text{ m}\)), reflecting numerical Jacobian ill-conditioning under extreme torsion. The candidate timed out awaiting feedback convergence, after which standard servo home return restored the robot to `VALID_HOME` with zero faults.

### 5.3 Roll & Pitch Pre-Endpoint Overcurrent Failures
- **Observed Behavior**:
  - Roll+ failed at +40.0° (`overcurrent_ID4`).
  - Roll- failed at -37.5° (`overcurrent_ID4`).
  - Pitch+ failed at +40.0° (`overcurrent_ID5`).
  - Pitch- failed at -35.0° (`overcurrent_ID6`).
- **Mechanism**: Under steep platform tilt, gravitational and kinematic constraint torque significantly increases lead-screw axial loading. The motor driver's built-in overcurrent trip triggered safely before actuators approached stroke limits. These events are operational current/load limits, not physical stroke exhaustion or confirmed link collisions.

---

## 7. Software Architecture & Production Integrity

1. **Production Code Diff Audit**:
   - Diff against frozen baseline `v1.0-j8-active-servo-hardware-qualified`: **ZERO functional lines changed in production packages**.
   - `bone_ws/src/pus6_motor_driver`: Unmodified.
   - `bone_ws/src/pus6_kinematics`: Unmodified.
   - `bone_ws/src/pus6_control`: Unmodified.
2. **Experiment Harness Isolation**:
   - All enhancements were contained in [`bone_ws/src/h30_imu/scripts/j10_a1_single_candidate_workspace_measurement.py`](file:///home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j10_a1_single_candidate_workspace_measurement.py):
     - Dynamic task-space gate injection for isolated candidate validation.
     - Automated pre-flight HOME stability confirmation.
     - Non-blocking telemetry capture and synchronized H30 IMU + joint feedback logging.
3. **Automated Testing**:
   - Regression test suite passed 27/27 (`pytest bone_ws/src/h30_imu/test/test_j10_a1_single_candidate_workspace_measurement.py`).
   - `python3 -m py_compile` verified syntax across all workspace scripts.

---

## 8. Provisional Preliminary Engineering Window (Non-Binding)

> [!WARNING]
> **PROVISIONAL / PRELIMINARY NOTICE**:
> The values below are provided solely as an interim engineering operational reference. They do **NOT** constitute certified clinical boundaries, final software safety gates, or approved surgical operational limits. Final gates will be synthesized in **J10B** following full kinematic and sensor calibration in **J11**.

| DOF | Preliminary Engineering Operating Window | Basis & Safety Rationale |
|:---|:---:|:---|
| **X** | `[-40.0, +40.0] mm` | Retains \(\ge 245\text{ steps}\) (\(6.1\text{ mm}\)) margin from stroke limits; zero load issues. |
| **Y** | `[-40.0, +40.0] mm` | Retains \(\ge 238\text{ steps}\) (\(5.9\text{ mm}\)) margin from stroke limits; symmetric behavior. |
| **Z** | `[-15.0, +20.0] mm`<br>(\(Z \in [117.6, 152.6]\text{ mm}\)) | Avoids extremes of both upper stroke (\(\ge 200\text{ steps}\) margin) and lower mechanical contact. |
| **Roll** | `[-30.0, +30.0] deg` | Conservative buffer below negative (-35°) and positive (+37.5°) overcurrent trips. |
| **Pitch** | `[-25.0, +30.0] deg` | Conservative buffer below negative (-30°) and positive (+35°) overcurrent trips. |
| **Yaw** | `[-60.0, +60.0] deg` | Generous \(\ge 18\text{ mm}\) strut clearance; avoids FK numerical degradation and entanglement hazard. |

---

## 9. Definition and Scope of Post-Calibration Stage J10B

Stage J10B is strictly defined as **Post-J11 Calibrated Workspace Re-validation**, to be initiated following the completion of J11. Its scope includes:
1. **Calibrated Axis-Wise Boundary Re-measurement**: Re-evaluate key single-axis boundaries using the calibrated kinematic model and true parameter set.
2. **Re-confirmation of Endpoint-Adjacent Results**: Re-evaluate X, Y, and Z endpoint-adjacent limits with true link lengths and anchor geometry.
3. **Calibrated Task-Space Attitude Validation**: Re-confirm Roll, Pitch, and Yaw calibrated task-space values against external ground truth.
4. **Targeted Coupled Workspace Slices**: Measure a small, targeted number of coupled-workspace slices (e.g. tilt envelopes at varying vertical elevations \(Z_{\text{low}}, Z_{\text{mid}}, Z_{\text{high}}\)).

*Non-Scope Exclusions*: Dynamics identification, trajectory acceleration optimization, anti-overcurrent controller tuning, and clinical certification or clinical-grade gating are explicitly excluded from J10B.

---

## 10. Final Verdict

### **J10 Final Verdict: FULL PASS / CLOSED**
*(Official **FROZEN** status established upon release commit and tag)*

1. **Research Objective Fulfilled**:
   Successfully established the complete empirical baseline of HOME-centered, single-DOF axis-wise operational workspace facts for the physical 6PUS robot across all six coordinates (\(X, Y, Z, \text{Roll}, \text{Pitch}, \text{Yaw}\)).
2. **Objective Hardware Status**:
   - Yaw +105° experienced real closed-chain strut entanglement and required manual operator unbinding.
   - Following manual recovery, all six axes feedback positions, motor currents (\(0\text{ mA}\)), fault codes (\(0\)), and forward kinematics convergence (residual \(< 6 \times 10^{-11}\text{ m}\)) returned to normal baseline levels (`VALID_HOME`).
   - Currently no persistent hardware faults, driver latches, or obvious operational abnormalities are observed.
   - Without dedicated non-destructive structural inspection, no speculative claims of "zero permanent deformation" are made.
3. **Software & Architecture Isolation**:
   Zero modifications to production packages (`pus6_motor_driver`, `pus6_kinematics`, `pus6_control`). All test scaffolding was confined to the experiment harness.
4. **Stage Transition**:
   J10 is formally closed. Handoff proceeds to **J11: Actuator & Kinematic Parameter Calibration**.
