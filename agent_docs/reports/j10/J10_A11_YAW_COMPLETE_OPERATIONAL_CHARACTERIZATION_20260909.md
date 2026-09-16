# J10-A11 Yaw +/- Operational Characterization & A11.4 Closure Report Correction

Date: 2026-09-09
Stage: J10-A11.4 — Yaw Closure Report Correction

The J10-only exploratory Yaw task-space gate was initially set to +/-110 deg,
where offline projection placed the first absolute actuator-invalid point at approximately +/-120 deg.
Following the +105 deg physical strut crossing/entanglement incident (A11.2) and subsequent analysis,
the temporary **J10 experimental test gate** was tightened to **+97.5 deg** (direction-specific safety gate for
the J10 measurement runner, based on the observed geometric/configuration hazard at +105 deg and FK numerical degradation
near +98~99 deg). The negative Yaw test gate is retained at **-100 deg** without assuming bilateral mechanical symmetry.

> [!NOTE]
> **Gate Distinction**: The +97.5 deg / -100.0 deg limits represent the **current J10 experiment runner gate**,
> designed strictly to protect hardware during single-candidate exploration. They are distinct from any future final
> operational software gates that may be deployed in application layers. Production packages (J7/J8/J9 motor driver,
> kinematics, control pipeline, and safety defaults) remain completely unmodified.

## Offline projection

| Yaw | Predicted steps (ID1..6) | Min absolute margin | Limiter | IK |
|---:|---|---|---:|---|---|
| +30 deg | [1141,1000,1141,1000,1141,1000] | 859 | ID1/3/5 | valid |
| +60 deg | [1401,1141,1401,1141,1401,1141] | 599 | ID1/3/5 | valid |
| +90 deg | [1734,1401,1734,1401,1734,1401] | 266 | ID1/3/5 | valid |
| +105 deg | [1905,1562,1905,1562,1905,1562] | 95 | ID1/3/5 | valid; historical exceeded |
| +110 deg | [1959,1619,1959,1619,1959,1619] | 41 | ID1/3/5 | valid; historical exceeded |
| +120 deg | [2062,1734,2062,1734,2062,1734] | -62 | ID1/3/5 | valid, absolute LIMIT |
| -30 deg | [1000,1141,1000,1141,1000,1141] | 859 | ID2/4/6 | valid |
| -60 deg | [1000,1401,1000,1401,1000,1401] | 599 | ID2/4/6 | valid |
| -90 deg | [1401,1734,1401,1734,1401,1734] | 266 | ID2/4/6 | valid |
| -105 deg | [1562,1905,1562,1905,1562,1905] | 95 | ID2/4/6 | valid; historical exceeded |
| -120 deg | [1734,2062,1734,2062,1734,2062] | -62 | ID2/4/6 | valid, absolute LIMIT |

## Hardware results

| Candidate | Result | Candidate / HOME | Measurement / FK Yaw | Margin / traffic |
|---:|---|---|---|---|
| +30 deg | PASS | 2.411 s / 2.287 s confirmed | valid; +29.920 deg | 859; 209 cycles / 1254 frames |
| +60 deg | PASS | 5.949 s / 4.764 s confirmed | valid; +60.374 deg | 599; 299 / 1794 |
| +90 deg | PASS | 11.431 s / 10.773 s confirmed | valid; +90.117 deg | 266; 407 / 2442 |
| +95 deg (A11.3) | FAIL/STOP | candidate timeout 14.69 s; HOME unconfirmed | FK numerical/kinematic execution failure + tracking timeout (residual 6.81e-10m) | 209; 195 / 1170 |
| +97.5 deg | NOT ATTEMPTED | blocked by immediate abort on +95 failure | strictly prohibited by safety policy | 180; 0 / 0 |
| +105 deg (maintenance) | FAIL (INCIDENT) | timeout 16.583 s; HOME unconfirmed; manual recovery | observed geometric/configuration entanglement hazard | 95; 348 / 2088 |
| -30 deg (A11.3) | PASS | 3.210 s / 2.390 s confirmed | valid; FK -29.735 deg, H30 -28.961 deg | 859; 187 / 1122 |
| -60 deg (A11.3) | PASS | 6.601 s / 7.532 s confirmed | valid; FK -59.975 deg, H30 -55.132 deg | 599; 276 / 1656 |
| -90 deg (A11.3) | PASS | 12.602 s / 12.511 s confirmed | valid; FK -89.617 deg, H30 -73.307 deg | 266; 412 / 2472 |

## Yaw Failure Mechanism Stratification (+95° vs +105°)

A rigorous physical and algorithmic distinction is maintained between the +95° and +105° failure modes:

1. **+95°: FK Numerical / Kinematic Execution Failure + Tracking Timeout**
   - **Empirical observation**: Actuators smoothly reached approximately 1787/1450 steps.
   - **Root mechanism**: The analytical FK solver encountered `WorkspaceLimitError` (residual norm $6.8115 \times 10^{-10}\text{ m} > 1.0 \times 10^{-10}\text{ m}$ tolerance), reflecting ill-conditioned kinematic Jacobian and numerical degradation under high torsion.
   - **Outcome**: Feedback position failed to settle within candidate acceptance window prior to the 14.68 s timeout (`candidate_feedback_target_timeout`).
   - **Crucial distinction**: **+95° did NOT exhibit mechanical collision or physical entanglement**. Following timeout, standard servo home return (`run_j7_servo_home_park.py --action home`) immediately restored the robot to `VALID_HOME` with zero faults.

2. **+105°: Observed Geometric / Configuration Entanglement Hazard**
   - **Empirical observation**: Strut rods and spherical rod ends between adjacent limbs (ID4-ID5, ID6-ID1, ID2-ID3) physically crossed and interlocked in space, mechanically binding the closed kinematic chain.
   - **Simulation consistency**: Pairwise 3D strut centerline clearance dropped from 11.52 mm (+90°) to 8.51 mm (+105°), which is consistent with the observed physical interference between strut rods and spherical joint housings.
   - **Outcome**: Actuators physically stalled near 1894/1558 steps, triggering timeout at 16.58 s. Automated return was impossible until manual operator unbinding.
   - **Classification**: **Observed Geometric / Configuration Entanglement Hazard**. Yaw +105 deg is strictly prohibited from rerun.

3. **Status of Prior +97.5° and -30° Runs**:
   Because HOME was not confirmed following the +105° incident, both subsequent attempts stopped immediately at pre-candidate HOME timeout with **0 motion frames executed**. They are classified as **INVALID** (not failures, and do not represent workspace limitations).

## Real Hardware Read-Only Verification (A11.2)

Following manual untangling and visual HOME inspection, strictly read-only 0x30 telemetry verified:
- **Six-axis positions**: ID1: 999, ID2: 998, ID3: 999, ID4: 998, ID5: 998, ID6: 998 steps (all within `[995, 1005]` HOME band).
- **Currents & Faults**: 0 mA across all axes, 0 active faults, 0 driver latch.
- **Authoritative startup classification**: `VALID_HOME` (`is_home=True`, `is_valid=True`).
- **Platform & Base H30 IMUs**: Active and streaming with 0 packet loss; relative orientation consistent with mechanical mounting bias.
- **Auxiliary Analytical FK**: Converged (`True`), 4 iterations, residual norm `5.88e-11 m` (< 1e-10 m tolerance); FK Pose: X +0.00 mm, Y -0.07 mm, Z +132.57 mm (nominal 132.61 mm).
- **Processes**: Strictly zero motion writers or lock holders running.

## J10 Runner Safety Gate Modifications (A11.3)

1. **Direction-specific J10 experiment runner gate**:
   - `J10_TASK_SPACE_YAW_POS_LIMIT_RAD` tightened to **+97.5 deg** (`97.5 * math.pi / 180.0`).
   - `J10_TASK_SPACE_YAW_NEG_LIMIT_RAD` retained at **-100.0 deg** (`100.0 * math.pi / 180.0`).
2. The J10 runner explicitly checks `task_space_gate_passed`. Any candidate exceeding the gate is immediately stopped at preflight with `status = "LIMIT"`, emitting 0 motion frames and 0 UART writes.
3. Automated regression suite passes 27/27 (`test_j10_a1_single_candidate_workspace_measurement.py`).
4. J7/J8/J9 production packages and default attitude gates remain completely untouched.

## Safe-Side Execution Results (A11.3)

1. **Yaw+ Requalification**:
   - **+95.0 deg**: Motion executed (195 UART cycles, 1170 feedback frames). Analytical FK solver encountered `WorkspaceLimitError` (residual norm `6.8115e-10 m`). Target feedback tolerance was not achieved within window, triggering `candidate_feedback_target_timeout` at 14.69 s (`status = LIMIT`). Restored to HOME via `run_j7_servo_home_park.py --action home` (final error 1 step, 0 faults, `VALID_HOME`).
   - **+97.5 deg**: **NOT ATTEMPTED**. Per strict pre-established safety protocol, failure/timeout at +95 deg immediately halted positive Yaw expansion. +97.5 deg was not commanded.
   - **Yaw+ First Current Operational Failure**: **+95.0 deg** (FK numerical/kinematic execution failure + tracking timeout).
   - **Yaw+ Maximum Fully-Qualified Observed**: **+90.0 deg**.

2. **Yaw- Characterization Protocol**:
   - **-30.0 deg**: Target reach 3.210 s, HOME confirmed in 2.390 s (err 1 step). FK Yaw `-29.735 deg`, Platform H30 relative Yaw `-28.961 deg` (discrepancy 0.774 deg). Parasitic translation: X -0.002 mm, Y -0.063 mm. **PASS**.
   - **-60.0 deg**: Target reach 6.601 s, HOME confirmed in 7.532 s (err 1 step). FK Yaw `-59.975 deg`, Platform H30 relative Yaw `-55.132 deg` (discrepancy 4.843 deg). Parasitic translation: X -0.093 mm, Y -0.082 mm. **PASS**.
   - **-90.0 deg**: Target reach 12.602 s, HOME confirmed in 12.511 s (err 1 step). FK Yaw `-89.617 deg`, Platform H30 relative Yaw `-73.307 deg` (discrepancy 16.311 deg). Parasitic translation: X -0.096 mm, Y -0.076 mm. **PASS**.
   - **Post-run Verification**: Read-only telemetry confirmed `VALID_HOME` (`[1000, 1000, 999, 1000, 1000, 1001]` steps, 0 mA, 0 faults, FK residual `5.89e-11 m`).
   - **Yaw- Maximum Fully-Qualified Observed**: **-90.0 deg**.

3. **Offline Negative Yaw Analysis (-92.5° ~ -100°)**:
   - `-90.0 deg`: Strut centerline min clearance 11.52 mm; FK converged (`4.18e-11 m`); PASS.
   - `-92.5 deg`: Clearance 11.02 mm; predicted steps `[1427, 1763, ...]`; margin 237 steps.
   - `-95.0 deg`: Clearance 10.51 mm; predicted steps `[1453, 1791, ...]`; margin 209 steps.
   - `-97.5 deg`: Clearance 10.00 mm; predicted steps `[1480, 1820, ...]`; margin 180 steps.
   - `-99.0 deg`: FK solver residual diverges (`1.33e-9 m` > `1.0e-10 m`, `converged=False`).
   - **Recommendation**: **STOP AT -90.0 deg**. Real hardware +95 deg already exhibited real-time FK divergence and tracking timeout. Negative Yaw presents identical kinematic symmetry and progressive strut clearance narrowing (~10 mm). Attempting -95 deg or -97.5 deg carries disproportionate risk of solver stalling or approaching the spatial interference zone, without practical engineering necessity.

## Boundary Conclusion & Engineering Workspace Summary

### Yaw Experimental Conclusions
- **Maximum Fully-Qualified Observed**: **`[-90.0°, +90.0°]`**.
- **Yaw+ First Current Operational Failure**: **`+95.0 deg`** (FK numerical/kinematic execution failure + tracking timeout; not a mechanical collision).
- **Observed Geometric / Configuration Hazard**: **`+105.0 deg`** (physical spatial strut crossing and closed-chain mechanical binding).
- **Engineering Characterization Reference**:
  - Preliminary empirical operating window: `[-60.0°, +60.0°]` (conservative operating margin, clearance > 18 mm, minimal IMU drift).
  - Extended qualified boundary: `[-90.0°, +90.0°]` (requires verified FK convergence and absence of spatial interference).
  - Software gating recommendation: In future operational software, a hard gate at or below $\pm 90.0^\circ$ is recommended to protect against high-torsion numerical conditioning loss and geometric interference.

### Five-DOF Characterized Workspace Summary (J10)
> [!IMPORTANT]
> **Completion Status**: J10 has completed single-axis operational characterization for **5 degrees of freedom** (X, Y, Roll, Pitch, Yaw).
> **Z-axis characterization is pending J10-A12**. Complete 6DOF status is NOT yet declared.

| Axis / DOF | Characterized Fully-Qualified Envelope | Boundary / Stop Classification |
|:---|:---:|:---|
| **X** | **`[-45.75, +45.75] mm`** | Pre-calibration absolute-endpoint-proximity censored |
| **Y** | **`[-45.50, +49.00] mm`** | Pre-calibration absolute-endpoint-proximity censored |
| **Z** | **Pending J10-A12** | HOME reference $z_0 \approx 132.61\text{ mm}$ |
| **Roll** | **`[-35.0, +37.5] deg`** | Operational bracket: negative `[-35, -37.5]`, positive `[+37.5, +40]` |
| **Pitch** | **`[-30.0, +35.0] deg`** | Operational bracket: negative `[-30, -35]`, positive `[+35, +40]` |
| **Yaw** | **`[-90.0, +90.0] deg`** | First failure at `+95.0 deg` (FK/timeout); observed hazard at `+105.0 deg` |

No controller tuning, sensor fusion, or J7/J8/J9 production behavior change was made. Regression test suite passed (27/27). No commit was created.
