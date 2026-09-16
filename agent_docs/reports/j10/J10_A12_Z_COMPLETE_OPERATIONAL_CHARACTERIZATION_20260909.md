# J10-A12 Z+/- Complete Operational Characterization

Date: 2026-09-09
Stage: J10-A12 — Z+/- Operational Range Characterization & Complete 6DOF Workspace Facts

## Executive Summary

J10-A12 completes the single-degree-of-freedom operational characterization of the 6PUS parallel robot by evaluating the vertical translational axis (**Z**).

Unlike planar translations (X/Y) or rotational degrees of freedom (Roll/Pitch/Yaw), pure vertical motion along the Z-axis is a **symmetric common-mode motion**: all six linear actuators displace identically ($\frac{dq_i}{dz} = 1.0$, corresponding to $40.0\text{ steps/mm}$ or $40{,}000\text{ steps/m}$). The motion is kinematically decoupled from planar translations and rotations, exhibiting minimal parasitic coupling across the entire stroke.

Actuator bounds remain strictly defined by the physical hardware hard limit $[0, 2000]$ steps. The historical $[200, 1800]$ range serves only as legacy metadata.

- **Nominal Model Z Range**: $Z \in [107.612, 157.612]\text{ mm}$ ($\Delta Z \in [-25.000, +25.000]\text{ mm}$) relative to nominal HOME $z_0 = 132.612\text{ mm}$.
- **Cartesian Characterization**:
  - **Z-**: Verified to **$\Delta Z = -20.0\text{ mm}$** ($Z_{\text{FK}} = 112.612\text{ mm}$) in Cartesian task space, and to **$\Delta Z = -24.983\text{ mm}$** ($Z_{\text{FK}} = 107.629\text{ mm}$) via joint-space `STORAGE_PARK = [0]*6` audit.
  - **Z+**: Characterized progressively from $+10.0\text{ mm}$ up to **$\Delta Z = +24.5\text{ mm}$** ($Z_{\text{FK}} = 157.076\text{ mm}$), where actuator position reaches $1980\text{ steps}$, leaving an exact margin of **$20\text{ steps}$** ($0.500\text{ mm}$).
- **Boundary Classification**: **`pre-calibration absolute-endpoint-proximity censored`**. Testing halted intentionally at $+24.5\text{ mm}$ per pre-flight safety protocol before commanding the theoretical dead stop at $2000\text{ steps}$. Zero motor overcurrents, zero driver faults, and zero tracking anomalies were observed.

---

## 1. Kinematic Common-Mode Analysis & Offline Z Mapping

For a 6PUS parallel manipulator with vertical prismatic actuators aligned symmetrically about the central axis, pure Z translation with $x = y = \text{roll} = \text{pitch} = \text{yaw} = 0$ results in equal slider displacement on all six limbs:
$$\Delta q_1 = \Delta q_2 = \Delta q_3 = \Delta q_4 = \Delta q_5 = \Delta q_6 = \Delta Z$$
With a screw pitch conversion of $40{,}000\text{ steps/m}$ ($40\text{ steps/mm}$), each $1.0\text{ mm}$ of Z displacement commands exactly $40\text{ steps}$ on all six motors.

Numerical evaluation with `Analytical6PusKinematics` confirms that cross-limb variance across all six actuators is $< 2.0 \times 10^{-13}\text{ m}$.

### Offline Pure Z Projection Table

| $\Delta Z$ (mm) | Absolute $Z$ (mm) | Predicted Steps (ID1..6) | Min Margin (steps) | Limiting Actuator | Kinematic Status |
|---:|---:|---|---:|:---:|:---|
| **-25.0 mm** | 107.612 mm | `[0, 0, 0, 0, 0, 0]` | 0 | ID1..6 | Lower Stroke Bound ($q=0$) |
| **-24.5 mm** | 108.112 mm | `[20, 20, 20, 20, 20, 20]` | 20 | ID1..6 | Valid |
| **-20.0 mm** | 112.612 mm | `[200, 200, 200, 200, 200, 200]` | 200 | ID1..6 | Valid |
| **-15.0 mm** | 117.612 mm | `[400, 400, 400, 400, 400, 400]` | 400 | ID1..6 | Valid |
| **-10.0 mm** | 122.612 mm | `[600, 600, 600, 600, 600, 600]` | 600 | ID1..6 | Valid |
| **-5.0 mm** | 127.612 mm | `[800, 800, 800, 800, 800, 800]` | 800 | ID1..6 | Valid |
| **0.0 mm** | **132.612 mm** | **`[1000, 1000, 1000, 1000, 1000, 1000]`** | **1000** | **ID1..6** | **Nominal HOME ($z_0$)** |
| **+5.0 mm** | 137.612 mm | `[1200, 1200, 1200, 1200, 1200, 1200]` | 800 | ID1..6 | Valid |
| **+10.0 mm** | 142.612 mm | `[1400, 1400, 1400, 1400, 1400, 1400]` | 600 | ID1..6 | Valid |
| **+15.0 mm** | 147.612 mm | `[1600, 1600, 1600, 1600, 1600, 1600]` | 400 | ID1..6 | Valid |
| **+20.0 mm** | 152.612 mm | `[1800, 1800, 1800, 1800, 1800, 1800]` | 200 | ID1..6 | Valid |
| **+22.5 mm** | 155.112 mm | `[1900, 1900, 1900, 1900, 1900, 1900]` | 100 | ID1..6 | Valid (Maintenance Mode) |
| **+24.0 mm** | 156.612 mm | `[1960, 1960, 1960, 1960, 1960, 1960]` | 40 | ID1..6 | Valid (Maintenance Mode) |
| **+24.5 mm** | 157.112 mm | `[1980, 1980, 1980, 1980, 1980, 1980]` | 20 | ID1..6 | Valid (Maintenance Mode) |
| **+25.0 mm** | 157.612 mm | `[2000, 2000, 2000, 2000, 2000, 2000]` | 0 | ID1..6 | Upper Stroke Bound ($q=2000$) |

---

## 2. Z- / PARK Side Characterization & Qualification

### 2.1 Context of STORAGE_PARK
In J7, J8, and early J9 stages, `STORAGE_PARK = [0]*6` steps was formally qualified and utilized as the safe resting state for the manipulator. In this configuration, all six lead-screw sliders retract to their absolute base mechanical zero (bottom stroke limit).

### 2.2 Task-Space Real Hardware Confirmation
To verify smooth downward travel and stability without driving the Cartesian inverse kinematics into singular edge conditions, two task-space candidates were executed:

1. **Z -10.0 mm**:
   - Commanded pose: $Z = 122.612\text{ mm}$ ($\Delta Z = -10.0\text{ mm}$).
   - Target steps: `[600, 600, 600, 600, 600, 600]`. Min absolute margin: $600\text{ steps}$.
   - Reach duration: $7.06\text{ s}$; HOME return confirmed in $6.81\text{ s}$ (final error: $2\text{ steps}$).
   - UART traffic: $272\text{ cycles}$ ($1632\text{ feedback frames}$).
   - FK window median: $X = +0.002\text{ mm}$, $Y = +0.117\text{ mm}$, $Z = 122.608\text{ mm}$ ($\Delta Z = -10.004\text{ mm}$), $\text{Roll} = +0.058^\circ$, $\text{Pitch} = +0.015^\circ$, $\text{Yaw} = -0.050^\circ$.
   - **Result**: **PASS**.

2. **Z -20.0 mm**:
   - Commanded pose: $Z = 112.612\text{ mm}$ ($\Delta Z = -20.0\text{ mm}$).
   - Target steps: `[200, 200, 200, 200, 200, 200]`. Min absolute margin: $200\text{ steps}$.
   - Reach duration: $14.06\text{ s}$; HOME return confirmed in $13.53\text{ s}$ (final error: $2\text{ steps}$).
   - UART traffic: $443\text{ cycles}$ ($2658\text{ feedback frames}$).
   - FK window median: $X = 0.000\text{ mm}$, $Y = 0.000\text{ mm}$, $Z = 112.612\text{ mm}$ ($\Delta Z = -20.000\text{ mm}$), $\text{Roll} = -0.000^\circ$, $\text{Pitch} = +0.031^\circ$, $\text{Yaw} = -0.124^\circ$.
   - **Result**: **PASS**.

### 2.3 Official Joint-Space PARK Endpoint Audit
To verify the physical lower bound without forcing Cartesian trajectory extrapolation near $q=0$, the official production joint-space routine was executed:
```bash
python3 bone_ws/src/pus6_motor_driver/scripts/run_j7_servo_home_park.py --action park
```
- **Observed Feedback at PARK**: `[1, 0, 0, 1, 1, 1]` steps (within `[0, 5]` PARK band).
- **Classification**: `VALID_PARK` (`is_park=True`, `is_valid=True`).
- **Telemetry**: Motor currents: $0\text{ mA}$ on all 6 axes; fault codes: $0$.
- **Analytical Forward Kinematics at PARK**:
  - $Z_{\text{FK}} = \mathbf{107.629\text{ mm}}$ ($\Delta Z = \mathbf{-24.983\text{ mm}}$ from HOME).
  - Parasitic coupling: $X = 0.000\text{ mm}$, $Y = 0.000\text{ mm}$, $\text{Roll} = 0.000^\circ$, $\text{Pitch} = 0.000^\circ$, $\text{Yaw} = 0.000^\circ$.
  - FK solver convergence: `converged=True`, 4 iterations, residual norm $2.86 \times 10^{-11}\text{ m}$.
- **Restoration to HOME**: Executed `--action home`. Reached `[998, 999, 998, 998, 998, 998]` steps (max error $2\text{ steps}$), confirmed `VALID_HOME`.

---

## 3. Z+ Progressive Hardware Characterization

In the positive Z direction, actuators move upward towards the upper stroke limit ($2000\text{ steps}$). A coarse-to-fine sequence was executed up to the $20\text{-step}$ safety proximity boundary.

### 3.1 Hardware Execution Results

| Candidate | Result | Target Steps | Absolute Margin | Reach / HOME Duration | UART Traffic | FK Window Median Pose |
|:---|:---:|:---:|:---:|:---:|:---:|:---|
| **Z+10.0 mm** | **PASS** | `[1400]*6` | 600 steps | 6.62 s / 6.55 s | 287 c / 1722 f | $X=+0.173\text{ mm}, Y=+0.076\text{ mm}, Z=142.587\text{ mm}$<br>$\Delta Z=+9.975\text{ mm}, R=+0.041^\circ, P=-0.075^\circ, Y=0.000^\circ$ |
| **Z+15.0 mm** | **PASS** | `[1600]*6` | 400 steps | 10.71 s / 10.34 s | 355 c / 2130 f | $X=-0.040\text{ mm}, Y=-0.035\text{ mm}, Z=147.592\text{ mm}$<br>$\Delta Z=+14.980\text{ mm}, R=-0.022^\circ, P=-0.008^\circ, Y=-0.069^\circ$ |
| **Z+20.0 mm** | **PASS** | `[1800]*6` | 200 steps | 13.40 s / 13.37 s | 435 c / 2610 f | $X=+0.024\text{ mm}, Y=-0.024\text{ mm}, Z=152.588\text{ mm}$<br>$\Delta Z=+19.976\text{ mm}, R=-0.014^\circ, P=+0.004^\circ, Y=0.000^\circ$ |
| **Z+22.5 mm** | **PASS** | `[1900]*6` | 100 steps | 15.98 s / 14.50 s | 482 c / 2892 f | $X=-0.058\text{ mm}, Y=-0.011\text{ mm}, Z=155.091\text{ mm}$<br>$\Delta Z=+22.479\text{ mm}, R=-0.012^\circ, P=+0.019^\circ, Y=+0.268^\circ$ |
| **Z+24.0 mm** | **PASS** | `[1960]*6` | 40 steps | 14.76 s / 15.38 s | 509 c / 3054 f | $X=-0.045\text{ mm}, Y=0.000\text{ mm}, Z=156.579\text{ mm}$<br>$\Delta Z=+23.967\text{ mm}, R=+0.023^\circ, P=+0.006^\circ, Y=-0.054^\circ$ |
| **Z+24.5 mm** | **PASS** | `[1980]*6` | 20 steps | 15.41 s / 15.36 s | 513 c / 3078 f | $X=-0.115\text{ mm}, Y=0.000\text{ mm}, Z=157.076\text{ mm}$<br>$\Delta Z=+24.464\text{ mm}, R=+0.019^\circ, P=+0.032^\circ, Y=+0.048^\circ$ |

> [!NOTE]
> Candidates $\ge +22.5\text{ mm}$ command steps $\ge 1900$, which exceed the historical $[200, 1800]$ envelope. These runs operated under `--maintenance-mode` with `j10_extended_task_space_gate` set to $25.0\text{ mm}$.

### 3.2 Proximity Stopping Condition
Advancing to $\Delta Z = +25.0\text{ mm}$ would command exactly $2000\text{ steps}$ on all six actuators (margin = $0\text{ steps}$). In accordance with the pre-established safety protocol:
- Testing halts when remaining actuator margin drops to $\le 20\text{ steps}$.
- Direct command of $q=2000\text{ steps}$ is strictly avoided to prevent hard mechanical stops or pre-calibration bottoming/topping out.
- Therefore, testing terminated with **$\Delta Z = +24.5\text{ mm}$** as the final qualified data point.

---

## 4. Boundary Results & Classification

### 4.1 Z Summary Metrics
- **HOME Reference**: $z_0 \approx 132.612\text{ mm}$ (measured feedback: $132.608\text{ mm}$).
- **Nominal Model Physical Stroke**: $[107.612, 157.612]\text{ mm}$ ($\Delta Z \in [-25.000, +25.000]\text{ mm}$).
- **Maximum Fully-Qualified Observed Cartesian Range**:
  - $\mathbf{\Delta Z \in [-20.0, +24.5]\text{ mm}}$
  - Corresponding to absolute $Z_{\text{FK}} \in [112.612, 157.076]\text{ mm}$.
- **Joint-Space Mechanical Endpoint Evidence (STORAGE_PARK)**:
  - Actuator steps: `[1, 0, 0, 1, 1, 1]` $\approx [0]*6$.
  - Absolute FK height: $\mathbf{Z_{\text{FK}} = 107.629\text{ mm}}$ ($\Delta Z = -24.983\text{ mm}$).
- **Boundary Classification**:
  - **Z+**: **`pre-calibration absolute-endpoint-proximity censored`**. Halted at $+24.5\text{ mm}$ with $20\text{ steps}$ ($0.500\text{ mm}$) remaining margin. No physical binding, overcurrent, or kinematic degradation.
  - **Z-**: **`pre-calibration absolute-endpoint-proximity censored`** in Cartesian mode; physically qualified down to bottom stroke limit via joint-space PARK.

---

## 5. Complete 6DOF Observed Workspace Facts Table

With the conclusion of J10-A12, empirical characterization across all six independent degrees of freedom is complete.

The table below presents **strictly observed measurement facts and hardware boundaries**. It does not make clinical envelope claims or final production gating recommendations.

| Axis / DOF | Nominal HOME Reference | Fully-Qualified Observed Range | Limiting / Censoring Actuator | Boundary / Stop Classification |
|:---|:---:|:---:|:---:|:---|
| **X** | $x_0 = 0.0\text{ mm}$ | **`[-45.75, +45.75] mm`** | ID2 (at -45.75)<br>ID5 (at +45.75) | Pre-calibration absolute-endpoint-proximity censored (margin 3 steps) |
| **Y** | $y_0 = 0.0\text{ mm}$ | **`[-45.50, +49.00] mm`** | ID3/4 (at -45.50)<br>ID6/1 (at +49.00) | Pre-calibration absolute-endpoint-proximity censored (margin 8 steps on +49, 13 steps on -45.5) |
| **Z** | $z_0 = 132.61\text{ mm}$ | **`[-20.0, +24.5] mm`**<br>*(Cartesian task-space)*<br>**`[-24.98, +24.5] mm`**<br>*(incl. PARK joint-space)* | ID1..6 (symmetric common-mode) | Pre-calibration absolute-endpoint-proximity censored (upper margin 20 steps / 0.500 mm; lower stroke reached at STORAGE_PARK) |
| **Roll** | $\alpha_0 = 0.0^\circ$ | **`[-35.0, +37.5] deg`** | ID2 (negative)<br>ID1 (positive) | Operational current/load limit (operational bracket: `[-35.0, -37.5] deg` and `[+37.5, +40.0] deg`, overcurrent trip) |
| **Pitch** | $\beta_0 = 0.0^\circ$ | **`[-30.0, +35.0] deg`** | ID6 (negative)<br>ID5 (positive) | Operational current/load limit (operational bracket: `[-30.0, -35.0] deg` and `[+35.0, +40.0] deg`, overcurrent trip) |
| **Yaw** | $\gamma_0 = 0.0^\circ$ | **`[-90.0, +90.0] deg`** | ID1/3/5 (positive)<br>ID2/4/6 (negative) | First failure at `+95.0 deg` (FK numerical execution failure + tracking timeout; no collision).<br>Observed geometric/configuration entanglement hazard at `+105.0 deg` (physical strut crossing). |

---

## 6. Verification and Integrity Sign-Off

1. **Hardware State**: Verified at **`VALID_HOME`** with strictly read-only $0\text{x}30$ telemetry:
   - Actuator positions: `[1000, 1000, 999, 1000, 1000, 1000]` steps (max deviation $1\text{ step}$).
   - Motor currents: $0\text{ mA}$ across all six axes; fault codes: $0$.
   - Analytical FK converged (residual norm $2.91 \times 10^{-11}\text{ m} < 10^{-10}\text{ m}$).
2. **Production Integrity**:
   - Zero changes made to production packages (`pus6_motor_driver`, `pus6_kinematics`, `pus6_control`).
   - Zero controller, filter, or calibration parameters modified.
   - Zero git commits created.
3. **Artifacts Stored**:
   - `results/J10_A12/J10_workspace_z_neg100mm_20260909_214857_{csv,metadata.json}`
   - `results/J10_A12/J10_workspace_z_neg200mm_20260909_214929_{csv,metadata.json}`
   - `results/J10_A12/J10_workspace_z_pos100mm_20260909_215130_{csv,metadata.json}`
   - `results/J10_A12/J10_workspace_z_pos150mm_20260909_215200_{csv,metadata.json}`
   - `results/J10_A12/J10_workspace_z_pos200mm_20260909_215238_{csv,metadata.json}`
   - `results/J10_A12/J10_workspace_z_pos225mm_20260909_215624_{csv,metadata.json}`
   - `results/J10_A12/J10_workspace_z_pos240mm_20260909_215712_{csv,metadata.json}`
   - `results/J10_A12/J10_workspace_z_pos245mm_20260909_215903_{csv,metadata.json}`
