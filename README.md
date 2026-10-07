# 6PUS-Anti-Vibration

> [English] | [中文](README_CN.md)  
> **A ROS 2 based real-time sensing and control framework for a 6PUS parallel surgical robot with active vibration suppression.**

---

## 1. Project Overview

**6PUS-Anti-Vibration** is an open, high-precision, real-time sensing and motion control framework designed for a 6-PUS (Prismatic-Universal-Spherical) parallel manipulator / Stewart platform. Developed for precision orthopaedic surgical robotics, the system provides high-bandwidth, deterministic Cartesian motion alongside sub-millisecond multi-sensor observation for active disturbance rejection and vibration suppression.

### Key Milestones:
- **J1–J10 (Completed & Frozen)**: Comprehensive real-hardware foundation covering low-level multi-actuator UART communication, analytical inverse kinematics, $C^2$ jerk-limited trajectory shaping, 50 Hz continuous servo streaming, dual-IMU microsecond clock synchronization, numerical forward kinematics, and complete 6-DOF single-axis operational workspace qualification on the physical platform.
- **J11-M1 (Completed & Frozen)**: Base-IMU full-$SO(3)$ feedforward attitude hold, dedicated driver runtime qualification, two-tier safety policy, 300s static home qualification, and low-speed dynamic base disturbance rejection on real physical hardware (`v1.0-j11-m1-feedforward-hardware-qualified`).
- **J11 (Completed & Frozen)**: Retained Base-IMU SO(3) Feedforward + Platform-IMU Classical P/PD Attitude Feedback. Roll / Pitch / Yaw gains: `Kp = [0.05, 0.05, 0.025]`, `Kd = [0, 0.0015, 0.0015] s`, `Ki = 0`; low-pass filter cutoff: `fc = 5 Hz` (Roll=P, Pitch=PD, Yaw=PD). Demo A and Demo B verified on real hardware (`v1.0-j11-attitude-stabilization-frozen`).
- **J12 (Completed & Frozen)**: Classical Feedforward + P/PD Dynamic Performance Characterization. Comprehensive large-angle static load qualification (J12-1), 3DOF combined screening (J12-2), and handheld dynamic performance qualification up to observed effective peak angular rate of `92.33°/s` (J12-3). Max measured motor current `468 mA`. Host safety threshold unified with actuator register at `MAX_CURRENT_MA = 1500` (`v1.0-j12-classical-dynamic-performance-frozen`).
- **J13 (Completed & Engineering Qualified)**: First-Order Linear ADRC / LADRC Platform Attitude Residual Control on Real 6PUS. Full closed-loop real-hardware architecture ($SO(3)\text{ Feedforward} + \text{3-axis LADRC}$ with exact ZOH discrete second-order LESO and $\pm 0.5^\circ$ residual clamp). Empty-load 10s qualification completed (0 faults). Verified under eccentric payload condition against Classical P/PD baseline. Identified runtime communication long-tail latency as core bottleneck for prolonged continuous operation. Status: CLOSED / ENGINEERING QUALIFIED.

---

## 2. Current Development Status

The development of the 6PUS robotic platform is organized into two distinct overarching phases:

```text
┌─────────────────────────────────────────────────────────────────────────┐
│              Part I — Real-Hardware Motion Foundation (J1–J10)          │
│                      [STATUS: COMPLETED & FROZEN]                       │
│  • J1: RS485 UART Protocol Baseline (921600 8N1, 6 actuators online)    │
│  • J2: Analytical Inverse Kinematics (closed-form, microsecond compute) │
│  • J3: Non-blocking Scheduling & Latest-Value-Wins (LVW) Atomicity      │
│  • J4: Stateful Jerk-Limited Trajectory Shaping (C² continuous motion)  │
│  • J5: Monotonic Nanosecond Timeline & Analytical-Jacobian LM FK        │
│  • J6: Mechanical Assembly & Centralized HOME/PARK State Classification │
│  • J7: 50Hz Real-Hardware Servo Driver & Design-B Accompanying Feedback │
│  • J8: 9-Phase Cartesian Video Demonstration (100% feedback, 0 faults)  │
│  • J9: Dual Yesense H30 IMU Integration & SO(3) Attitude Integration    │
│  • J10: 6-DOF Single-Axis Safe Operational Workspace Envelope Characterization
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│               Part II — Active Stabilization Research (J11+)            │
│                       [STATUS: ACTIVE ROADMAP]                          │
│  • J11-M1: Base-IMU Feedforward Attitude Hold & Driver Runtime Qualified│
│            [STATUS: COMPLETED & FROZEN — Tag: v1.0-j11-m1]              │
│  • J11: Base-IMU SO(3) Feedforward + Platform-IMU P/PD Attitude Feedback │
│         [STATUS: COMPLETED & FROZEN — Tag: v1.0-j11]                     │
│  • J12: Classical Dynamic Performance Characterization                   │
│         [STATUS: COMPLETED & FROZEN — Tag: v1.0-j12]                     │
│  • J13: First-Order Linear ADRC / LADRC Residual Control                 │
│         [STATUS: CLOSED / ENGINEERING QUALIFIED]                        │
│  • Current Active: J14 — Long-Duration Runtime & Communication Reliability │
│    - Eliminate UART synchronous blocking and scheduler long-tail timing  │
│    - Strict invariants: 80-step recovery / 283.3ms warning / 1000ms stop │
│    - Qualification Target: 10-minute continuous 50Hz active execution    │
│    - Future Research: Eccentric payload transient disturbance comparison │
└─────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **Honesty & Verification Boundary**: J1-J10, J11, and J12 are frozen; J13 is closed as engineering qualified. All metrics are supported by verified real-hardware telemetry. In eccentric payload tests, manual Base excitation was non-identical; no claim of comprehensive LADRC superiority over Classical control is made.

### J10 Single-Axis Operational Workspace Envelope (HOME-Centered)

| DOF | Operational Bound | Limiting Factor |
| :--- | :--- | :--- |
| **X** | `[-45.75, +45.75] mm` | Actuator stroke envelope (ID2 / ID5) |
| **Y** | `[-45.50, +49.00] mm` | Actuator stroke envelope (ID1 / ID4) |
| **Z** | `[-25.00, +25.00] mm` | 50.0 mm total symmetric common-mode heave |
| **Roll** | `[-35.0, +37.5] deg` | Mechanical spherical joint boundary |
| **Pitch** | `[-30.0, +35.0] deg` | Actuator travel limits |
| **Yaw** | `[-90.0, +90.0] deg` | Full qualified rotational envelope |

*(Note: These ranges represent independent single-axis explorations centered at `OPERATING_HOME = [1000]*6` steps. Full details are documented in [`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](./agent_docs/stages/J10/J10_FINAL_SUMMARY.md)).*

### Real-Hardware Positioning & Attitude Tracking Performance Near HOME

These results describe small-range, single-axis **command-to-feedback / FK / IMU tracking performance near HOME**. They are not whole-workspace absolute positioning accuracy, external-metrology calibration, or repeatability specifications.

| Test | Command / Condition | Measured Response | Metric | Interpretation |
| --- | --- | --- | --- | --- |
| Actuator position resolution | Configuration mapping: 40 steps/mm | 0.025 mm/step | Joint-command quantization from `steps_per_meter` / `meters_per_step` | Actuator/joint resolution; not TCP Cartesian positioning accuracy |
| X translation | +0.5 / -0.5 mm | FK local displacement +0.3311 / -0.3991 mm; absolute command-response difference 0.1689 / 0.1009 mm | Mean of the last 100 action-stage FK samples minus mean of the last 100 pre-HOME FK samples | Local, model-based command-to-FK tracking in two runs |
| Y translation | +0.5 / -0.5 mm | FK local displacement +0.5184 / -0.4957 mm; absolute difference 0.0184 / 0.0043 mm | Same last-100-sample action-minus-pre-HOME FK mean | Local, model-based command-to-FK tracking in two runs |
| Z translation | +0.5 / -0.5 mm | FK local displacement +0.4769 / -0.5018 mm; absolute difference 0.0231 / 0.0018 mm | Same last-100-sample action-minus-pre-HOME FK mean | Local, model-based tracking; the +Z run also shows cross-axis coupling |
| Roll | +0.5 / -0.5 deg | Platform-IMU mean +0.4486 / -0.5083 deg; signed mean command-minus-response error +0.0514 / +0.0083 deg | Arithmetic mean over the latter half of the action stage | Near-HOME small-angle tracking; direction-dependent in the available runs |
| Pitch | +0.5 / -0.5 deg | Platform-IMU mean +0.4995 / -0.5213 deg; signed mean error +0.0005 / +0.0213 deg | Same latter-half action-stage mean | Near-HOME small-angle tracking; direction-dependent in the available runs |
| Yaw | +2.0 deg | Platform-IMU mean +1.4825 deg; signed mean error +0.5175 deg | Same latter-half action-stage mean | Materially weaker than Roll/Pitch; no uniform attitude-accuracy claim is supported |
| Whole-workspace absolute positioning accuracy | No dedicated external calibration | Unknown | Not measured | Not currently qualified |
| Repeatability | No systematic repeated-point statistics | Unknown | Not measured | Not currently qualified |

The axis- and direction-dependent results above use explicitly defined, internally consistent metric families: one for J9.1 translation and one for J9.1 attitude. They support a local command-to-feedback characterization near HOME, not a single Cartesian or attitude accuracy specification. Whole-workspace absolute accuracy and repeatability remain to be established by dedicated external metrology and repeated-point statistics. See the [`detailed evidence summary`](./agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md).

### J11 Dual-IMU Attitude Stabilization (Stage Result)

**Objective**: when the Base is disturbed in attitude, use the dual IMUs + 6PUS active compensation to keep the Platform as close as possible to its initial 3-axis attitude.

**Approach**:

- **M1 (Feedforward, frozen)**: Base-IMU full SO(3) Feedforward; Platform IMU is observer/validation only.
- **M2 (Feedforward + Feedback, final)**: Base-IMU SO(3) Feedforward + Platform-IMU Classical P/PD Attitude Feedback (Roll=P, Pitch=PD, Yaw=PD).

**Final gains**:

```text
Kp = [0.05, 0.05, 0.025]
Ki = [0.0, 0.0, 0.0]
Kd = [0.0, 0.0015, 0.0015] s
Derivative LPF fc = 5 Hz
Residual limit = ±0.5° / axis
```

**Metrics**: `Peak = max_k |θ_k|` (worst-case deviation) and `RMS = sqrt((1/N) Σ θ_k²)` (overall error level). `G_Peak = Platform Peak / Base Peak` and `G_RMS = Platform RMS / Base RMS` are the normalized Platform/Base attitude-response ratios — smaller G means stronger attitude-disturbance rejection. Roll/Pitch/Yaw describe individual attitude components; the SO(3) attitude error describes the overall 3-D rotational error relative to the target attitude.

**Final result**:

| Metric | M1 Feedforward | M2 Feedforward + P/PD | Reduction |
|---|---:|---:|---:|
| Roll G_RMS | 0.4071 | 0.2662 | 34.6% |
| Pitch G_RMS | 0.4153 | 0.2607 | 37.2% |
| Yaw G_RMS | 0.2912 | 0.2297 | 21.1% |
| SO(3) G_Peak | 0.7353 | 0.3063 | 58.3% |
| SO(3) G_RMS | 0.3713 | 0.2575 | 30.6% |

`Reduction = (1 - G_M2 / G_M1) × 100%`. Full 8-metric table and calculation details: [`docs/j11/J11_FINAL_CONTROL_COMPARISON.md`](./docs/j11/J11_FINAL_CONTROL_COMPARISON.md).

> These percentages reflect the observed normalized attitude-response reductions in two representative real-hardware experiments. Because the two manual Base disturbance trajectories were not strictly identical, the result is a normalized hardware characterization rather than a strict identical-input transfer-function gain.

**Data Sources**:

- M1 reference dataset: `results/j11/runtime_validation/20260918_112354/J11_DISTURB_QUAL_feedforward_powered.csv`
- M2 reference dataset: `results/j11/m2_pid/20260920_095850/j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_telemetry_powered.csv`

These raw CSVs are the starting point for any future re-plotting or re-computation of Peak / RMS / G_Peak / G_RMS.

### J11 Demos

No manual ROS 2 or workspace sourcing is required in a fresh terminal. Both wrappers load ROS 2 Jazzy and `bone_ws/install/setup.bash` themselves, while remaining compatible with an already-sourced development terminal.

Feedforward only:

```bash
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j11_demo_feedforward.sh
```

Feedforward + P/PD:

```bash
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j11_demo_feedforward_pd.sh
```

**Status**: J11 COMPLETED & FROZEN (`v1.0-j11-attitude-stabilization-frozen`) — Demo A (Feedforward) PASS; Demo B (Feedforward + P/PD) PASS ON REPEAT.

### J12 Classical Dynamic Performance Characterization (Stage Result)

- **Status**: **COMPLETED & FROZEN** (`v1.0-j12-classical-dynamic-performance-frozen`).
- **Scope**: Large-Angle Static Load Qualification (J12-1), 3DOF Combined Attitude Screening (J12-2), and Handheld Dynamic Performance Characterization (J12-3).
- **Key Findings**:
  - Validated classical feedforward + P/PD disturbance rejection under handheld dynamic motion up to observed effective peak angular rate of `92.33°/s` (P95 `46.16°/s`).
  - Maximum observed actuator current remained well below limits at `468 mA` (0 samples > 500 mA). Bottleneck identified as control residual saturation (±0.5°) and actuator tracking lag, not motor current.
  - Production host current limit formally aligned with actuator hardware protection register at `MAX_CURRENT_MA = 1500`.
  - Detailed findings: [`agent_docs/J12_FINAL_SUMMARY.md`](./agent_docs/J12_FINAL_SUMMARY.md); Handoff: [`agent_docs/J12_TO_J13_HANDOFF.md`](./agent_docs/J12_TO_J13_HANDOFF.md).

### J13 First-Order Linear ADRC Attitude Residual Control and Eccentric-Payload Hardware Validation (Stage Result)

- **Status**: **CLOSED / ENGINEERING QUALIFIED** (Release Tag: `v1.0-j13-ladrc-attitude-residual-control-qualified`, Final Commit: `89d38b3ac2bbb63234aa56f8ecd87178010dde76`).
- **Architecture**:
  - **3-axis LADRC**: Base $SO(3)$ Feedforward + 3-axis First-Order Linear ADRC / LADRC Residual Control (exact ZOH discrete second-order LESO).
  - **Classical Baseline**: Base $SO(3)$ Feedforward + Roll P + Pitch PD + Yaw PD.
- **Final LADRC Parameters**:

| Axis | $b_0$ | $\omega_c$ | $\omega_o$ | Residual Clamp |
| :--- | :---: | :---: | :---: | :---: |
| **Roll** | 20.0 | 0.8 rad/s | 2.5 rad/s | $\pm 0.5^\circ$ |
| **Pitch** | 10.0 | 1.0 rad/s | 4.0 rad/s | $\pm 0.5^\circ$ |
| **Yaw** | 6.0 | 0.25 rad/s | 1.2 rad/s | $\pm 0.5^\circ$ |

*(Definitions: $b_0$ = nominal input-gain estimate, $\omega_c$ = controller bandwidth, $\omega_o$ = observer bandwidth).*

- **Real-Hardware Qualification & Experimental Execution**:
  1. **Empty-load 10s qualification**: 500 cycles (~9.98 s), 430 active cycles, 0 hard stops, 0 motor faults, 0 recovery rejects, observer strictly stable. **Conclusion: PASS / QUALIFIED**.
  2. **Eccentric-payload hardware experiment**: Classical run (500 cycles, 429 active cycles) vs. 3-axis LADRC run (500 cycles, 446 active cycles). Both runs completed without fatal runtime failures.

- **Eccentric-Payload A/B Comparison & Evidence Boundaries**:
Platform $SO(3)$ attitude tracking error RMS:

| Axis | Classical RMS | 3-axis LADRC RMS | Base RMS Excitation Ratio (LADRC / Classical) |
| :--- | :---: | :---: | :---: |
| **Roll** | $0.446^\circ$ | $1.072^\circ$ | 1.896× |
| **Pitch** | $0.868^\circ$ | $1.309^\circ$ | 1.501× |
| **Yaw** | $0.608^\circ$ | $0.441^\circ$ | 0.525× |

> [!WARNING]
> **No claim of comprehensive LADRC superiority over Classical control is made.** Because manual Base excitations were non-identical between the two runs (LADRC Base Roll excitation was 1.896× Classical, Pitch was 1.501×, while Yaw was 0.525×), the observed higher Roll/Pitch errors and lower Yaw error in the LADRC run cannot be attributed solely to the controller.

- **Key Engineering Findings**:
  - **Pitch Residual Saturation**: Pitch LADRC saturated in 73 / 446 active samples (**16.37%**), with $u_\text{raw}$ peak reaching $\approx 1.025^\circ$ before being clipped to $\pm 0.5^\circ$. This highlights the engineering boundary of Pitch residual authority and parameter matching under strong excitation, reflecting expected control limits rather than algorithm failure.
  - **LESO Stability**: Observer states $z_1$ and $z_2$ remained strictly stable throughout all runs. State $z_2$ represents the *Equivalent Lumped Disturbance* (等效集总扰动项), not physical disturbance torque.
  - **Runtime & Communication Long-Tail Latency**: J13 runs uncovered sporadic communication and scheduling jitter over the RS485 bus:
    - Max Scheduler Gap: Classical $\approx 274.9\text{ ms}$, LADRC $\approx 370.6\text{ ms}$
    - Max UART Drain: Classical $\approx 182.6\text{ ms}$, LADRC $\approx 363.1\text{ ms}$
    - Max Feedback Age: Classical $\approx 269.9\text{ ms}$, LADRC $\approx 352.4\text{ ms}$
    While 10-second runs execute reliably, this timing long-tail limits continuous multi-minute to 10-minute operation.

- **Core Conclusions**:
  - First-Order 3-axis LADRC successfully deployed and qualified on real 6PUS hardware;
  - 10s empty-load qualification PASS; eccentric payload hardware runs completed; LESO states numerically stable;
  - Existing evidence does not prove LADRC superiority over Classical control; Pitch exhibits noticeable residual saturation (16.37%);
  - Runtime timing long-tail handed off to J14.
- **Detailed Documentation**:
  - Detailed Stage Summary: [`agent_docs/J13_FINAL_SUMMARY.md`](./agent_docs/J13_FINAL_SUMMARY.md)
  - Stage Handoff: [`agent_docs/J13_TO_J14_HANDOFF.md`](./agent_docs/J13_TO_J14_HANDOFF.md)

**Next**: J14 — Long-Duration Runtime & Communication Reliability
- **J14-A**: 10-minute continuous runtime & communication qualification (eliminate UART blocking and scheduler starvation).
- **J14-Q1**: ROS 2 QoS Deadline / Liveliness fault-detection characterization (standby/planned non-blocking track).
- **J14-B**: Following J14-A qualification PASS, conduct controlled eccentric-payload + manual end-effector transient disturbance comparison between Classical and 3-axis LADRC.

---

## 3. Repository Layout

The repository is structured with a strict separation of concerns between code, human/agent documentation, and machine-generated experimental artifacts:

```text
/home/lyapunov/bone_drill/
├── bone_ws/                               <-- [ROS 2 Workspace] Source packages, nodes, unit tests, experiment scripts
│   ├── src/
│   │   ├── pus6_msgs/                     # Custom ROS 2 msg definitions (TimedJointFeedback, FkStateStamped, etc.)
│   │   ├── pus6_kinematics/               # Analytical IK, LM numerical FK, 6pus_geometry.yaml single truth source
│   │   ├── pus6_motor_driver/             # RS485 UART driver, 50Hz servo transport, 2-tier safety watchdogs
│   │   ├── pus6_control/                  # Stateful jerk-limited trajectory shaper & trajectory controllers
│   │   ├── pus6_bringup/                  # Launch files and parameter configurations
│   │   └── h30_imu/                       # Yesense H30 IMU C++ driver & SO(3) relative attitude estimator
│   └── scripts/                           # Reproducible experiment runners & verification scripts
│
├── agent_docs/                            <-- [Agent-facing internal docs] stage history, decisions, tuning, handoff
│   ├── stages/                            # Stage plans (J1..J11), final closure summaries, cross-stage handoffs
│   ├── reports/                           # Empirical analysis reports (J10 workspace characterization, J9.1 lever arm)
│   ├── decisions/                         # Architecture Decision Records (ADRs) & hardware qualification logs
│   ├── frozen/                            # Read-only frozen baseline manifests & integrity checksums
│   └── archive/                           # Legacy operational manuals and superseded references
│
├── results/                               <-- [Experimental Evidence] Machine-generated data, telemetry & artifacts
│   ├── README.md                          # Evidence catalog and dataset index
│   ├── stewart3_j8_figure6_20260916_.../  # Canonical J8 9-phase Cartesian real-hardware dataset (Stewart-3 Fig 6)
│   ├── j7/                                # Canonical J7 preflight and 50Hz servo benchmarks
│   ├── j9_1/                              # Canonical J9.1 dual-IMU dynamic attitude and coupling datasets
│   └── j10/                               # Canonical J10 axis-wise workspace boundary telemetry summaries
│
├── hardware/                              <-- [Hardware Specs] Actuator datasheets, wiring pinouts, CAD geometry
├── up/                                    <-- [Low-Level UART Baseline] Standalone multi-actuator serial validation bench
├── STM32/                                 <-- [Firmware] Embedded controller sources
├── docs/                                 <-- [Human-facing docs] tutorials, operation guide, final results
└── AGENTS.md                              <-- [Agent Operational Rules] Global safety guardrails and engineering invariants
```

> **Separation of Concerns**:
> - `docs/` = Human-facing documentation (tutorials, operation guide, final technical results).
> - `agent_docs/` = Agent-facing internal documentation (stage history, decisions, tuning history, handoff, fault audits).
> - `results/` = Machine-generated experimental evidence, CSVs, telemetry JSONs, and raw execution logs.

---

## 4. Recommended Reading Order

For engineers, researchers, and automated agents engaging with this repository for the first time:

1. **System Overview & Baseline**: Read this [`README.md`](./README.md) (or [`README_CN.md`](./README_CN.md)) for the high-level architecture and current status.
2. **Full-Stack Tutorial**: Work through [`docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](./docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md) for an in-depth breakdown of the dual-pipeline architecture, inverse/forward kinematics, and 50Hz servo driver.
3. **Documentation Navigation**: Consult [`agent_docs/README.md`](./agent_docs/README.md) to locate specific stage implementation plans, closure reports, and cross-stage handoff contracts.
4. **Physical Hardware Operations**: If operating real actuators, review [`docs/整体学习参考/6PUS_SERVO_OPERATION_GUIDE.md`](./docs/整体学习参考/6PUS_SERVO_OPERATION_GUIDE.md) before powering on.
5. **Canonical Datasets & Evidence**: Explore [`results/README.md`](./results/README.md) to inspect real 50Hz Cartesian flight data (such as the 9-phase J8 demonstration dataset).

---

## 5. Offline Public Reproducibility (No Physical Hardware Required)

Any third-party researcher can build, inspect, and verify the codebase without access to the physical robot platform:

### 1. Prerequisites & Environment Setup
- **OS**: Ubuntu 24.04 LTS
- **ROS 2**: Jazzy Jalisco (or compatible ROS 2 distribution)
- **Python**: $\ge 3.10$

```bash
# Clone the repository
git clone https://github.com/lyapunov/bone_drill.git
cd bone_drill

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
```

### 2. Build the Workspace
```bash
cd bone_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Run Offline Unit & Integration Tests (Mock Mode)
The test suite executes with `hardware_mode:=mock` and **strictly asserts **`hardware_write_count = 0`****:
```bash
# Run all 346+ package tests across bone_ws
colcon test --ctest-args -R test
colcon test-result --all

# Run the J8 Figure 6 recorder qualification test
pytest scripts/test_j8_figure6_recorder.py -v
```

### 4. Inspect Real Physical Experimental Data
Load the canonical 9-phase J8 Cartesian run directly:
```python
import pandas as pd
import json

# Inspect 50Hz Cartesian command vs. actual FK pose
dataset_dir = "../results/stewart3_j8_figure6_20260916_135013"
cmd_df = pd.read_csv(f"{dataset_dir}/command_steps.csv")
fb_df = pd.read_csv(f"{dataset_dir}/joint_feedback.csv")
fk_df = pd.read_csv(f"{dataset_dir}/fk.csv")

print(f"Recorded cycles: {len(cmd_df)} @ 50 Hz (~{len(cmd_df)/50:.1f}s)")
print("Columns in feedback:", list(fb_df.columns[:8]))
```

---

## 6. Safety Guardrails & Real-Hardware Operation

- **Zero-Motion Default**: All nodes, tests, and runners default to `Mock / FakeBus` mode.
- **Explicit Authorization**: Activating the physical RS485 bus (`--real-hardware` / `hardware_mode:=active`) requires verified valid homing, physical emergency-stop readiness, and explicit human confirmation.
- **Zero-Retry Motion Commands**: Write commands on the RS485 bus never retry automatically; communication anomalies trigger a controlled single pause command (0x1A) and an orderly stop.
