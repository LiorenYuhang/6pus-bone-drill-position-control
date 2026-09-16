# 6PUS-Anti-Vibration

> [English] | [中文](README_CN.md)  
> **A ROS 2 based real-time sensing and control framework for a 6PUS parallel surgical robot with active vibration suppression.**

---

## 1. Project Overview

**6PUS-Anti-Vibration** is an open, high-precision, real-time sensing and motion control framework designed for a 6-PUS (Prismatic-Universal-Spherical) parallel manipulator / Stewart platform. Developed for precision orthopaedic surgical robotics, the system provides high-bandwidth, deterministic Cartesian motion alongside sub-millisecond multi-sensor observation for active disturbance rejection and vibration suppression.

### Key Milestones:
- **J1–J10 (Completed & Frozen)**: Comprehensive real-hardware foundation covering low-level multi-actuator UART communication, analytical inverse kinematics, $C^2$ jerk-limited trajectory shaping, 50 Hz continuous servo streaming, dual-IMU microsecond clock synchronization, numerical forward kinematics, and complete 6-DOF single-axis operational workspace qualification on the physical platform.
- **J11+ (Active Research Phase)**: Active attitude stabilization and anti-vibration research under dynamic base disturbances (Stage J11 PID world-attitude closed-loop control currently in progress; advanced ADRC and adaptive control on the future roadmap).

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
│  • J11: PID 3-DOF World-Attitude Disturbance Rejection [IN PROGRESS]    │
│  • Future Research Roadmap:                                             │
│    - Active Disturbance Rejection Control (ADRC) & Extended State Obs   │
│    - IMU Acceleration Feedforward & Disturbance Observer (DOB)          │
│    - Learning-Based Adaptive Trajectory & Stiffness Compensation        │
└─────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **Honesty & Verification Boundary**: Stages J1 through J10 are fully qualified on real physical hardware and frozen (`v1.0-j8-active-servo-hardware-qualified`). Stage J11 is actively in development (pure offline simulations/dry-runs have been validated; active closed-loop real-hardware motion is pending qualification). No unverified claims are made for future phases.

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
├── agent_docs/                            <-- [Project Knowledge Base] Human/Agent-readable technical documentation
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
├── STM32/ / teach/                        <-- [Firmware & Pendant] Embedded controller firmware and teaching pendant tools
├── 6PUS_REAL_HARDWARE_OPERATION_GUIDE.md  <-- [Real-Hardware Guide] Operational checklist for physical power/homing/shutdown
└── AGENTS.md                              <-- [Agent Operational Rules] Global safety guardrails and engineering invariants
```

> **Separation of Concerns**:
> - `agent_docs/` = Human/Agent-readable knowledge, architectures, and design decisions.
> - `results/` = Machine-generated experimental evidence, CSVs, telemetry JSONs, and raw execution logs.

---

## 4. Recommended Reading Order

For engineers, researchers, and automated agents engaging with this repository for the first time:

1. **System Overview & Baseline**: Read this [`README.md`](./README.md) (or [`README_CN.md`](./README_CN.md)) for the high-level architecture and current status.
2. **Full-Stack Tutorial**: Work through [`6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](./6PUS_ROS2_CODE_LEARNING_TUTORIAL.md) for an in-depth breakdown of the dual-pipeline architecture, inverse/forward kinematics, and 50Hz servo driver.
3. **Documentation Navigation**: Consult [`agent_docs/README.md`](./agent_docs/README.md) to locate specific stage implementation plans, closure reports, and cross-stage handoff contracts.
4. **Physical Hardware Operations**: If operating real actuators, review [`6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`](./6PUS_REAL_HARDWARE_OPERATION_GUIDE.md) before powering on.
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
The test suite executes with `hardware_mode:=mock` and **strictly asserts $\mathbf{\text{hardware\_write\_count} = 0}$**:
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
