# 6PUS-Anti-Vibration

> [English] | [中文](README_CN.md)
> **A ROS 2 based real-time sensing and control framework for a 6PUS parallel surgical robot with active vibration suppression.**

---

## 1. Project Overview

**6PUS-Anti-Vibration** is an open ROS 2 sensing and motion-control framework for a 6-PUS (Prismatic-Universal-Spherical) parallel manipulator / Stewart platform, developed for orthopaedic robotics research. It combines Cartesian motion, trajectory shaping, and high-frequency multi-sensor observation for active attitude stabilization. Current capabilities, hardware validation, and limitations are described below.

---

## 2. Current Development Status

### 2.1 Active Attitude Stabilization

The platform aims to keep the Platform close to its reference attitude while the Base moves. Three schemes have been implemented and operated on real 6PUS hardware:

| Scheme | Method |
| --- | --- |
| **Base SO(3) Feedforward** | Base IMU supplies the base attitude for geometric compensation. Platform IMU observes the response for validation, without residual feedback. |
| **Feedforward + Classical P/PD** | Adds Platform IMU attitude residual feedback to the common feedforward path. The retained implementation is **Roll=P, Pitch/Yaw=PD, Ki=0**. |
| **Feedforward + First-order LADRC** | Retains the common feedforward path; three independent linear extended state observers (LESO) estimate equivalent lumped disturbances for first-order LADRC residual compensation. |

The technical route is **geometric compensation → attitude residual feedback → residual control with disturbance observation**. **P/PD and LADRC are two residual-control schemes built on the same feedforward foundation.**

Hardware evidence includes feedforward/P/PD comparisons, LADRC empty-load qualification, and Classical P/PD/LADRC operation under an eccentric payload. These establish implementation and comparative characterization; they do not establish a uniform three-scheme ranking or comprehensive LADRC superiority. Historical sources are linked in Section 2.5.

### 2.2 Runtime & Communication Reliability

Runtime engineering addresses UART blocking, TX timeouts, stale feedback, startup packet loss, and ROS executor starvation:

- **Three-process architecture:** Parent manages ROS command ingress and coordination; UART Child owns the motor UART; Feedback Bridge consumes feedback IPC and publishes ROS feedback. **Compatibility Relay is an external adapter, not one of the three core processes.**
- **Communication isolation:** serial execution and feedback publication are separated from Parent callbacks; feedback continuity, gaps, and age are monitored. Diagnostics are separated from the critical communication path.
- **Coordinated startup:** a readiness handshake precedes servo execution. Parent uses `SingleThreadedExecutor` to address the identified IPC lease starvation under active subscription workloads.
- **Lifecycle handling:** the current path supports timeout detection, controlled pause, and shutdown. Normal Ctrl+C shutdown follows HOME return → six-axis pause → cleanup. Fault shutdown follows fault detection → pause → cleanup, skipping HOME return. Automatic recovery and resume are not supported or qualified.

| Recorded real-hardware qualification | Scope |
| --- | --- |
| **600s static qualification** | Static servo runtime and feedback continuity; scheduler re-anchor events were recorded. |
| **Classical P/PD: 60s active session** | Nominal controller session including preflight/reference capture and ACTIVE operation. |
| **LADRC: 60s active session** | Nominal controller session including preflight/reference capture and ACTIVE operation. |

These validations apply to their recorded conditions and windows. Static qualification does not extend the active-control duration; continuity does not imply jitter-free execution.

### 2.3 Motion & Sensing Foundation

- Six-axis UART servo communication with coordinated commands and accompanying feedback.
- Analytical inverse kinematics (IK), numerical forward kinematics (FK), and stateful jerk-limited trajectory shaping.
- Dual-IMU attitude sensing, time alignment, and SO(3) relative-attitude processing.
- `OPERATING_HOME` is the operating reference at approximately **1000 steps per axis**. Active stabilization starts after startup confirmation of this reference. `STORAGE_PARK` is the storage/power-off state; HOME return and shutdown are managed separately from mechanical homing.
- Near-HOME local command-to-feedback/FK/IMU tracking and HOME-centered single-axis envelope characterization.

Recorded hardware motion, sensing, and characterization evidence is linked in Section 2.5.

### 2.4 Validation Scope & Known Limits

- **Workspace:** HOME-centered single-axis envelopes do not establish a complete coupled six-dimensional workspace or qualify arbitrary combined poses.
- **Accuracy:** FK/feedback/IMU tracking and actuator resolution do not establish external absolute positioning accuracy or repeatability; these remain unqualified.
- **Comparisons:** manual Base disturbances differ between runs, preventing a strict controller ranking.
- **Control authority:** strong disturbances can cause residual saturation and actuator tracking lag; disturbance compensation does not eliminate every disturbance.
- **Stability:** LESO states were **bounded within recorded operating windows**. This observation is not a general theoretical stability proof.
- **Recovery:** timeout detection, pause, and shutdown are available; automatic recovery and control resumption are not. Scheduler re-anchoring is distinct from communication recovery.

### 2.5 Demos & Technical References

The three Demo modes use the common production runtime. The [Demo operation guide](./docs/j14/J14_DEMO_ENTRYPOINTS.md) records human hardware confirmation of startup, ACTIVE operation, normal HOME-return shutdown, pause acknowledgement, and resource release, and provides operating prerequisites and commands.

| Mode | Entrypoint |
| --- | --- |
| **Base SO(3) Feedforward** | [run_j14_demo_feedforward.sh](./bone_ws/scripts/run_j14_demo_feedforward.sh) |
| **Feedforward + Classical P/PD** | [run_j14_demo_feedforward_pd.sh](./bone_ws/scripts/run_j14_demo_feedforward_pd.sh) |
| **Feedforward + First-order LADRC** | [run_j14_demo_feedforward_ladrc.sh](./bone_ws/scripts/run_j14_demo_feedforward_ladrc.sh) |

Technical references: [user documentation](./docs/), [production driver operation](./docs/j14/J14_PRODUCTION_DRIVER.md), [feedforward/P/PD comparison and metrics](./docs/j11/J11_FINAL_CONTROL_COMPARISON.md), [near-HOME tracking evidence](./agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md), and [raw evidence index](./results/README.md).

**Historical source baselines:** stage identifiers below preserve evidence provenance, not separate current capabilities. Historical summaries retain their original test conditions and status wording; they do not define current production capabilities.

| Historical source baseline | Source / version |
| --- | --- |
| Motion characterization | [J10](./agent_docs/stages/J10/J10_FINAL_SUMMARY.md) — `v1.0-j10-axis-wise-workspace-qualified` |
| Feedforward qualification | [J11-M1](./agent_docs/stages/J11/J11_M1_FINAL_SUMMARY.md) — `v1.0-j11-m1-feedforward-hardware-qualified` |
| Classical P/PD baseline | [J11](./agent_docs/stages/J11/J11_FINAL_SUMMARY.md) — `v1.0-j11-attitude-stabilization-frozen` |
| Classical dynamic/load characterization | [J12](./agent_docs/stages/J12/J12_FINAL_SUMMARY.md) — `v1.0-j12-classical-dynamic-performance-frozen` |
| LADRC engineering qualification | [J13](./agent_docs/stages/J13/J13_FINAL_SUMMARY.md) — `v1.0-j13-ladrc-attitude-residual-control-qualified` |
| Runtime reliability baseline (CLOSED / FROZEN) | [J14-A](./agent_docs/stages/J14/J14_FINAL_SUMMARY.md) — `v1.0-j14-a-three-process-runtime-reliability-qualified` |

The [internal documentation index](./agent_docs/README.md) retains stage history, detailed analyses, and handoffs.

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
│   ├── stages/                            # Stage plans, final closure summaries, cross-stage handoffs
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
Run software tests in Mock / FakeBus mode:
```bash
# Run package tests across bone_ws
colcon test --ctest-args -R test
colcon test-result --all

# Run the J8 Figure 6 recorder qualification test
pytest scripts/test_j8_figure6_recorder.py -v
```

### 4. Inspect Recorded Hardware Evidence

Use the [raw evidence index](./results/README.md) to locate recorded datasets and their analysis references. The [control comparison](./docs/j11/J11_FINAL_CONTROL_COMPARISON.md) and [near-HOME tracking summary](./agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md) define the relevant metrics and evidence boundaries.

---

## 6. Safety Guardrails & Real-Hardware Operation

- **Zero-Motion Default**: All nodes, tests, and runners default to `Mock / FakeBus` mode.
- **Explicit Authorization**: Activating the physical RS485 bus (`--real-hardware` / `hardware_mode:=active`) requires a confirmed operating reference, physical emergency-stop readiness, and explicit human confirmation. Active stabilization uses `OPERATING_HOME`, not `STORAGE_PARK`.
- **Zero-Retry Motion Commands**: Write commands on the RS485 bus never retry automatically; communication anomalies trigger a controlled single pause command (0x1A) and an orderly stop.
