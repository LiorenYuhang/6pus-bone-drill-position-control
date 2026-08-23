# 6PUS-Anti-Vibration

> [English] | [中文](README_CN.md)  
> **A ROS 2 based real-time sensing and control framework for a 6PUS parallel platform with active vibration suppression.**

---

## Overview

**6PUS-Anti-Vibration** is an open, high-precision, real-time sensing and motion control framework designed for a 6-PUS (Prismatic-Universal-Spherical) parallel manipulator / Stewart platform, tailored for active disturbance rejection and vibration suppression in precision surgical and robotic tasks.

Key capabilities include:
* **6PUS Parallel Platform Mechanics**: Closed-form analytical inverse kinematics and Levenberg-Marquardt numerical forward kinematics with analytical Jacobians.
* **Active Vibration Suppression**: Continuous $C^2$ jerk-limited trajectory shaping, low-latency joint dispatching, and high-bandwidth closed-loop control architecture.
* **Real-Time ROS 2 Architecture**: Strictly decoupled dual-pipeline design separating the 50Hz control loop from the 200Hz kinematic observation and multi-sensor pipeline.
* **Dual IMU Sensing**: High-rate (200Hz) dual Yesense H30 IMU hardware timestamping (microsecond 0x51/0x52 sampling sync), software clock skew compensation, session-start adaptive gyro bias estimation, and Lie-group $\text{SO}(3)$ relative attitude integration.
* **Serial Telemetry & Kinematic Observation**: Single-bus half-duplex RS485 921600 8N1 serial driver with Batch-Atomicity bus locking, host `CLOCK_MONOTONIC` timestamping, strictly bounded Fixed-Lag zero-extrapolation timeline reconstruction, and dedicated single-threaded FK state estimation.

---

## Project Status

**Current Milestone**: $\mathbf{J5\ PRE\text{-}ASSEMBLY\ BASELINE\ FROZEN\ v1.0}$  
*(Tag: `v1.0-j5-pre-assembly-frozen`)*

The pre-assembly software and independent actuator bench baseline has been fully verified and frozen:
* **RS485 Six-Actuator Communication**: 10.0Hz outer polling, 18.18ms serial communication duration, 15.18ms scan span, zero-overlap batch atomicity.
* **READ_ONLY Hardware Verification**: 60.0s continuous serial telemetry capture with strict `hardware_writes = 0` audit.
* **HOME / PARK Bench Validation**: Independent actuator S-curve homing (`[1000]*6` raw steps, 50.0mm) and parking (`[0]*6` raw steps) with steady-state error $\le 2\text{ steps}$.
* **Dual H30 IMU Timestamp Synchronization**: Dual-channel 100% active 0x51 hardware sample timestamps ($\Delta t = 5000\,\mu\text{s}$), zero drop rate.
* **Relative Attitude & Gyro Bias Estimation**: Session-start 6.0s 1200-sample 5% trimmed mean auto-bias estimation (`COMPLETE`), 122Hz $\text{SO}(3)$ relative attitude publishing, and co-rigid dynamic disturbance RMS $0.2767^\circ$ at $80.8^\circ/\text{s}$.
* **Timed Telemetry (`TimedJointFeedback`)**: Unified nanosecond timeline tagged at serial reception done (`per_axis_rx_done_monotonic_ns`).
* **Fixed-Lag State Reconstruction**: Sliding timeline synchronizer with strict zero-extrapolation gating (97.07% synced, deterministic WAIT under microsecond UART jitter).
* **Real-Time FK Observation (`Pus6FkNode`)**: Dedicated `SingleThreadedExecutor` achieving 90.6Hz execution rate with mean geometric residual $< 10^{-11}\text{ m}$.
* **Core Unit Test Suite**: 162 tests passed (100% PASS in 37.02s).

---

## Architecture

```text
┌───────────────────────────────────────────────────────────────────────┐
│                      Upper Planning / Target Stream                   │
└───────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼  (/pus6/target_cartesian_pose)
┌───────────────────────────────────────────────────────────────────────┐
│                      Kinematics & Control Layer                       │
│  • pus6_kinematics_node : Analytical Inverse Kinematics (50Hz)        │
│  • pus6_control_node    : Stateful Jerk-Limited Trajectory Shaper     │
└───────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼  (/pus6/command_joint_steps, LVW)
┌───────────────────────────────────────────────────────────────────────┐
│                          Motor Driver Layer                           │
│  • pus6_motor_driver_node: 6-Stage Safety Preflight & Batch Lock      │
│  • RS485 Half-Duplex Single Bus (921600 8N1, 0x29 Piggyback Protocol) │
└───────────────────────────────────────────────────────────────────────┘
             │                                              │
             ▼ (Motor Telemetry)                            ▼ (Hardware)
┌──────────────────────────────────────┐       ┌────────────────────────┐
│      Observation & Sensing Layer     │       │     Hardware Layer     │
│  • TimedJointFeedback Publisher      │       │  • 6x Linear Actuators │
│  • FixedLagJointSynchronizer         │       │  • Base H30 IMU        │
│  • Pus6FkNode (Dedicated Executor)   │       │  • Platform H30 IMU    │
│  • h30_relative_attitude_node (SO3)  │       │  • 6PUS Stewart Base   │
└──────────────────────────────────────┘       └────────────────────────┘
```

---

## Repository Structure

```text
6PUS-Anti-Vibration/
├── agent_docs/                     # Authoritative design, baseline, and test documentation
│   ├── CURRENT_STAGE.md            # Global development stage tracking card
│   ├── J5_FROZEN_MANIFEST.md       # J5 release scope, verified features & roadmap
│   ├── J5_FINAL_TEST_REPORT.md     # Comprehensive 12-item test acceptance report
│   ├── J5_SOFTWARE_ENVIRONMENT.md  # OS, ROS 2, Python, and By-ID serial specification
│   ├── J5_TUTORIAL_EXECUTION_AUDIT.md # Chapter-by-chapter tutorial code audit
│   ├── J5_PRE_ASSEMBLY_FROZEN_v1.0.md # Frozen release archive & SHA256 hashes
│   ├── PRE_ASSEMBLY_DOCUMENT_AND_HARDWARE_ACCEPTANCE.md # Hardware acceptance report
│   └── 6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md # Post-assembly 9-phase commissioning plan
│
├── bone_ws/                        # Core ROS 2 Workspace
│   ├── src/
│   │   ├── pus6_msgs/              # Custom ROS 2 interfaces (TimedJointFeedback, etc.)
│   │   ├── pus6_kinematics/        # Analytical IK, LM numerical FK, FixedLag synchronizer
│   │   │   └── config/6pus_geometry.yaml # Single source of truth geometry configuration
│   │   ├── pus6_motor_driver/      # RS485 UART driver, 6-motor controller, safety preflight
│   │   ├── pus6_control/           # Stateful trajectory shaper & joint trajectory controller
│   │   ├── pus6_bringup/           # System launch files and parameter configurations
│   │   └── h30_imu/                # C++ Yesense H30 IMU driver & relative attitude node
│   └── scripts/                    # Verified execution and benchmarking scripts
│
├── hardware/                       # Hardware datasheets, schematics, and mechanical models
├── up/                             # Standalone hardware utilities and test suites
│
├── 6PUS_REAL_HARDWARE_OPERATION_GUIDE.md    # Single authoritative hardware operation manual
└── 6PUS_ROS2_CODE_LEARNING_TUTORIAL.md      # Full 12-chapter technical learning tutorial
```

---

## Development Roadmap

* **J1 -- J4 (Completed & Frozen)**:
  * J1: Single-bus RS485 protocol & CRC verification.
  * J2: 6-Axis synchronous communication & baseline performance.
  * J3: Analytical IK & point-to-point S-curve homing/parking.
  * J4: Continuous Cartesian trajectory shaping, LVW buffer & UART batch-atomicity.
* **J5 (Pre-Assembly Baseline Frozen -- Current Release)**:
  * Timed telemetry (`CLOCK_MONOTONIC`), Fixed-Lag zero-extrapolation synchronizer, numerical FK observer with analytical Jacobian, dual H30 hardware microsecond timestamping, and session-start gyro bias initialization.
* **J6 (Post-Assembly Commissioning -- Next Stage)**:
  * Full 6PUS mechanical assembly, zero mapping verification, micro-jog direction check, physical reachability, J5-C2-HW real mechanism FK acceptance, and final dual-IMU installation extrinsic calibration (J5-B2).
* **Future Milestones**:
  * Dual-IMU / FK Error-State Kalman Filter (ESKF) fusion.
  * Translational acceleration modeling and state estimation.
  * Active disturbance rejection and real-time vibration suppression.
  * Learning-based predictive trajectory control.

---

## Environment & Prerequisites

* **Operating System**: Ubuntu 24.04 LTS (x86_64, Linux kernel 6.8+)
* **Middleware**: ROS 2 Jazzy Jalisco
* **Python Runtime**: Python 3.12.3+
* **Dependencies**: `numpy`, `scipy`, `pyserial`, `pyyaml`, `pytest`, `colcon`

### Build & Test

```bash
# 1. Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# 2. Build workspace
cd /home/lyapunov/bone_drill/bone_ws
colcon build --symlink-install
source /home/lyapunov/bone_drill/bone_ws/install/setup.bash

# 3. Run core test suite (162 tests)
pytest /home/lyapunov/bone_drill/bone_ws/src -v
```

---

## Documentation Links

* [J5 Frozen Release Manifest](agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md)
* [J5 Final Test Acceptance Report](agent_docs/frozen/J5_FINAL_TEST_REPORT.md)
* [Software Environment & Interface Specification](agent_docs/frozen/J5_SOFTWARE_ENVIRONMENT.md)
* [Post-Assembly Commissioning Plan (J6)](agent_docs/stages/J6/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md)
* [Real Hardware Operation Guide](6PUS_REAL_HARDWARE_OPERATION_GUIDE.md)
* [ROS 2 Code Learning & Architecture Tutorial](6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)

---

## License & Safety Notice

This software is developed for precision parallel robotic research. All physical operations on real hardware must strictly adhere to the safety procedures specified in the [Real Hardware Operation Guide](6PUS_REAL_HARDWARE_OPERATION_GUIDE.md).
