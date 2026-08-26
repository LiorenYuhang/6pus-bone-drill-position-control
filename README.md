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

**Current Milestone**: $\mathbf{J7\ SIX\text{-}AXIS\ 50Hz\ SERVO\ HARDWARE\ BASELINE\ FROZEN\ v1.0}$  
*(Tag: `v1.0-j7-servo-hardware-frozen`)*

The six-axis 50Hz Servo motion control baseline and hardware verification are fully verified and frozen:
* **Six-Axis 50Hz Continuous Target Stream**: 50.00Hz cycle rate (mean period $20.00\,\text{ms} \pm 0.10\,\text{ms}$), atomic write-only 0x29 batch transmission ($\text{mean } 3.38\,\text{ms}$, $\le 6.83\,\text{ms}$).
* **Non-Blocking Round-Robin Telemetry**: 1-axis 0x30 status query per 20ms cycle (~8.33Hz per-axis sampling rate), mean feedback latency $6.84\,\text{ms}$, zero runtime 0x25 mode readback.
* **PARK $\leftrightarrow$ HOME Full-Stroke Hardware Validation**: 1000 steps ($25.0\,\text{mm}$) quintic S-curve smooth trajectory with 3.0s station holds ($v_{\text{peak}} = 100\,\text{steps/s}$, 2326 cycles, 46.50s total duration).
* **Quantitative A/B Benchmark vs J6 Positioning Baseline**:
  * Target dispatch rate increased from 10Hz to 50Hz (5.0x higher target dispatch rate).
  * Max discrete tracking error reduced from $19\,\text{steps}$ to $11\,\text{steps}$.
  * Overall RMS tracking error reduced from $7.53\,\text{steps}$ to $2.89\,\text{steps}$ (-61.6%).
  * Peak current reduced from $145\,\text{mA}$ to $77\,\text{mA}$ (-46.9%).
  * Average current reduced from $34.2\,\text{mA}$ to $29.9\,\text{mA}$.
  * Current standard deviation / electrical fluctuation reduced by $\sim 48\%$, significantly damping acceleration/deceleration spikes.
  * Velocity step / micro-pause stutter events reduced by $>50\%$, yielding significantly smoother motion verified via camera observation.
  * Final PARK return repeatability error: $\le 1\,\text{step}$ ($0\,\text{step}$ return skew).
* **Safety & Emergency Stop**: 0 faults, 0 timeouts, and verified 0x1A stop ACK across all 6 axes.
* **Core Unit Test Suite**: 304 tests passed (100% PASS in 43.75s).

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
│  • J7 Six-Axis 50Hz Servo Transport (Single-bus write-only batch)     │
│  • J7 Round-Robin Feedback Scheduler (1 axis/cycle 0x30 status query) │
│  • pus6_motor_driver_node: 6-Stage Safety Preflight & Batch Lock      │
│  • RS485 Half-Duplex Single Bus (921600 8N1)                          │
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
│   ├── decisions/                  # Decision logs, timing audits, and stage status
│   ├── frozen/                     # Archived frozen baseline manifests & reports
│   └── stages/                     # Stage-specific commissioning guides
│
├── bone_ws/                        # Core ROS 2 Workspace
│   ├── src/
│   │   ├── pus6_msgs/              # Custom ROS 2 interfaces (TimedJointFeedback, etc.)
│   │   ├── pus6_kinematics/        # Analytical IK, LM numerical FK, FixedLag synchronizer
│   │   │   └── config/6pus_geometry.yaml # Single source of truth geometry configuration
│   │   ├── pus6_motor_driver/      # RS485 UART driver, J7 50Hz Servo transport & schedulers
│   │   ├── pus6_control/           # Stateful trajectory shaper & joint trajectory controller
│   │   ├── pus6_bringup/           # System launch files and parameter configurations
│   │   └── h30_imu/                # C++ Yesense H30 IMU driver & relative attitude node
│   └── scripts/                    # Verified execution and benchmarking scripts
│
├── hardware/                       # Hardware datasheets, schematics, and mechanical models
├── up/                             # Standalone hardware utilities and test suites
│
├── 6PUS_SERVO_OPERATION_GUIDE.md    # Single authoritative hardware operation manual
└── 6PUS_ROS2_CODE_LEARNING_TUTORIAL.md      # Full 12-chapter technical learning tutorial
```

---

## Development Roadmap

* **J1 -- J4 (Completed & Frozen)**:
  * J1: Single-bus RS485 protocol & CRC verification.
  * J2: 6-Axis synchronous communication & baseline performance.
  * J3: Analytical IK & point-to-point S-curve homing/parking.
  * J4: Continuous Cartesian trajectory shaping, LVW buffer & UART batch-atomicity.
* **J5 (Pre-Assembly Baseline Frozen)**:
  * Timed telemetry (`CLOCK_MONOTONIC`), Fixed-Lag zero-extrapolation synchronizer, numerical FK observer with analytical Jacobian, dual H30 hardware microsecond timestamping, and session-start gyro bias initialization.
* **J6 (Post-Assembly Commissioning Baseline Frozen)**:
  * Six-axis independent actuator HOME (`[1000]*6`) / PARK (`[0]*6`) commissioning in Positioning Mode (mode=0, 10Hz, speed=50), verified >10 consecutive cycles.
* **J7 (Six-Axis 50Hz Servo Hardware Baseline Frozen -- Current Release)**:
  * High-frequency 50Hz 6-axis Servo streaming (mode=1), non-blocking round-robin telemetry (~8.33Hz/axis), full-stroke PARK $\leftrightarrow$ HOME quintic S-curve verification, and quantitative A/B benchmark demonstrating 61.6% RMS error reduction and 46.9% peak current reduction over J6.
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

# 3. Run core test suite (304 tests)
pytest /home/lyapunov/bone_drill/bone_ws/src -v
```

---

## Documentation Links

* [J5 Frozen Release Manifest](agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md)
* [J5 Final Test Acceptance Report](agent_docs/frozen/J5_FINAL_TEST_REPORT.md)
* [Software Environment & Interface Specification](agent_docs/frozen/J5_SOFTWARE_ENVIRONMENT.md)
* [Post-Assembly Commissioning Plan (J6)](agent_docs/stages/J6/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md)
* [Real Hardware Operation Guide](6PUS_SERVO_OPERATION_GUIDE.md)
* [ROS 2 Code Learning & Architecture Tutorial](6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)

---

## License & Safety Notice

This software is developed for precision parallel robotic research. All physical operations on real hardware must strictly adhere to the safety procedures specified in the [Real Hardware Operation Guide](6PUS_SERVO_OPERATION_GUIDE.md).
