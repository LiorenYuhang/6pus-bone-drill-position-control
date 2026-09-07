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

**Current Milestone**: $\mathbf{J9\ IMU\ Integration\ \&\ Measurement\ Qualification\ COMPLETE}$  
*(Frozen runtime baseline: `v1.0-j8-active-servo-hardware-qualified`; closure docs: [`J9_FINAL_SUMMARY.md`](./J9_FINAL_SUMMARY.md), [`J9_TO_J10_HANDOFF.md`](./J9_TO_J10_HANDOFF.md))*

The continuous Cartesian 50Hz Servo motion control and Design-B accompanying feedback pipeline remain the frozen runtime baseline on real hardware:
* **J8 Status**: `J8 functional hardware qualification: PASS`
* **Design-B Accompanying Feedback**: ACTIVE mode utilizes 50Hz 0x29 write-response frames (`StatusFeedback`) as primary runtime feedback, excluding routine 0x30 queries from the 50Hz critical control path.
* **Two-Tier Watchdog**:
  * Tier 1: Per-executed-cycle 6/6 feedback completeness monitoring (trips fault if $N=4$ consecutive cycles miss feedback on any axis).
  * Tier 2: 263.324 ms scheduler deadline fail-safe ceiling to protect against OS stalls while tolerating benign Python/executor jitter.
* **Centralized PARK / HOME Contract**:
  * Mechanical bottom resting index `STORAGE_PARK = 0 step` ($25.0\text{ mm}$), operating home `OPERATING_HOME = 1000 step` ($50.0\text{ mm}$).
  * Feedback zero-drift provisional measurement band $[-5, +5]\text{ steps}$ ($[-0.125, +0.125]\text{ mm}$).
  * Explicit `VALID_PARK` state classification seeds trajectory planning at `[0]*6` (command domain strictly $\ge 0$).
* **Full-System Real-Hardware Qualification**:
  * 3 consecutive autonomous PARK $\leftrightarrow$ HOME validation rounds passed (56,754 / 56,754 responses, 0 faults/timeouts/stops).
  * 9-Phase Cartesian Video Demonstration completed: 137.128s, 6898 cycles @ 50.30 Hz sustained rate, 41,388 / 41,388 feedback responses (100.0% completeness), 0 faults, 0 STOPs.
* **Full Workspace Regression**: 349 / 349 tests passed (100% PASS across all 5 packages).
* **J9 Closure**: IMU integration and measurement qualification are complete; J9 final summary and J9->J10 handoff are the canonical closure entry points.

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
│  • J7/J8 Six-Axis 50Hz Servo Transport (Design-B 0x29 batch write)    │
│  • Two-Tier Safety Watchdog (Tier 1: 6/6 completeness, Tier 2: DL)    │
│  • Centralized Startup Classifier (safety.py)                         │
│  • pus6_motor_driver_node: 6-Stage Safety Preflight & Batch Lock      │
│  • RS485 Half-Duplex Single Bus (921600 8N1)                          │
└───────────────────────────────────────────────────────────────────────┘
             │                                              │
             ▼ (0x29 StatusFeedback)                        ▼ (Hardware)
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
│   │   ├── pus6_motor_driver/      # RS485 UART driver, J7/J8 50Hz Servo transport & safety
│   │   ├── pus6_control/           # Stateful trajectory shaper & joint trajectory controller
│   │   ├── pus6_bringup/           # System launch files and parameter configurations
│   │   └── h30_imu/                # C++ Yesense H30 IMU driver & relative attitude node
│   └── scripts/                    # Verified execution and benchmarking scripts
│
├── hardware/                       # Hardware datasheets, schematics, and mechanical models
└── 6PUS_SERVO_OPERATION_GUIDE.md   # Daily real-hardware operation guide
```
