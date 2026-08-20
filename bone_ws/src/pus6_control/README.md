# pus6_control: Control System Architecture for 6PUS Parallel Manipulator

## Overview
`pus6_control` hosts the control algorithms for the 6PUS parallel manipulator platform across three evolutionary stages:

1. **Stage 1 (Current / Baseline)**: Joint Trajectory Setpoint Dispatcher (`JointTrajectoryController`).
   - Bounds velocity and step deltas.
   - Enforces 6-axis coordinated dispatch.

2. **Stage 2 (Planned / J3+)**: Classical and Active Disturbance Rejection Control (ADRC).
   - Linear / Extended State Observers (LESO / ESO) for total disturbance estimation.
   - Decoupled parallel axis disturbance rejection using H30 IMU + actuator current feedback.

3. **Stage 3 (Future)**: Reinforcement Learning (Soft Actor-Critic / SAC).
   - Residual policy on top of nominal kinematic/ADRC controller.
   - Multi-modal state input: Platform pose error + IMU angular velocity/acceleration + cylinder forces/currents.

## Architecture Guidelines
- Controllers implement `ControllerInterface` (`update`, `reset`).
- Controllers must operate on 6-element vectors for ID1..ID6.
- Controllers do not execute raw UART writes; all commands pass through `pus6_motor_driver`.
