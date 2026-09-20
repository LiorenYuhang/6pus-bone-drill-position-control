# Implementation Plan - J11-RT2B: Servo Telemetry Publication Decoupling

## Goal
Eliminate the in-callback ROS publish blocking identified in J11-RT2 (where `self.joint_state_pub.publish(js_msg)` blocked for 324.03 ms due to DDS endpoint discovery / subscriber matching) by decoupling all ROS telemetry publication from the 50Hz Servo critical path into an asynchronous, latest-only Telemetry Publisher Worker (`async_latest`), while preserving 100% frozen baseline compatibility (`inline`).

---

## User Review Required

> [!IMPORTANT]
> **Strict Invariants**:
> - **Single Variable**: Only telemetry publication is decoupled from the servo thread.
> - **Zero threshold relaxation**: 283.3 ms watchdog, 500 mA overcurrent, debounce, 80-step reconnect jump, 5 mm/s max velocity remain strictly unchanged.
> - **Zero UART / hardware changes**: `serial.flush() / tcdrain` behavior is NOT altered (deferred to RT3).
> - **Zero motion / controller changes**: IK, full-SO(3) feedforward, and safety preflight logic are completely untouched.
> - **Zero Safety Gaps**: All safety checks (watchdog, overcurrent, motor faults, STOP latch) execute synchronously inside the Servo Thread. STOP latch immediately ceases motion writes regardless of telemetry worker state.
> - **Zero Queue Backlog Risk**: Telemetry handoff uses a latest-only snapshot protected by a fast lock (`threading.Lock` + `threading.Event`). No blocking queues, no backlog accumulation, no burst replay.
> - **Preserved Default Baseline**: `telemetry_publish_mode` defaults to `"inline"` in `ros_timer` mode, preserving 100% frozen J7/J8 baseline.

---

## Architecture Comparison

### RT2 Architecture (Synchronous In-Loop Publish - Failure Point)
```
Dedicated Servo Worker Thread (50Hz)
    ├── Target Lock (< 0.1ms)
    ├── Preflight & Slew (< 0.1ms)
    ├── UART TX / RX / Ingest (6~9ms)
    ├── Watchdog Check (< 0.1ms)
    └── SYNCHRONOUS ROS 2 PUBLISH:
            ├── joint_state_pub.publish(js_msg)  <-- BLOCKED 324ms IN DDS MATCHING!
            ├── timed_feedback_pub.publish(...)
            ├── timed_command_pub.publish(...)
            └── raw diagnostics publishes
                    ^
                    └── STALLED SERVO CYCLE DURATION TO 332.7ms -> WATCHDOG TRIPPED!
```

### RT2B Architecture (Decoupled Telemetry Worker)
```
Dedicated Servo Worker Thread (50Hz Critical Path)
    ├── Target Lock (< 0.1ms)
    ├── Preflight & Slew (< 0.1ms)
    ├── UART TX / RX / Ingest (6~9ms)
    ├── Watchdog Check (< 0.1ms)
    └── Update Latest Telemetry Snapshot (< 0.02ms, latest-only overwrite, non-blocking)
            ^
            └── SERVO CYCLE TOTAL DURATION GUARANTEED < 15ms (100% IMMUNE TO DDS STALLS!)

Asynchronous Telemetry Publisher Worker (Pus6TelemetryThread)
    └── Event-driven (wakes on new snapshot or 20ms timeout)
            ├── Fetch Latest Snapshot (drops stale frames if publisher was delayed)
            ├── joint_state_pub.publish(js_msg)  <-- Even if this blocks 350ms in DDS,
            ├── timed_feedback_pub.publish(...)       Servo Thread keeps pacing at 50Hz!
            └── raw diagnostics publishes
```

---

## Proposed Changes

### Component 1: `pus6_motor_driver`

#### [MODIFY] [`motor_driver_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py)
1. **Telemetry Snapshot Container**:
   - Define `ServoTelemetrySnapshot` dataclass containing measurement timestamps, motor steps, currents, fault codes, dispatched targets, and sequence ID.
2. **Parameters & Mode Selection**:
   - Add parameter `telemetry_publish_mode`: `"inline"` (default for `ros_timer`) vs `"async_latest"` (default for `dedicated_thread`).
3. **Telemetry Worker Thread**:
   - `_telemetry_thread: threading.Thread | None = None`
   - `_telemetry_stop_event = threading.Event()`
   - `_telemetry_ready_event = threading.Event()`
   - `_telemetry_lock = threading.Lock()`
   - `_latest_telemetry_snapshot: ServoTelemetrySnapshot | None = None`
   - Implement `_telemetry_publisher_loop(self) -> None`:
     - Waits on `_telemetry_ready_event` with 20ms timeout.
     - Grabs latest snapshot under lock.
     - Calls `_publish_telemetry_from_snapshot(self, snap)`.
4. **Refactor Servo Cycle**:
   - In `_execute_servo_cycle()`:
     - After feedback ingest and safety checks, construct `ServoTelemetrySnapshot`.
     - If `telemetry_publish_mode == "async_latest"`:
       - Update `_latest_telemetry_snapshot` under lock and set `_telemetry_ready_event` (takes < 0.02ms).
     - If `telemetry_publish_mode == "inline"`:
       - Directly call `_publish_telemetry_from_snapshot(snap)` synchronously (preserving legacy behavior).
5. **Thread Lifecycle & Shutdown**:
   - In `destroy_node()`:
     - Signal `_telemetry_stop_event.set()` and join `_telemetry_thread` with timeout.

#### [MODIFY] [`stall_diagnostic_recorder.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/stall_diagnostic_recorder.py)
1. Add telemetry snapshot update timing tapped inside servo cycle (`snapshot_handoff_ms`).
2. Add asynchronous telemetry publisher timing taps (`async_pub_total_ms`, `async_pub_joint_state_ms`, etc.) to prove that servo cycle gap stays ~20ms even when publish duration spikes to 300+ ms.

---

### Component 2: `pus6_bringup` Launch System

#### [MODIFY] [`pus6_system.launch.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/launch/pus6_system.launch.py)
1. Add launch argument `telemetry_publish_mode` (default `"inline"`).
2. Pass `telemetry_publish_mode` to `pus6_motor_driver_node`.

---

### Component 3: `h30_imu` J11 Staged Startup Orchestrator

#### [MODIFY] [`j11_staged_launcher.py`](file:///home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_staged_launcher.py)
1. Add `--telemetry-publish-mode` argument (default `"async_latest"` for J11 staged sessions).
2. Pass `telemetry_publish_mode:={args.telemetry_publish_mode}` into the `pus6_system.launch.py` invocation in Stage A.

---

### Component 4: Offline Test Suite

#### [NEW] [`test_j11_rt2b_telemetry_decoupling.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/test/test_j11_rt2b_telemetry_decoupling.py)
1. Test default mode is `"inline"` and preserves frozen baseline.
2. Test `"async_latest"` mode starts `_telemetry_thread` and runs without error.
3. **Core Isolation Test**: Mock `joint_state_pub.publish()` with a simulated 350 ms sleep:
   - Verify that the Servo Thread continues pacing at 50Hz ($\Delta t_{\text{servo}} \approx 20\text{ ms}$).
   - Verify that Servo Thread's watchdog gap does NOT exceed 283.3 ms!
4. Test latest-only overwrite: when publisher is blocked, intermediate snapshots are dropped and only the latest snapshot is published when it resumes (zero backlog replay).
5. Test STOP latch immediately ceases motion writes in Servo Thread regardless of telemetry thread status.
6. Test clean thread shutdown and join.

---

## Verification Plan

### Phase 1: Automated Offline Tests
1. `python3 -m py_compile` on all modified files.
2. Full pytest suite:
   ```bash
   pytest bone_ws/src/pus6_motor_driver/test/test_j7_c6_feedback_scheduler.py \
          bone_ws/src/pus6_motor_driver/test/test_safety.py \
          bone_ws/src/pus6_motor_driver/test/test_motor_driver_node.py \
          bone_ws/src/pus6_motor_driver/test/test_j11_rt2_servo_scheduler_isolation.py \
          bone_ws/src/pus6_motor_driver/test/test_j11_rt2b_telemetry_decoupling.py
   ```
3. Colcon build.

### Phase 2: ONE-SHOT Physical Hardware Static Home Validation
- **Target Command**:
  ```bash
  ./bone_ws/scripts/j11/j11_video_start.sh --duration 300.0 --servo-scheduler-mode dedicated_thread --telemetry-publish-mode async_latest
  ```
- **Execution Rules**:
  - Base completely static (user does NOT move Base).
  - No disturbance, no video recording.
  - Duration: up to 300s (or early stop on fault).
  - Strictly ONE run.

### Phase 3: Quantitative Comparison
Compare RT1 vs RT2 vs RT2B:
- Max Servo dispatch/cycle gap (RT1: 541.5ms, RT2: 140.2ms -> Target RT2B: < 50ms)
- >200ms Servo cycle gap (RT1: 2, RT2: 0 -> Target RT2B: 0)
- >150ms Servo cycle gap (RT1: 7, RT2: 0 -> Target RT2B: 0)
- >100ms Servo cycle gap (RT1: 38, RT2: 2 -> Target RT2B: 0)
- Scheduler STOP (RT1: YES, RT2: YES -> Target RT2B: 0)
- Worst ROS publish duration vs Corresponding Servo gap.
