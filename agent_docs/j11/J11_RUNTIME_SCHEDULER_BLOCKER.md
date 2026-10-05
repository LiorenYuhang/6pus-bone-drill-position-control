# J11/J12 Runtime Scheduler Blocker & Diagnostics Audit

**Document Status**: Active Engineering Diagnostic Document  
**Date**: 2026-09-23  
**Target Repository**: `/home/lyapunov/bone_drill`  
**Baseline**: Commit `1ad883b2ea4b42861455f2f3eea778940a3b7dff` (Tag: `v1.0-j11-attitude-stabilization-frozen`)  
**Scope**: Instrumentation-Only Diagnostics (Runtime-Fix-1). Zero real hardware motion, zero controller retuning, zero safety threshold alteration.

---

## 1. Executive Summary

During J11 frozen baseline evaluations and recent test runs on 2026-09-22 and 2026-09-23, intermittent runtime liveness anomalies were observed. These manifested either as:
1. `scheduler_deadline_exceeded` (where `J7ServoFeedbackScheduler` detects a feedback ingest gap > 1000 ms, triggering a six-axis `STOP`), or
2. Severe thread execution pauses / descheduling (e.g. 915 ms elapsed with only 6.5 ms thread CPU), triggering a target stream timeout and subsequent target jump rejection on re-arming.

Prior root-cause audits confirmed that these anomalies are **not** controller instability, **not** P/PD tuning defects, **not** motor over-current, and **not** mechanical jams. However, the precise leaf cause could not be established because the cycle in which a fault occurred was **dropped** before diagnostic dumps were flushed to disk.

This document records the offline forensic analysis of the latest user-initiated runs from 2026-09-23 and defines the **Runtime-Fix-1** instrumentation architecture designed to eliminate all diagnostic blind spots without introducing runtime overhead.

---

## 2. Latest User-Run Forensic Audit (2026-09-23)

Following the initial audit, the operator performed two new test executions on 2026-09-23:

### Run 1: `results/j11/m2_pid/20260923_133806`
* **Operator Observation**: System automatically homed and exited cleanly.
* **Run Characteristics**:
  - Total servo cycles recorded: 226
  - Total stall events (>50 ms): 12 (10 > 50 ms, 4 > 100 ms, 2 > 150 ms, 2 > 200 ms)
  - Max missing interval: **262.569 ms** at cycle 142
  - Inter-callback gap: 201.937 ms; Callback execution total: 60.632 ms (UART TX 4.793 ms, drain 55.538 ms)
  - Thread CPU delta: **9.567 ms** vs Process CPU delta: **217.266 ms**
  - Context switches: Voluntary 11,173, Involuntary 63
  - Classification: `A_TIMER_DISPATCH_DELAY_PYTHON_GIL_OR_BUSY`
* **Finding**: The servo thread was starved of CPU/GIL for ~202 ms outside the callback. The driver did not trigger a hard stop because the single-event gap remained < 1000 ms and the runner homed gracefully.

### Run 2: `results/j11/m2_pid/20260923_133818`
* **Operator Observation**: Noticeable motion stutter / stuttering, followed by abnormal termination.
* **Run Characteristics**:
  - Mode: `POWERED REAL EXECUTION` (`j11_demo_feedforward_pd`)
  - Total cycles attempted: 473; Cycles recorded in ring buffer: 429; Hardware writes: 402
  - Final State: **FAULT**
  - Fault Reason:
    ```text
    Driver fatal latch detected via /rosout: Timeout recovery target rejected: 
    max jump 227 steps > limit 80 steps. Target generator must re-anchor before resuming.
    ```
  - Total stall events (>50 ms): **62** (54 > 50 ms, 22 > 100 ms, 14 > 150 ms, 10 > 200 ms)
  - Max missing interval: **935.408 ms** at cycle 405
  - Worst stall cycle 405 breakdown:
    * Total missing interval: 935.408 ms
    * Inter-callback gap: 12.848 ms
    * In-callback execution: **922.560 ms**
    * Slew processing segment: **915.445 ms**
    * Thread CPU delta: **6.455 ms** vs Process CPU delta: **11.201 ms**
    * Voluntary context switches: 85,994, Involuntary: 112
    * Classification: `B_IN_CALLBACK_OTHER_BLOCK`
* **Critical Takeaway**:
  - The 915.445 ms recorded in the slew processing segment is **NOT 915 ms of CPU computation** (thread CPU was only 6.455 ms). Rather, this proves the thread was **descheduled / starved of execution** while in that section of code.
  - This is a **new fault manifestation**. It confirms that severe execution pauses (200–935 ms) are occurring in the environment.
  - **Leaf Root Cause Status**: The exact leaf root cause remains **OPEN** (e.g., Linux CFS scheduler starvation, Python GIL contention with ROS subscriber/timer threads, kernel USB driver delays, or hardware interrupt latency). It will be definitively isolated during the next one-shot diagnostic capture.

---

## 3. The Core Observability Blind Spot

Prior audits identified a fundamental recording blind spot:
1. **Ordering Inversion**: `_latch_j7_fault()` calls `self._stall_recorder.dump()` **before** `self._stall_recorder.finish_cycle()` is called in `_execute_servo_cycle()`.
2. As a consequence, the active cycle that encounters or triggers the fault is left in `_curr` and **never written to `ring_buffer`** before files are serialized to disk.
3. Furthermore, `scheduler_gap_ns` was never passed from `ingest_cycle_feedback()` into `FeedbackCycleAudit`, causing the diagnostic recorder to record `watchdog_scheduler_gap_ms = 0.0`.
4. `timer_dispatch_delay_ms` was computed as cumulative drift from cycle 0 (`t_entry - (t0 + N * 20ms)`), conflating long-term clock drift with single-cycle wake lateness.
5. Context switches were recorded as cumulative `rusage` counters rather than per-cycle deltas.

Because of this blind spot, previous logs could not definitively separate:
- **Hypothesis A**: Servo worker was starved of CPU / GIL prior to entry.
- **Hypothesis B**: Servo worker entered the cycle, but blocked on `serial.write()`, `in_waiting`, or `serial.read()` / RX drain.
- **Hypothesis C**: Compound effect (A + B).

---

## 4. Runtime-Fix-1 Instrumentation Specification

Runtime-Fix-1 implements comprehensive, lightweight, in-memory instrumentation to provide unambiguous observability:

### 4.1 In-Memory Ring Buffer Architecture
* Fixed-size `deque(maxlen=300)` holding lightweight `CycleTimingRecord` instances.
* Zero file I/O or JSON serialization during 50 Hz streaming.
* Dumps exactly once on driver stop, fault latch, or node shutdown.

### 4.2 Wall Time vs. Thread CPU Time
* **Wall time**: Measured via high-resolution monotonic timestamps (`time.perf_counter_ns()`).
* **Thread CPU time**: Measured via `time.thread_time_ns()` strictly for `Pus6ServoThread`.
* **Diagnostic Rule**:
  - `wall_elapsed >> thread_cpu_delta` (e.g. wall = 920 ms, CPU = 6 ms) $\implies$ Thread descheduling, GIL contention, or OS blocking sleep.
  - `wall_elapsed \approx thread_cpu_delta` (e.g. wall = 920 ms, CPU = 900 ms) $\implies$ CPU-bound busy-spin / spinloop inside the thread.

### 4.3 Single-Cycle Wake Lateness
* Replaces cumulative timer drift with:
  $$\text{wake\_lateness\_ns} = \text{actual\_cycle\_entry\_ns} - \text{expected\_cycle\_start\_ns}$$
* In `dedicated_thread` mode, `expected_cycle_start_ns` is anchored to `next_deadline_ns`.
* Evaluated strictly on a per-cycle basis.

### 4.4 Ten Segment Timings ($T_0$ to $T_9$) in `_execute_servo_cycle()`
* $T_0$: Worker cycle entry (`t_entry_ns`)
* $T_1$: Target lock acquired / candidate target resolved
* $T_2$: Vector slew recovery calculation complete
* $T_3$: Safety preflight checks passed
* $T_4$: Transport `send_cycle()` entry
* $T_5$: Transport `send_cycle()` exit
* $T_6$: Feedback scheduler `ingest_cycle_feedback()` entry
* $T_7$: Feedback scheduler ingest complete
* $T_8$: Telemetry snapshot handoff complete
* $T_9$: Cycle exit / completion

### 4.5 Sub-Phase Breakdown in `J7SixAxisServoTransport.send_cycle()`
* Phase A: Prior RX residual drain (wall duration, CPU duration, frames/bytes discarded)
* Phase B: Sequential writes for ID1 through ID6:
  - Per-axis write start/end timestamps, wall duration, CPU duration
  - Bytes requested vs. bytes written
  - Inter-frame pacing wait durations
* Phase C: Motion feedback drain (wall duration, CPU duration, calls to `in_waiting` and `read`, bytes read, max single-read latency)
* Phase D: Total transport wall duration

### 4.6 Preservation of `scheduler_gap_ns`
* `J7ServoFeedbackScheduler.ingest_cycle_feedback()` accurately calculates `scheduler_gap_ns = now_ns - self._last_poll_completion_ns`.
* The true value is exposed on `FeedbackCycleAudit.scheduler_gap_ns` and recorded in `CycleTimingRecord.scheduler_gap_ms`.
* If `scheduler_gap > warning` (283.3 ms) or `scheduler_gap > hard_stop` (1000 ms), a diagnostic snapshot is captured immediately before latching STOP.

### 4.7 Guaranteed Fault-Cycle Capture
* `record_fault_cycle(reason, phase)` is invoked directly inside `_latch_j7_fault()` **before** `dump()`.
* If `dump()` is invoked while `_curr` is active, it finalizes `_curr` into `ring_buffer` prior to serialization.
* The faulting cycle is guaranteed to be present in `cycle_timing.csv` and `fault_snapshot.json`.

### 4.8 Accurate Context Switch Deltas
* Uses `resource.getrusage(resource.RUSAGE_THREAD)` to extract:
  - `vol_ctx_switches_delta`
  - `invol_ctx_switches_delta`
* Captures true per-cycle scheduling preemptions.

### 4.9 Diagnostic Artifacts
On dump, the following files are produced in the run directory:
1. `cycle_timing.csv`: Full ring buffer with $T_0$–$T_9$ segments, CPU times, wake lateness, context switch deltas.
2. `transport_phase_timing.csv`: Sub-phase breakdown of every transport cycle (prior drain, ID1..ID6 writes, feedback drain).
3. `fault_snapshot.json`: Dedicated post-mortem record isolating the faulting cycle, root-cause classification, and transport metrics.
4. `stall_diagnostic_summary.json` & `stall_events_all.csv`: Maintained for full backwards compatibility.

---

## 5. Invariants and Safety Guarantees

Runtime-Fix-1 strictly preserves all production invariants:
* No changes to controller gains ($K_p, K_i, K_d, f_c$).
* No changes to kinematics, IK, or workspace limits.
* No changes to safety preflight limits ($[0, 2000]$ raw, $[100, 1900]$ normal, 20-step inter-frame delta).
* No changes to feedback scheduler thresholds (283.3 ms warning, 1000 ms hard stop, 4-miss escalation).
* No changes to UART communication parameters (baudrate 921600, write timeout None, zero flush).
* Zero changes to motor commands or trajectory generation.

---

## 6. Runtime-Fix-1B Diagnostics Finalization

To ensure complete production safety and preserve the frozen baseline:
1. **Default Disabled**: Parameter `enable_runtime_diagnostics` defaults to `False` in `motor_driver_node.py` and `pus6_system.launch.py`. When disabled, `_stall_recorder` is `None`, allocating zero ring buffer memory and generating zero disk I/O.
2. **Explicit Diagnostic Mode**: Staged launchers (`j11_staged_launcher.py`, `j11_m2_staged_launcher.py`) and top-level shell scripts (`run_j11_demo_feedforward.sh`, `run_j11_demo_feedforward_pd.sh`) provide the `--runtime-diagnostics` CLI flag to explicitly enable diagnostics for targeted capture sessions.

---

## 7. OS Scheduler Policy & One-Shot Hardware Capture Protocol

### 7.1 Strict Prohibition on OS Scheduler Modification in Initial Capture
* **Prohibited Actions**: `chrt`, `nice`, `SCHED_FIFO`, `SCHED_RR`, `taskset` (CPU affinity pinning), or CPU frequency governor alterations are strictly **prohibited** for the initial fault capture.
* **Engineering Rationale**: Stalls occurred under the standard Linux CFS scheduler. Modifying thread priority or scheduling policy prior to capturing uncorrupted diagnostic data would mask or distort the timing dynamics. The root cause must first be captured under the exact operating conditions in which it occurred.

### 7.2 Proposed One-Shot Diagnostic Capture Run (Awaiting User Authorization)
* **Configuration**:
  - Script: `bone_ws/scripts/run_j11_demo_feedforward.sh --runtime-diagnostics`
  - Controller: Frozen Base-IMU SO(3) Feedforward only (simplest reproducible configuration)
  - Disturbance: Base stationary (no manual disturbance until stable active state confirmed)
  - Initial Pose: OPERATING_HOME
  - OS Scheduler: Default Linux CFS (unmodified nice/priority)
  - Safety Gate: Operator must authorize real hardware power-on. Zero hardware commands are issued in this review session.

