# J2-C ROS2 Six-Axis Vector Real-Hardware Plan

## Status and Scope

`J2-C REAL HARDWARE SYNCHRONOUS MOTION TEST [COMPLETE / PASS]`

Authoritative log:
`/home/lyapunov/bone_drill/bone_ws/log/j2c_vector_20260818_161601.json`

The objective is to validate the formal ROS2 driver path for 6-axis coordinated/
synchronous motion verification on the shared UART bus.

The approved next candidate test is:

```text
Delta q = [+10, +10, +10, +10, +10, +10] step
speed = 50 step/s
observation = 5.0 s
```

## PRE-HW Software Implementation

- runner: `/home/lyapunov/bone_drill/bone_ws/scripts/run_j2_c_vector.py`;
- dedicated driver API: `write_validated_config_block()` plus sequential
  `command_positions_with_metrics()` with per-axis completion timestamps;
- Mock gate tests:
  `/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/test/test_j2_c_vector.py`;
- default runner mode is Mock; real mode requires
  `--real-hardware --confirm-motion --validated-config --confirm-vector`.

A real run requires all real-hardware gates. Attempt 1 is permanently retained
as `STOP` in `bone_ws/log/j2c_vector_20260818_141618.json`: it stopped during
read-only static admission on ID2 `0x29` and produced zero write frames. This
baseline-gate revision does not authorize or execute another real run.

## Required References and Fixed Rules

The implementation and any later run must follow:

- `BONE_WS_ROS2_ARCHITECTURE_PLAN.md`;
- `J2-B_ROS2_SINGLE_MOTOR_REAL_HARDWARE_PLAN.md`;
- `up/motor/uart_multi/AGENTS.md`;
- the M6-B/M6-C validated configuration and stop sequence.

The following rules are fixed:

- `hardware_mode=active` is required for a real run;
- static control deadband is `+/-2 step` (`+/-0.05 mm`); raw actual and
  register `0x29` values must remain unmodified in the JSON log;
- actual-position reference is selected independently for each axis from a
  short histogram mode; ties select the smaller mode;
- actual baseline samples must be stable, have no sustained one-direction
  drift, `current=0 mA`, and `fault=0`;
- `0x29` is an independent raw state. It must satisfy
  `abs(raw_0x29-reference) <= 2` during static admission and configuration
  readback; it never selects the reference. A one-step raw transition such as
  `75 -> 74` is recorded and admitted when it remains inside this band. A
  sustained multi-step monotonic excursion (`span > 1`), an out-of-band value,
  or actual-position drift stops before any write; raw target drift remains
  preserved for diagnosis.
- the validated configuration payload for axis `i` is one contiguous block:
  `[2, 0, 0, 50, reference_i]` starting at `0x25`;
- all six candidate targets must be validated before any configuration or
  target write is sent;
- UART dispatch is sequential ID1 -> ID2 -> ... -> ID6. It is not atomic;
- **Safety STOP Conditions (True Hardware Risks)**:
  - Hardware fault: `fault != 0` on any axis (pre-flight, configuration, or runtime);
  - Abnormal overcurrent: `current > 500 mA` (or pre-flight static current != 0);
  - Boundary overrun risk: any target position outside safe physical/software bounds;
  - Communication/Write failure: UART write exception on configuration or target frames;
  - Pause failure: inability to dispatch pause commands or confirm emergency stop;
  - Genuine mechanical runaway / uncommanded drift outside deadband before write.
- **Performance Results (Recorded Metrics, NOT Safety Aborts)**:
  - Small static quantization jitter: normal `+/-2 step` (`+/-0.05 mm`) fluctuations;
  - Small residual positioning errors: final error `|p - target|` recorded in telemetry;
  - Settle time / latency differences / slow start: per-axis start latency and arrival skew;
  - Axis-to-axis tracking differences: recorded in `sync_metrics`.
- when all six axes have reached `abs(error) <= 2` with required stable samples,
  issue `pause_all` once; if observation timeout (5.0s) completes, issue `pause_all`
  and record performance metrics;
- stop confirmation validates actuator stationary state, current=0 mA, and fault=0;
- configuration, target, and pause writes are never retried;
- `pause_all` is best-effort over all six axes. Every axis must receive one
  pause attempt even if another pause fails;
- any STOP/FAIL preserves the original JSON and is not automatically rerun.

## Fixed Candidate Parameters

- Next candidate target vector: `baseline_i + Delta_q[i]` for ID1..ID6;
- Candidate relative increment: `Delta_q = [+10, +10, +10, +10, +10, +10] step`;
- control mode: `2` (Speed Mode);
- target speed: `50 step/s`;
- observation timeout: `5.0 s`;
- pause trigger: all six axes within `2 step` of their targets or observation window complete;
- stop confirmation: actuators stationary, `current=0 mA`, `fault=0`;
- no Homing, IK, zero-offset adjustment, or mechanical-end discovery.

The J2-B2 result is a dynamic risk input, not a new command parameter:

- first positive motion latency: `288.217 ms`;
- first `error <=2`: `1306.487 ms` (about `1.306 s`);
- peak current: `33 mA`;
- final displacement: `+8 step` for the `+10 step` B2 move.

J2-C must not increase speed or change the protocol because of these values.
The six-axis worst-case completion time remains an open measurement.

## Pre-flight and Baseline Admission

No write is allowed during this phase.

1. Open the ROS2 driver in active mode only after all human conditions and
   authorization gates are confirmed.
2. Read a full six-axis snapshot: actual position, raw `0x29`, current, fault,
   limits, temperatures, and timestamps. Require all six currents to be zero
   and all faults to be zero.
3. Collect a short actual-position histogram for each axis. Record raw samples,
   histogram, mode candidates, min/max/span, and drift result.
4. Select `reference_i` from the actual histogram mode, choosing the smaller
   value on a tie. Do not use a single target-register sample to select it.
5. Collect independent raw `0x29` samples for each axis. Record their complete
   histogram and min/max/span. Reject before any write if any raw value is more
   than two steps from that axis reference, if the actual samples show
   sustained one-direction drift, or if current/fault admission fails. Do not
   reject a one-step `0x29` transition solely because it is monotonic.
6. Compute the candidate vector from the live references. Validate all six
   integer targets against global and per-axis hardware limits, and record the
   complete vector before any configuration write.

If any axis or any target fails, the run ends with `STOP` and zero hardware
writes.

## Configuration and Readback

The vector pre-validation above must finish before the first configuration
frame. Then perform the M6/J2-B validated block sequence in axis order:

```text
ID1 0x25 block [2,0,0,50,reference_1]
ID2 0x25 block [2,0,0,50,reference_2]
...
ID6 0x25 block [2,0,0,50,reference_6]
```

Each block is one frame and one attempt. Do not replace it with separate
register writes, `configure_all()` safety-limit payloads, or a retry loop.
Record the dispatch and completion timestamp, motor ID, register address, and
payload for every frame.

After all six blocks have been sent, read back all six axes before any target
write. Require for each axis:

- `0x25=2`;
- `0x28=50`;
- actual position within `reference_i +/-2`;
- raw `0x29` within `reference_i +/-2`;
- current `0 mA`, fault `0`, and no detected drift.

Any configuration write exception or readback failure stops before target
dispatch. If configuration has already written one or more axes, invoke the
best-effort six-axis pause path once, record its complete result, and report
`HARD STOP REQUIRED` if any pause attempt fails.

## Sequential Target Dispatch and Monitoring

Only after all six configuration readbacks pass:

1. Reconfirm the complete target vector and limits without changing it.
2. Dispatch exactly one `0x29=[target_i]` frame per axis in ID1..ID6 order.
3. Record per-axis write start/completion timestamps and raw response status.
4. If any target write raises or returns an unsafe status, do not retry. Invoke
   `pause_all` once across ID1..ID6 and enter STOP handling.
5. Poll all six axes. On any `fault != 0` or `current >500 mA`, immediately
   invoke `pause_all` once and stop. Normal non-zero motion current below the
   threshold is allowed.
6. When every axis has consecutive feedback samples with `abs(error)<=2`,
   issue `pause_all` once. Record each of the six pause frames independently.
7. Confirm two or more stable post-pause samples for all axes. Require current
   zero, fault zero, and final error `<=3` on every axis.

`pause_all` is a best-effort safety action, not an atomic operation. A failed
pause on any axis requires `HARD STOP REQUIRED` and operator hardware
emergency-stop/power removal.

## JSON and Write Audit

The authoritative JSON must independently contain:

- stage/status, authorization flags, fixed parameters, and candidate vector;
- six-axis pre-flight and post-flight snapshots;
- per-axis actual histogram/mode/min/max/span/drift and raw `0x29`
  histogram/mode/min/max/span/drift;
- selected references, static deadband checks, and target validation result;
- every configuration block frame and readback result;
- every target dispatch frame with per-axis start/completion timestamps;
- every pause attempt, result, timestamp, and confirmation sample;
- position-versus-time samples, first positive motion latency, first error
  `<=3`, first error `<=2`, peak current, and final errors;
- UART transaction/attempt/retry/timeout/checksum/protocol/failure telemetry;
- per-axis write counts and an explicit sequential (non-atomic) dispatch audit;
- STOP reason, `HARD STOP REQUIRED` state, and whether all six pause attempts
  were made.

## Mock Test Design

Before any real approval, Mock/FakeBus tests must cover at least:

1. Valid `Delta_q` vector: all six configuration blocks, six target frames,
   one six-axis pause, monotonic per-axis timestamps, and complete audit.
2. Invalid vector length/type/out-of-limit target: zero configuration and
   target writes.
3. Static actual/`0x29` jitter within `+/-2`, including `0x29` `75 -> 74`:
   admission passes with raw samples preserved and references selected from
   actual modes.
4. Any actual drift, current non-zero, fault non-zero, or raw `0x29` deviation
   greater than two: STOP before configuration with zero writes. A sustained
   multi-step monotonic `0x29` excursion also STOPs; a stable actual position
   plus a one-step raw `0x29` change must not false STOP.
5. Configuration readback failure: no target writes; best-effort pause audit
   covers all six axes if configuration writes had begun.
6. Target write failure at each dispatch position: no retry, one pause attempt
   for every axis, correct partial-write audit, and STOP.
7. Runtime current values from normal motion through `500 mA` do not stop;
   `501 mA` and any non-zero fault stop and invoke pause_all.
8. All six axes reaching `abs(error)<=2` produce one pause_all and final
   acceptance only when every final error is `<=3` and current/fault recover.
9. Final error greater than three, timeout, pause failure, or unstable stop
   confirmation produces STOP/HARD STOP as appropriate and never retries.
10. READ_ONLY mode and missing real authorization gates perform no hardware
    access and no writes.

## Risk Review and Real-Hardware Gate

Open risks before a real J2-C request:

- sequential six-axis dispatch is not atomic; a mid-sequence failure can leave
  a partial target vector;
- the shared UART adds dispatch skew and the six axes may not reach error<=2
  within the fixed timeout;
- B2 measured only one axis, so its latency is not a six-axis timing bound;
- static raw `0x29` jitter, current transients, and per-axis mechanical load
  may differ across the six axes;
- best-effort pause can itself partially fail and requires hardware emergency
  stop intervention;
- no Homing, IK, or software zero-offset behavior is part of this stage.

J2-C may be submitted for a separate human real-hardware approval only after
the Mock tests above pass, the JSON/write-audit implementation is reviewed,
and the operator confirms the six-axis safe region and hardware emergency
stop. This document itself does not authorize execution.

## PRE-HW Verification Result

- J2-C Mock tests: `20 passed` after the post-stop revision;
- bone_ws full test result: `105 passed`;
- uart_multi regression: `306 passed`;
- real UART/write/motion: Attempt 1 STOP during baseline admission, `0` writes;
  the approved Retry STOP is recorded in the post-stop audit below; no further
  retry is authorized by this revision.

## POST-STOP AUDIT / PRE-RETRY REVISION (2026-08-18)

The approved same-condition Retry was executed once and is permanently retained
as `STOP`. The original JSON is immutable:

```text
/home/lyapunov/bone_drill/bone_ws/log/j2c_vector_20260818_152653.json
```

The run passed the complete read-only baseline admission, six-axis target
validation, all six configuration writes and readbacks, then stopped because
the fixed `3.0 s` tracking window ended before every axis reached
`abs(error)<=2`.

### STOP audit

- references ID1..ID6: `[66, 74, 30, 71, 23, 16]`;
- commanded targets: `[70, 80, 38, 81, 29, 20]`;
- final positions: `[66, 78, 37, 80, 28, 16]`;
- final displacement from reference: `[0, +4, +7, +9, +5, 0]` step;
- signed final error (`actual-target`): `[-4, -2, -1, -1, -1, -4]` step;
- config and target dispatch order was correct: 18 frames, three per axis,
  with zero retries;
- UART telemetry: `576/576` transactions/attempts, retry/timeout/checksum/
  protocol/failed transaction counts all zero;
- `pause_all` attempted once for every axis and all six attempts succeeded;
- ID1 and ID6 had zero final displacement;
- final STOP reason: `J2-C timeout before all axes reached error<=2`.

The historical JSON remains unchanged. Its `stop_confirmation` is `null`
because the pre-revision timeout path closed after one post-flight snapshot.
The runner revision now performs the formal two-sample stop confirmation after
any successful `pause_all` on PASS, timeout, runtime STOP, and write-failure
paths. A failed or unstable confirmation sets `HARD STOP REQUIRED`.

### Raw position trace audit

The 45 feedback snapshots were inspected without smoothing or rewriting raw
positions. ID1 remained exactly `66` in all 45 samples, so its first raw
positive sample and confirmed motion are both absent. ID6 was `16` in 44
samples and changed to `17` once at about `1721.108 ms`, then returned to `16`.
That single in-band one-step change is static quantization/jitter, not valid
mechanical motion; it is not counted as confirmed motion.

The runner now records two separate timing fields:

- `first_raw_positive_sample_latency_ms`: first feedback sample with
  `position > reference`;
- `first_confirmed_motion_latency_ms`: first feedback sample with
  `position > reference + 2` for two consecutive feedback samples.

The second definition deliberately requires leaving the static `+/-2 step`
band and consecutive confirmation, so a one-step baseline change cannot be
reported as motion.

### Candidate and timeout revision

The candidate test in the runner is:

```text
Delta q = [+10, +10, +10, +10, +10, +10] step
speed = 50 step/s
observation = 5.0 s
```

This homogeneous $+10\text{ step}$ test provides clear excitation for all 6 axes
(well beyond the $\pm 2\text{ step}$ static band) to directly characterize
6-axis start latency skew, arrival skew, and motion synchronization.
It is a candidate only and requires a separate human real-hardware approval.

The $5.0\text{ s}$ observation window allows complete data capture of the
multi-axis transient trajectory. If all axes settle within $\pm 2\text{ step}$
early, the runner executes `pause_all` immediately; if the full $5.0\text{ s}$
window completes, the runner executes `pause_all` and records all performance
metrics. Small residual errors or settling differences are recorded as
Performance Results and do not abort the trial as safety failures.

## 6-AXIS SYNCHRONIZATION REVISION SUMMARY (2026-08-18)

1. **Explicit Safety STOP vs Performance Result Decoupling**:
   - **Safety STOP**: `fault != 0`, `current > 500 mA`, target boundary violation,
     UART write failure, pause dispatch failure, unconfirmed stop.
   - **Performance Result**: small residual error, settling time, start delay,
     axis synchronization skew. Recorded in telemetry, not treated as safety aborts.
2. **Synchronization Metrics**:
   - `start_skew_ms`: difference between slowest and fastest moving axis start latencies;
   - `arrival_skew_ms`: difference between slowest and fastest arrival latencies (error $\le 2$ step);
   - `settled_axes` and `all_axes_settled`: tracking settling status across all 6 axes;
   - `final_positions_steps`, `final_errors_steps`, `final_displacements_steps`.
3. **Stop Confirmation**:
   - Validates that actuators are physically stationary, `current = 0 mA`, and `fault = 0` (health safe).

## J2-C REAL-HARDWARE RUN AUDIT (2026-08-18 16:16:01) — [COMPLETE / PASS]

Authoritative JSON log:
```text
/home/lyapunov/bone_drill/bone_ws/log/j2c_vector_20260818_161601.json
```

### Execution Audit
- Command: `run_j2_c_vector.py --real-hardware --confirm-motion --validated-config --confirm-vector`;
- Hardware mode: `active`;
- Baseline references ID1..ID6: `[66, 78, 37, 80, 28, 16]` step;
- Commanded targets ID1..ID6: `[76, 88, 47, 90, 38, 26]` step ($\Delta q = +10$ step homogeneous);
- Final positions ID1..ID6: `[74, 86, 45, 88, 37, 24]` step;
- Displacements ID1..ID6: `[+8, +8, +8, +8, +9, +8]` step;
- Final errors ID1..ID6: `[-2, -2, -2, -2, -1, -2]` step;
- All 6 axes settled within error $\le 2$ step: `all_axes_settled = True` (reached at $1.444\text{ s}$, confirmed at $1.511\text{ s}$);
- Motion latencies:
  - First raw positive sample: ID6@$35.93\text{ ms}$, ID3@$371.49\text{ ms}$, ID4@$371.49\text{ ms}$, ID5@$438.36\text{ ms}$, ID2@$571.94\text{ ms}$, ID1@$772.61\text{ ms}$;
  - First confirmed motion: ID3@$638.47\text{ ms}$, ID4@$705.26\text{ ms}$, ID5@$772.61\text{ ms}$, ID6@$772.61\text{ ms}$, ID1@$906.59\text{ ms}$, ID2@$1041.03\text{ ms}$;
  - First arrival (error $\le 2$ step): ID5@$1108.42\text{ ms}$, ID4@$1241.40\text{ ms}$, ID2@$1376.18\text{ ms}$, ID3@$1376.18\text{ ms}$, ID1@$1443.74\text{ ms}$, ID6@$1443.74\text{ ms}$;
- Synchronization skews:
  - `start_skew_ms`: $402.57\text{ ms}$ ($\max=1041.03\text{ ms (ID2)} - \min=638.47\text{ ms (ID3)}$);
  - `arrival_skew_ms`: $335.32\text{ ms}$ ($\max=1443.74\text{ ms (ID1/6)} - \min=1108.42\text{ ms (ID5)}$);
- Peak currents: ID1=$62\text{ mA}$, ID2=$53\text{ mA}$, ID3=$33\text{ mA}$, ID4=$32\text{ mA}$, ID5=$42\text{ mA}$, ID6=$41\text{ mA}$ (all $\le 500\text{ mA}$);
- Hardware fault codes: $0\text{x}00$ across all 6 axes throughout the entire run;
- Target dispatch spread: $18.17\text{ ms}$ (ID1 start $1787040962.516\text{ s}$ to ID6 completion $1787040962.535\text{ s}$);
- Writes and telemetry: exactly 18 writes (6 config + 6 target + 6 pause), 456 total UART transactions, 0 retries, 0 timeouts, 0 checksum errors, 0 failed transactions;
- Pause and Stop confirmation: `pause_all` succeeded across all 6 axes; stop confirmation passed with 2 consecutive stable samples, current=0 mA, fault=0;
- Final outcome: `PASS / COMPLETE`.
