# J2-B2 ROS2 Single-Motor Position-Tracking Plan

## Status and Scope

> **文档定位说明**：本文保留 J2-B2 的原始设计、Mock 验证和授权门禁材料；后续执行结果和当前阶段状态以 `agent_docs/README.md`、J2-B 实机计划及最终审计记录为准。不要把本计划中的“未授权/未执行”句子当作当前全局状态。

This document defines J2-B2 only. It is a design and Mock-validation artifact;
no J2-B2 real-hardware run has been authorized or executed.

- target motor: `ID4`
- live baseline: read during pre-flight
- motion delta: `+10 step`
- target: `live baseline + 10`
- control mode: validated block value `0x25=2`
- target speed: `0x28=50 step/s`
- motion timeout: `3.0 s` (the J2-B1 verified timeout)
- pause trigger: absolute target error `<=2 step`, after two consecutive samples
- final acceptance: absolute target error `<=3 step`

J2-B1 remains the separate fixed `ID4/+3 step/3.0 s` hardware-write-chain
profile. This B2 profile must not weaken or widen the B1 gate.

## B2 Baseline Admission

Before any configuration write, B2 takes ten short read-only ID4 samples of
actual position and the 0x29 target, with current and fault checked on every
sample. Actual and target are analyzed independently with histogram, mode
candidates, min, max, span, and M6-style sustained one-direction/disjoint-half
drift checks.

Admission requires actual span <=1 step, target span <=1 step, and the union
of both static bands to span <=1 step. This is the explicit actual/0x29
consistency rule; no 2-step tolerance is admitted. The two bands and all
samples are persisted in the JSON.

The configuration reference is always the actual-position histogram mode. If
there is a mode tie within the admitted one-step actual band, the deterministic
lowest mode candidate is used; no instantaneous 0x29 sample can replace it.
The validated block writes [2, 0, 0, 50, reference], and the motion target is
reference + 10.

## Evidence from J2-B1

Authoritative log:

`bone_ws/log/j2b_motor4_20260818_123655.json`

J2-B1 target write was `ID4 0x29=66` from baseline `63`.

- First feedback arrived about `19.7 ms` after the target write, while actual
  position was still `63` and error was `-3`.
- Therefore the `<=3` criterion was already true at the baseline and cannot
  measure positive motion for a `+3` command.
- The first observed positive position was `64`, error `-2`, about `2.729 s`
  after the target write. The pause followed after the second consecutive
  `<=2` sample.
- Feedback samples were full six-axis sequential scans with the configured
  poll period; the observed delay combines command/drive response, UART scan
  scheduling, polling, and position feedback quantization.
- This log does not isolate a physical mechanism for the delay. No mechanism
  is inferred from it.

The `+10` target makes the initial error `-10`, so `<=3` and `<=2` become
meaningful tracking milestones rather than being satisfied at the baseline.

## Software Authorization Gates

The B2 runner is:

`bone_ws/scripts/run_j2_b2_tracking.py`

Mock is the default and always exercises the validated configuration block.
Real hardware requires all four explicit gates:

```bash
python3 scripts/run_j2_b2_tracking.py \
  --real-hardware \
  --confirm-motion \
  --validated-config \
  --confirm-tracking
```

The runner rejects real execution before constructing a controller if any gate
is missing. It fixes ID4, `+10`, `3.0 s`, and the `50 step/s` setup; there are no
parameters that can widen the motion for a retry.

## Authoritative Run Sequence

1. Six-axis read-only pre-flight; all currents must be `0mA` and all faults
   `0x00`.
2. ID4-only read of 0x25, 0x28, 0x29, actual position, current, and fault, then
   take the ten-sample static baseline window.
3. Reject before writes if either span exceeds one step, the actual/0x29 union span exceeds one step, any drift is detected, any current/fault sample is unsafe, or the current 0x29 target violates the band/reference policy.

4. Send exactly one ID4 0x25 block using the actual histogram reference:

   `[2, 0, 0, 50, selected_reference]`

5. Read back mode, speed, 0x29 inside its static target band, actual inside the actual static band, current, and fault.

   A readback failure sends no target.
6. Send exactly one ID4 0x29 = selected_reference + 10 target.
7. Monitor all six axes. ID1/2/3/5/6 are read-only observers.
8. On two consecutive ID4 samples with error `<=2`, send one ID4 `0x1A=[1]`
   pause. Runtime current is allowed to be nonzero; `current >500mA` or any
   nonzero fault is STOP.
9. Require stable stop, target-axis current `0mA`, fault `0x00`, and final error
   `<=3` step. Save the six-axis post-flight snapshot.

The only allowed successful write sequence is:

```text
ID4: 0x25 block -> ID4: 0x29 target -> ID4: 0x1A pause
```

ID1/2/3/5/6 write counts must remain zero. Every write uses the existing
single-attempt transport path and is independently captured by `UartBus`
write audit.

## Tracking Measurements

The B2 JSON records:

- `first_positive_motion_latency_ms`: first feedback position greater than the
  live baseline;
- `position_vs_time_samples`: elapsed time, actual position, displacement,
  target error, current, and fault for every six-axis feedback sample;
- `time_to_error_lte_3_ms` and `time_to_error_lte_2_ms`;
- `peak_current_mA`;
- `final_displacement_steps` and `final_target_error_steps`;
- complete UART telemetry and per-frame/per-axis write audit.

## B2 PASS / STOP Criteria

PASS requires all of the following:

- first positive motion and both error milestone timestamps are recorded;
- positive final displacement with final target error `<=3 step`;
- no runtime fault and no current above `500mA`;
- pause and two-sample stop confirmation pass, with current returning to zero;
- exact three-frame ID4 write sequence and zero observer writes;
- no UART retry, timeout, checksum, protocol, or failure counters.

Any failed gate, timeout, fault, overcurrent, pause failure, failed stop
confirmation, or audit mismatch is STOP. Do not software-retry or rerun; use
hardware emergency stop/power removal if the stop confirmation is not proven.

## Prohibitions

Do not execute J2-B2 without a new explicit human hardware approval. Do not
execute J2-C, change the `+10` target, change the timeout or speed, configure
other axes, or use B2 to authorize any other motion stage.
