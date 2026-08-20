# J2-B ROS2 Single-Motor Real-Hardware Plan

## Scope

This document is the execution reference for one approved J2-B real-hardware
attempt at a time. It covers one target axis only:

- J2-B1 write-chain validation: `ID4`, `+3 step`, `3.0 s` timeout;
- J2-B2 position-tracking validation: `ID4`, `+10 step`, `50 step/s`,
  `3.0 s` timeout.

This plan does not authorize execution by itself. A separate human approval is
required immediately before the run. Do not execute J2-C, J2-F0, or any other
motion stage in the same run.

## Required Human Conditions

- The operator is beside the mechanism.
- The mechanism is in the declared safe region.
- Hardware emergency stop and power removal are immediately available.

## Software Authorization Gate

The real runner must be started with all three explicit gates:

```bash
python3 scripts/run_j2_b_bringup.py \
  --real-hardware \
  --confirm-motion \
  --validated-config \
  --motor-id 4 \
  --delta-steps 3 \
  --timeout-sec 3.0
```

The runner rejects real execution before constructing a controller when any
gate is missing. Real execution parameters are fixed to ID4, `+3`, and
`3.0 s`; they are not widened or retried automatically.

`--mock-configure-target` is a Mock-only test option and is forbidden with
`--real-hardware`.

The separate B2 tracking runner has an additional stage gate:

```bash
python3 scripts/run_j2_b2_tracking.py \
  --real-hardware \
  --confirm-motion \
  --validated-config \
  --confirm-tracking \
  --motor-id 4 \
  --delta-steps 10 \
  --timeout-sec 3.0
```

## Static Position Rule

`STATIC_POSITION_TOLERANCE = +/-2 step` (`+/-0.05 mm`, total deadband
`0.10 mm`) is a formal control-layer deadband. It does not modify raw
register/status data and must not be described as evidence that M6 validated
`+/-2 step` noise.

The static baseline/reference procedure is:

1. Collect a short read-only actual-position histogram.
2. Select `reference` as the actual histogram mode; if modes tie, select the
   smaller value.
3. Require actual feedback to be stable, with no sustained one-direction
   drift, `current=0 mA`, and `fault=0`.
4. Treat register `0x29` as an independent raw state. Do not require it to
   equal actual position and do not use an instantaneous `0x29` sample to
   select `reference`. Require `abs(0x29 - reference) <= 2`; otherwise STOP
   before any write.

After the validated configuration block, read back and require:

- `0x25 = 2`;
- `0x28 = 50`;
- actual position within `reference +/- 2`;
- `0x29` within `reference +/- 2`;
- `current = 0 mA`, `fault = 0`, and no detected drift.

## Authoritative Configuration Sequence

The setup sequence is the sequence that passed M6-B/M6-C real hardware tests.
It must not be replaced by separate writes to individual configuration
registers.

1. Perform the six-axis read-only pre-flight snapshot and health checks.
2. Read ID4 `0x25`, `0x28`, `0x29`, actual position, current, and fault.
3. If the static baseline/reference admission fails, current is non-zero, or
   fault is non-zero, stop with zero writes. Do not configure or repair the
   device.
4. With the three real-hardware authorization gates present, send exactly one
   `CMD_WR_REGISTER (0x32)` frame to ID4 starting at `0x25` with five
   contiguous values, even when the pre-flight configuration already matches.
   This makes the normal approved path auditable as exactly three ID4 writes:

   ```text
   0x25 = 2
   0x26 = 0
   0x27 = 0
   0x28 = 50
   0x29 = selected reference
   ```

5. Read back `0x25=2`, `0x28=50`, and verify both actual position and `0x29`
   are within `reference +/- 2`, with current `0 mA`, fault `0x00`, and no
   drift. Any readback failure stops before a target write.
6. Send exactly one ID4 `0x29` target write. For J2-B1 the target is
   `reference + 3`; for J2-B2 it is `reference + 10`. The target write is
   never retried.
7. Monitor all six axes. The five observer axes receive reads only.
8. Trigger ID4 pause (`0x1A=[1]`) once the absolute target error is `<=2`
   steps. A timeout, fault, current anomaly, or monitoring safety violation
   stops the software flow.
9. Confirm ID4 stopped using repeated feedback samples. Require final error
   `<=3` steps, current `0 mA`, fault `0x00`, and stable position.
10. Save the post-flight six-axis snapshot, UART telemetry, and write audit.

The normal successful write sequence is exactly:

```text
ID4: 0x25 block -> ID4: 0x29 target -> ID4: 0x1A pause
```

All writes to ID1, ID2, ID3, ID5, and ID6 must remain zero.

## J2-B2 Position-Tracking Profile

J2-B2 is a separate, single-run approval after the J2-B1 write-chain
validation. Its fixed parameters are:

- target axis: `ID4` only;
- motion: `+10 step` at `50 step/s`;
- timeout: `3.0 s`;
- target: `reference + 10`;
- pause trigger: absolute target error `<=2 step`;
- final acceptance: absolute target error `<=3 step`;
- runtime safety: `fault != 0` or `current >500 mA` causes STOP;
- observer axes ID1/2/3/5/6 receive reads only and have zero writes;
- every configuration, target, and pause write is one attempt with no retry.

The B2 baseline uses the static position rule above. It records separate raw
actual and `0x29` histograms, modes, min/max/span, deviations from reference,
and drift results. The actual histogram mode alone selects reference. The
`0x25` block is exactly `[2, 0, 0, 50, reference]`, followed by one `0x29`
target write and one ID4 `0x1A` pause write.

## Stage Result

`J2-B COMPLETE / PASS`

Authoritative log:

`/home/lyapunov/bone_drill/bone_ws/log/j2b2_tracking_motor4_20260818_134547.json`

The approved J2-B2 run used actual histogram `{63: 10}`, reference `63`, and
raw `0x29` histogram `{63: 7, 64: 3}`. The target was `73`; final actual was
`71` with final error `-2 step`. First positive motion was `288.217 ms`, first
error `<=3` was `1102.107 ms`, and first error `<=2` was `1306.487 ms`.
Peak current was `33 mA`; pause and stop confirmation succeeded with current
returning to `0 mA` and fault `0`. The only writes were ID4
`0x25 block -> 0x29 target -> 0x1A pause`; ID1/2/3/5/6 write counts were zero.
UART telemetry was `223` transactions with zero retries, timeouts, checksum
errors, protocol errors, and failed transactions.

## Failure Handling

- Configuration write exception: one attempt only; no automatic retry and no
  target write.
- Configuration readback failure: stop before target write; do not retry the
  configuration frame.
- Target write exception: do not retry; follow the existing pause/stop safety
  path if a target command was accepted.
- Pause failure or failed stop confirmation: report `HARD STOP REQUIRED`, keep
  the UART lifecycle open for the prescribed safety handling, and instruct the
  operator to use hardware emergency stop or power removal.
- Any failed run keeps its original JSON log and must not be rerun
  automatically.

## Audit Requirements

The JSON log must include:

- six-axis pre-flight and post-flight snapshots;
- ID4 configuration values and readback result;
- live baseline, target, feedback samples, arrival and final error;
- pause and stop-confirmation records;
- transaction/attempt/retry/timeout/checksum/protocol/failure telemetry;
- complete write sequence and per-axis write counts.

Before a real approval, Mock tests must continue to verify the configuration
success path, readback-failure zero-target path, single-attempt configuration
failure, `<=2` pause trigger, `<=3` final acceptance, observer zero writes, and
zero controller/UART access when any real authorization gate is absent.

## Prohibitions

Do not configure other axes. Do not issue a standalone `0x25` or `0x28` write
for the validated path. Do not run a second J2-B attempt, J2-C, or any
multi-axis motion without a new explicit approval.
