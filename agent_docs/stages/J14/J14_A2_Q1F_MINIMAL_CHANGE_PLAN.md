# J14-A2-Q1F minimum change plan

Handoff independently checked from old Q1 raw data before source edits: R1 5997/5997, one deadline phase reset (3.062 nominal-slot displacement); R2 14990/14990, three resets (10.792 slots displacement, conventionally approximately11 skipped slots). Nominal-minus-executed for R2 is10, a different statistic. Both runs have contiguous Child/Bridge sequences and no TX timeout or hardware fault. Four R2 queue-after events occur at ID5 cycles1829/5210/8591/11972 (3381-cycle spacing); queue-after22.069/51.644/86.430/115.454ms while actual write0.027/0.032/0.032/0.045ms. Their amplitude has a monotonic increasing trend, strongly supported by these four points; strict linear growth and a particular kernel-lock mechanism are not established.

History: A1.1 introduced optional queue-before/after observations to diagnose TX backpressure. Original adapter defaultsFalse; original driver wires enable_runtime_diagnostics. A1.9 experimental Child hardcodedTrue. P1 formalized architecture/evidence profiles but retained that literal, so production and qualification still queried queues per frame.

Only production edit: UART Child adapter factory selects diagnostics ON for explicit debug and historical nonproduction experiments; OFF for formal production/qualification. No new parameter is needed: driver_telemetry_mode already exists. Legacy wiring, adapter implementation and _read_tx_queue_state remain unchanged. Necessary RX availability reads remain unchanged.

OFF removes queue-before/after and the extra per-write process-CPU/context-switch/clock-sync snapshot bound to that option. Serial.write wall/thread-CPU/timestamps/bytes/error/configured timeout, wrapper timing, cycle timing/CPU, RX observer, fault/pause and bounded evidence remain. Qualification queue distributions are expected absent, never synthesized zeros.

Validate with protocol-aware FakeSerial on the actual factory, all relevant four-package regressions, build/syntax/diff/AST checks, default three-process and legacy Mock tests. Hardware authorization is limited to F-R1 30s, then F-R2 60s only if R1passes; no long-duration run, LADRC, trajectory or disturbance. Capture and analysis are new stage scripts; historical P1/Q1 scripts and evidence remain unchanged.

Before F-R2, require full nominal cycle count and sequence, zero timeout/fault/missing/duplicate, zero deadline resets/slot displacement, six-axis pause and resource cleanup. An obvious new hot-path stall (wrapper>=20ms or cycle>=40ms) fails this offline admission; no driver safety gate or scheduler change is introduced. All report distributions use inclusive linear interpolation (P50/P95/P99/P99.9/Max). Keep control/kinematics shutdown exception as an independent known limitation.

If both short runs pass, stop and report. A new source version must begin a new120->300->600s qualification chain; old Q1 remains pre-fix qualification and instrumentation-effect evidence. No commit/push/tag.
