# J10-A9.2 Roll Operational Boundary Closure

Date: 2026-09-09

The J10-only Roll gates remained +62.5 deg / -69.0 deg. No J7/J8/J9
production, controller, trajectory, or motor-safety changes were made.

## Executed candidates

| Candidate | Predicted steps | Absolute margin / limiter | Candidate | Measurement / FK Roll | HOME | Result |
|---:|---|---:|---|---|---|---|
| Roll +35 deg | [1582,1000,467,467,1000,1582] | 418 / ID1, ID6 | 8.710 s | valid, 0 invalid; +34.889 deg | 8.700 s confirmed | PASS |
| Roll +37.5 deg | [1621,1000,437,437,1000,1621] | 379 / ID1, ID6 | 9.613 s | valid, 0 invalid; +37.312 deg | 8.958 s confirmed | PASS |
| Roll -40 deg | [436,1000,1632,1632,1000,436] | 368 / ID3, ID4 | timeout at 12.033 s | not established | not confirmed | FAIL/STOP |

The +35 deg run used 447 UART cycles / 2682 0x29 frames. The +37.5 deg run
used 377 / 2262. The -40 deg run used 164 / 984. Each run confirmed
pre-candidate HOME; the failed -40 deg run then stopped during candidate
transition. No further candidate was attempted after the failure.

For the valid windows, FK coupling was:

- +35 deg: `dX=-0.0598 mm, dY=-0.2267 mm, dZ=-0.0146 mm,
  dRoll=+34.82 deg, dPitch=+0.0092 deg, dYaw=+0.1371 deg` relative to its
  post-HOME FK median.
- +37.5 deg: `dX=-0.0270 mm, dY=-0.2224 mm, dZ=-0.0185 mm,
  dRoll=+37.24 deg, dPitch=-0.0040 deg, dYaw=+0.1426 deg`.

Platform H30 Roll, using the stable post-HOME relative zero and existing
canonical frame transform, was approximately +34.11 deg (+35 deg) and +35.95
deg (+37.5 deg). The FK/H30 differences were approximately +0.78 deg and
+1.36 deg respectively. Direction remained consistent; the small calibration
discrepancy was not treated as a limit.

The -40 deg fatal event occurred at approximately 17:39:07.885. The command
was `[436,1000,1632,1632,1000,436]`; feedback was approximately
`[488,1001,1575,1566,1003,490]`, leaving ID4 about 66 steps from target. The
driver reported `overcurrent_ID4` and latched STOP. The candidate did not reach
feedback target, so no FK/H30 measurement window was valid. CSV artifacts do
not contain current telemetry; the overcurrent evidence is the driver fatal
log. No direct mechanical interference evidence was recorded.

Artifacts are in `results/J10_A9_2/`:

- `J10_workspace_roll_pos350deg_20260909_173701_{metadata.json,csv}`
- `J10_workspace_roll_pos375deg_20260909_173802_{metadata.json,csv}`
- `J10_workspace_roll_neg400deg_20260909_173855_{metadata.json,csv}`

## Boundary closure

- Roll+ last fully-qualified PASS: **+37.5 deg**.
- Roll+ first observed FAIL: **+40 deg** from A9.1, `overcurrent_ID4`.
- Roll+ operational bracket: **+37.5 to +40 deg**.
- Roll- last fully-qualified PASS: **-30 deg** from A9.1.
- Roll- first observed FAIL: **-40 deg**, `overcurrent_ID4` (the later -50 deg
  `overcurrent_ID3` remains a prohibited rerun).
- Roll- operational bracket: **-30 to -40 deg**.

The limiting behavior is a repeatable pre-endpoint operational current/load
failure, with ID4 involved at +40 and -40 and ID3 at the earlier -50 test. It
is not an actuator stroke limit and is not sufficient evidence of a mechanical
limit. Candidate timeouts without a fault were not observed in this closure;
the timeout runs were accompanied by overcurrent STOP events.

No controller tuning, sensor fusion, or production change was performed. The
J10 experiment-layer pre-HOME confirmation remains the only runner correction.
`py_compile` and the J10 regression suite passed (`26 passed`). No commit was
created.
