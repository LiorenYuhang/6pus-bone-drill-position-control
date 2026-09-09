# J10-A9.3 Roll- Operational Boundary Refinement

Date: 2026-09-09

Only the requested safe-side candidates were attempted. The J10-only Roll
negative gate remained -69 deg; no J7/J8/J9 production or motion-stack code
was changed.

## Results

| Candidate | Predicted steps | Absolute margin / limiter | Candidate | Measurement / FK Roll | HOME | Qualification |
|---:|---|---:|---|---|---|---|
| Roll -35 deg | [489,1000,1560,1560,1000,489] | 440 / ID3, ID4 | 8.141 s reached | valid, 0 invalid; -34.823 deg | 8.006 s confirmed | PASS |
| Roll -37.5 deg | [462,1000,1596,1596,1000,462] | 404 / ID3, ID4 | timeout at 11.433 s | not established | not confirmed | FAIL/STOP |

The -35 deg run used 449 UART cycles and 2694 0x29 frames. Its measurement FK
median was `[-0.0598 mm, 0.2206 mm, 132.6089 mm, -34.8233 deg, 0.0105 deg,
-0.0788 deg]`. The corresponding H30 Roll, calculated from the stable
post-HOME relative zero and existing frame transform, was approximately
-32.66 deg, a -2.17 deg difference from FK. Direction was consistent; the
discrepancy is recorded as calibration/frame uncertainty and did not by itself
fail the candidate.

The -37.5 deg run used 161 UART cycles and 966 0x29 frames. At the driver
fatal event (approximately 18:15:45.688), feedback was about
`[482,1001,1574,1571,1001,487]` against target
`[462,1000,1596,1596,1000,462]`; ID3/ID4 remained roughly 22/25 steps short.
The driver reported `overcurrent_ID4` and latched STOP. No measurement window
or HOME confirmation was obtained. Current magnitude is not present in the
CSV schema; the overcurrent evidence is the driver fatal log. No direct
mechanical interference evidence was recorded.

Artifacts are in `results/J10_A9_3/`:

- `J10_workspace_roll_neg350deg_20260909_181430_{metadata.json,csv}`
- `J10_workspace_roll_neg375deg_20260909_181533_{metadata.json,csv}`

## Boundary closure

- Roll- last fully-qualified PASS: **-35 deg**.
- Roll- first observed FAIL: **-40 deg** from A9.2; it remains the first
  operational failure in the direction and was not rerun.
- Refined Roll- operational bracket: **-35 to -40 deg**.

Current Roll summary:

- Roll+ last fully-qualified PASS: **+37.5 deg**.
- Roll+ first observed FAIL: **+40 deg**, `overcurrent_ID4`.
- Roll- last fully-qualified PASS: **-35 deg**.
- Roll- first observed FAIL: **-40 deg**, `overcurrent_ID4`.

The observed Roll limits are operational current/load failures before the
predicted actuator endpoint. They are not actuator stroke limits and are not
direct proof of a mechanical limit. No further binary refinement was done.

No controller, trajectory, motor safety, or J7/J8/J9 production changes were
made. No commit was created.
