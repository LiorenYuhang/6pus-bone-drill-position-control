# J10-A9 Roll +/- Characterization

Date: 2026-09-09

This report separates offline prediction from the real-hardware evidence. The
J10-only attitude gate was extended to +62.5 deg / -69.0 deg. J7/J8/J9
production gates were unchanged.

## Offline projection

| Roll target | Predicted steps (ID1..6) | Min absolute margin | Limiting actuator | IK | Historical envelope | J10 preflight |
|---:|---|---:|---|---|---|---|
| +30 deg | [1502,1000,533,533,1000,1502] | 498 | ID1/ID6 | valid | within | PASS |
| +50 deg | [1809,1000,306,306,1000,1809] | 191 | ID1/ID6 | valid | exceeded | PASS (maintenance required) |
| +60 deg | [1947,1000,236,236,1000,1947] | 53 | ID1/ID6 | valid | exceeded | PASS (maintenance required) |
| +62.5 deg | [1980,1000,223,223,1000,1980] | 20 | ID1/ID6 | valid | exceeded | PASS (maintenance required) |
| +64.5 deg | [2005,1000,215,215,1000,2005] | -5 | ID1/ID6 | valid | exceeded | LIMIT, absolute range |
| -30 deg | [549,1000,1486,1486,1000,549] | 514 | ID3/ID4 | valid | within | PASS |
| -50 deg | [350,1000,1766,1766,1000,350] | 234 | ID3/ID4 | valid | within | PASS |
| -55 deg | [319,1000,1828,1828,1000,319] | 172 | ID3/ID4 | valid | exceeded | PASS (maintenance required) |
| -65 deg | [284,1000,1940,1940,1000,284] | 60 | ID3/ID4 | valid | exceeded | PASS (maintenance required) |
| -69 deg | [281,1000,1980,1980,1000,281] | 20 | ID3/ID4 | valid | exceeded | PASS (maintenance required) |
| -71.5 deg | [283,1000,2004,2004,1000,283] | -4 | ID3/ID4 | valid | exceeded | LIMIT, absolute range |

The predicted absolute-bound brackets are +62.5 to +64.5 deg and -69.0 to
-71.5 deg. The 20-step points are the endpoint-near search stop rule, not a
qualification envelope.

## Hardware results

| Candidate | Predicted steps | Result | Candidate reached | Measurement window | HOME | Timing / traffic |
|---:|---|---|---:|---|---|---|
| Roll +30 deg | [1502,1000,533,533,1000,1502] | PASS | 7.602 s (timeout 9.867 s) | valid, 0 invalid samples | confirmed, 7.189 s; final [1000,1000,999,999,1000,1000] | 353 UART cycles; 2118 0x29 frames |
| Roll +50 deg | [1809,1000,306,306,1000,1809] | FAIL/LIMIT | not reached; 15.000 s (timeout 14.983 s) | not established | not confirmed | 185 cycles; 1110 frames; driver reported `overcurrent_ID4` and latched STOP |
| Roll -30 deg | [549,1000,1486,1486,1000,549] | FAIL/LIMIT | not reached; 9.600 s timeout | not established | not confirmed | 148 cycles; 888 frames; no reported fault code |

For +30 deg, the candidate measurement FK median was
`[-0.0059 mm, 0.1271 mm, 132.6084 mm, 30.0296 deg, 0.0190 deg, -0.0689 deg]`.
Relative to the post-HOME FK median, the coupling was approximately
`dX=-0.0059 mm, dY=-0.0059 mm, dZ=+0.0004 mm, dRoll=+29.9622 deg,
dPitch=+0.0017 deg, dYaw=+0.0004 deg`.

Using the Platform H30 HOME-relative quaternion and the existing canonical
Rz(+90 deg) frame transform, the +30 deg measurement median was approximately
`dRoll=+29.50 deg, dPitch=-0.81 deg, dYaw=-0.17 deg`. This is a small
calibration/frame discrepancy relative to FK, not a J10 limit. No H30/FK
measurement consistency claim is made for the timeout runs because their
measurement windows were never valid.

Artifacts:

- [+30 CSV](../../results/J10_A9/J10_workspace_roll_pos300deg_20260909_163601.csv)
- [+30 metadata](../../results/J10_A9/J10_workspace_roll_pos300deg_20260909_163601_metadata.json)
- [+50 CSV](../../results/J10_A9/J10_workspace_roll_pos500deg_20260909_163919.csv)
- [+50 metadata](../../results/J10_A9/J10_workspace_roll_pos500deg_20260909_163919_metadata.json)
- [-30 CSV](../../results/J10_A9/J10_workspace_roll_neg300deg_20260909_164252.csv)
- [-30 metadata](../../results/J10_A9/J10_workspace_roll_neg300deg_20260909_164252_metadata.json)

## Qualification and boundary

The maximum fully-qualified Roll+ observed in this run is +30 deg. Roll- has
no fully-qualified hardware point in this run. Roll+ progression stopped at
+50 deg after a candidate tracking timeout and an ID4 overcurrent latch; Roll-
progression stopped at -30 deg after candidate tracking timeout. These are
operational execution failures, not evidence of a mechanical or absolute
actuator boundary. The predicted actuator endpoint remains untested and is
deferred under the endpoint-proximity protocol.

Current observed workspace summary:

- X: [-45.75, +45.75] mm
- Y: [-45.5, +49.0] mm
- Roll: no qualified negative point in A9; +30 deg qualified, with larger
  positive and all negative candidates stopped by execution qualification.

No controller tuning, sensor fusion, or J7/J8/J9 production change was made
for this characterization, and no commit was created.
