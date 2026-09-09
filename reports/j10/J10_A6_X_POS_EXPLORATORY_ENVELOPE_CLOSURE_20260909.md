# J10-A6 X+ Exploratory-Envelope Closure

## Offline refinement

| X+ (mm) | predicted steps | ID3 | min margin to [100,1900] | historical [200,1800] exceeded | IK | predicted status |
|---:|---|---:|---:|---|---|---|
| 40.5 | `[1349,1687,1772,1114,1184,1502]` | 1772 | 128 | no | PASS | safe |
| 41.0 | `[1359,1703,1790,1121,1191,1514]` | 1790 | 110 | no | PASS | safe |
| 41.5 | `[1369,1719,1808,1127,1199,1527]` | 1808 | 92 | yes | PASS | safe |
| 42.0 | `[1380,1735,1826,1134,1207,1540]` | 1826 | 74 | yes | PASS | safe |
| 42.5 | `[1390,1752,1844,1141,1215,1554]` | 1844 | 56 | yes | PASS | safe |
| 43.0 | `[1401,1769,1863,1148,1223,1567]` | 1863 | 37 | yes | PASS | safe |
| 43.5 | `[1412,1786,1882,1156,1232,1581]` | 1882 | 18 | yes | PASS | largest predicted-safe |
| 44.0 | `[1423,1804,1901,1163,1240,1595]` | 1901 | -1 | yes | PASS | **predicted LIMIT, not executed** |
| 44.5 | `[1435,1821,1921,1171,1249,1609]` | 1921 | -21 | yes | PASS | predicted LIMIT, not executed |
| 45.0 | `[1446,1840,1941,1178,1258,1623]` | 1941 | -41 | yes | PASS | predicted LIMIT, not executed |

The J10 experiment gate was minimally extended from +40.0 mm to +43.5 mm, the largest offline-safe point. Default J7/J8/J9 behavior remains unchanged. The +44.0 mm point was excluded because ID3 would be 1901, outside the exploratory envelope.

## Hardware closure

Only the necessary candidate X+43.5 mm was executed after the previously qualified X+40 mm point. Its preflight was PASS with maintenance mode required because the historical envelope is exceeded; maintenance mode was enabled for this one J10 extended-envelope run. No point with predicted actuator steps above 1900 was executed.

Candidate final command was `[1412,1786,1882,1156,1232,1581]`; final feedback was `[1411,1784,1880,1155,1230,1580]`. Candidate reached in 14.017 s against a derived 16.2 s timeout. The candidate measurement window contained 142 samples with zero invalid tracking samples. FK coupling relative to pre-home median was ΔX `+43.489 mm`, ΔY `−0.057 mm`, ΔZ `−0.038 mm`, ΔRoll `−0.077°`, ΔPitch `−0.012°`, ΔYaw `−0.063°`. HOME was feedback-confirmed in 12.695 s with final feedback `[1001,1000,999,1000,1000,1001]` and maximum error 1 step. UART count was 468 cycles with 2808 six-axis `0x29` frames. Fault codes remained `[0,0,0,0,0,0]` and no timeout or mechanical anomaly occurred.

The known J7 worker context-invalid shutdown warning appeared after completed data collection and is recorded as a teardown warning; it did not invalidate candidate tracking or HOME confirmation.

## Closure result

The maximum fully-qualified observed X+ is **+43.5 mm**. The first unexecuted upper bracket is **+44.0 mm**, which is predicted LIMIT because ID3=1901 exceeds [100,1900]. Therefore:

**X+ true operational boundary not reached; current X+ result is exploratory-envelope censored.**

This is an exploratory-policy/actuator-envelope stop, not a demonstrated mechanical limit. No X+44.0 mm, X+44.5 mm, X+45.0 mm, or any larger point was tested. No production files were changed and no commit was created.

## Artifacts

- Offline refinement and closure report: `/home/lyapunov/bone_drill/reports/j10/J10_A6_X_POS_EXPLORATORY_ENVELOPE_CLOSURE_20260909.md`
- X+43.5 CSV: `/home/lyapunov/bone_drill/results/J10_A6/J10_workspace_x_pos435mm_20260909_131149.csv`
- X+43.5 metadata: `/home/lyapunov/bone_drill/results/J10_A6/J10_workspace_x_pos435mm_20260909_131149_metadata.json`
- X+43.5 runtime log: `/home/lyapunov/bone_drill/results/J10_A6/J10_x43p5_runtime_20260909_1311.log`
