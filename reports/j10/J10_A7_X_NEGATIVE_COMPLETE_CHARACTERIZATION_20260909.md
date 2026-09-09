# J10-A7 X- Complete Operational Workspace Characterization

## Offline projection

`Analytical6PusKinematics` was evaluated from HOME toward negative X. J10's explicit X gate was extended symmetrically to +/-45.75 mm; default J7/J8/J9 gates remain unchanged.

| X (mm) | predicted steps `[1..6]` | abs margin | limiter | IK | historical exceeded |
|---:|---|---:|:---:|:---:|:---:|
| -15.00 | `[1080,974,949,1164,1138,1030]` | 836 | ID4 | PASS | no |
| -30.00 | `[1279,1057,1007,1461,1404,1173]` | 539 | ID4 | PASS | no |
| -40.00 | `[1489,1176,1108,1755,1671,1339]` | 245 | ID4 | PASS | no |
| -44.00 | `[1595,1240,1163,1901,1804,1423]` | 99 | ID4 | PASS | yes |
| -44.50 | `[1609,1249,1171,1921,1821,1435]` | 79 | ID4 | PASS | yes |
| -45.00 | `[1623,1258,1178,1941,1840,1446]` | 59 | ID4 | PASS | yes |
| -45.50 | `[1638,1267,1186,1961,1858,1458]` | 39 | ID4 | PASS | yes |
| -45.75 | `[1645,1271,1190,1971,1867,1464]` | 29 | ID4 | PASS | yes |
| -46.00 | `[1652,1276,1194,1982,1877,1470]` | 18 | ID4 | PASS | yes; not auto-executed |
| -46.25 | `[1660,1280,1198,1992,1886,1476]` | 8 | ID4 | PASS | yes; not executed |
| -46.50 | `[1667,1285,1202,2003,1896,1482]` | -3 | ID4 | PASS | yes; absolute invalid |

The predicted absolute-bound bracket is `[-46.25, -46.50]` mm. Automatic hardware progression stopped at -45.75 mm because the next point had minimum margin 18 steps (<20). The first predicted absolute-range-invalid point is -46.50 mm.

## Hardware results

All points used the latest J10 runner with `--j10-extended-task-space-gate`. Maintenance mode was enabled only after historical `[200,1800]` was exceeded. Each run had zero invalid measurement samples, zero fault/timeout, feedback-reached candidate, and feedback-confirmed HOME.

| X (mm) | predicted | final candidate feedback | abs margin / limiter | FK median `(X,Y,Z,Roll,Pitch,Yaw)` | candidate / HOME (s) | UART / 0x29 | result |
|---:|---|---|---:|---|---:|---:|:---:|
| -30.00 | `[1279,1057,1007,1461,1404,1173]` | `[1278,1055,1007,1459,1403,1171]` | 539 / ID4 | `(-29.9386,+0.0392,132.6073,-0.0872,-0.0049,+0.1832)` | 7.572 / 7.447 | 329 / 1974 | PASS |
| -40.00 | `[1489,1176,1108,1755,1671,1339]` | `[1487,1175,1107,1754,1670,1338]` | 245 / ID4 | `(-39.9228,-0.0001,132.6244,-0.0097,-0.0479,-0.0173)` | 11.137 / 11.163 | 423 / 2538 | PASS |
| -44.50 | `[1609,1249,1171,1921,1821,1435]` | `[1609,1247,1170,1919,1819,1433]` | 79 / ID4 | `(-44.5410,-0.1221,132.5520,-0.0971,+0.0528,+0.1004)` | 14.069 / 13.121 | 488 / 2928 | PASS |
| -45.50 | `[1638,1267,1186,1961,1858,1458]` | `[1637,1265,1184,1960,1857,1456]` | 39 / ID4 | `(-45.5900,+0.0239,132.5146,-0.0140,+0.0354,+0.0569)` | 14.087 / 13.317 | 525 / 3150 | PASS |
| -45.75 | `[1645,1271,1190,1971,1867,1464]` | `[1643,1269,1188,1970,1866,1463]` | 29 / ID4 | `(-45.7395,+0.0315,132.5815,+0.0712,-0.0089,-0.0623)` | 12.237 / 14.070 | 510 / 3060 | PASS |

The known J7 context-invalid teardown warning appeared after completed runs; it did not affect qualification. Final artifacts report `motion_stop=false` and `fault_stop=false`.

Artifacts: [`results/J10_A7`](/home/lyapunov/bone_drill/results/J10_A7).

## Summary

- Maximum fully-qualified observed X-: **-45.75 mm**.
- Limiting actuator: **ID4**, with absolute margin trend 539 -> 245 -> 79 -> 39 -> 29 steps.
- No mechanical, direction, tracking, HOME, or hardware fault limitation was observed.
- Current X summary: maximum observed X+ **+45.75 mm**; maximum observed X- **-45.75 mm**.
- Boundary classification: **absolute actuator bound censored; true operational boundary not reached**.

No recommended safe limit is assigned; that remains deferred until all six DOF are characterized.
