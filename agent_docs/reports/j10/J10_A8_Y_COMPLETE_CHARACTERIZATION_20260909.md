# J10-A8 Y+/- Complete Operational Workspace Characterization

## Offline projection

`Analytical6PusKinematics` was scanned from HOME toward both Y directions. The J10-only gate was extended to `Y+ <= 49.0 mm` and `Y- >= -45.5 mm`; default J7/J8/J9 behavior remains unchanged. Historical `[200,1800]` is metadata only.

| direction / target | predicted steps `[1..6]` | abs margin | limiter | IK | endpoint status |
|---|---|---:|:---:|:---:|---|
| Y+ 30.0 | `[1003,1391,1296,1296,1391,1003]` | 609 | ID2/5 | PASS | safe |
| Y+ 40.0 | `[1102,1652,1514,1514,1652,1102]` | 348 | ID2/5 | PASS | safe |
| Y+ 45.0 | `[1172,1817,1652,1652,1817,1172]` | 183 | ID2/5 | PASS | historical exceeded |
| Y+ 48.0 | `[1221,1929,1746,1746,1929,1221]` | 71 | ID2/5 | PASS | endpoint-near |
| Y+ 49.0 | `[1238,1969,1779,1779,1969,1238]` | 31 | ID2/5 | PASS | last executed |
| Y+ 49.5 | `[1247,1989,1796,1796,1989,1247]` | 11 | ID2/5 | PASS | not executed: margin `<20` |
| Y+ 49.75 | `[1252,2000,1805,1805,2000,1252]` | 0 | ID2/5 | PASS | endpoint command avoided |
| Y+ 50.0 | `[1256,2010,1814,1814,2010,1256]` | -10 | ID2/5 | PASS | first invalid |
| Y- 30.0 | `[1466,1068,1157,1157,1068,1466]` | 534 | ID1/6 | PASS | safe |
| Y- 40.0 | `[1762,1192,1316,1316,1192,1762]` | 238 | ID1/6 | PASS | safe |
| Y- 44.0 | `[1910,1258,1397,1397,1258,1910]` | 90 | ID1/6 | PASS | historical exceeded |
| Y- 45.5 | `[1970,1285,1431,1431,1285,1970]` | 30 | ID1/6 | PASS | last executed |
| Y- 45.75 | `[1981,1290,1437,1437,1290,1981]` | 19 | ID1/6 | PASS | not executed: margin `<20` |
| Y- 46.0 | `[1991,1295,1442,1442,1295,1991]` | 9 | ID1/6 | PASS | not executed |
| Y- 46.25 | `[2002,1300,1448,1448,1300,2002]` | -2 | ID1/6 | PASS | first invalid |

Predicted absolute-bound brackets are `Y+ 49.75-50.0 mm` and `Y- 46.0-46.25 mm`. The `<20` rule is only an automatic search stop, not a `[20,1980]` qualification envelope.

## Hardware results

Every executed point had feedback-reached candidate, a valid measurement window with zero invalid samples, zero fault/timeout, and feedback-confirmed HOME. Maintenance mode was used only after historical envelope exceedance.

| target | predicted | final candidate feedback | abs margin / limiter | FK median `(X,Y,Z,Roll,Pitch,Yaw)` | candidate / HOME (s) | UART / 0x29 | result |
|---:|---|---|---:|---|---:|---:|:---:|
| Y+30.0 | `[1003,1391,1296,1296,1391,1003]` | `[1002,1390,1295,1295,1390,1001]` | 609 / ID2 | `(-0.0527,29.9596,132.5983,-0.0145,+0.0497,+0.0595)` | 5.853 / 6.118 | 289 / 1734 | PASS |
| Y+45.0 | `[1172,1817,1652,1652,1817,1172]` | `[1170,1816,1651,1652,1816,1170]` | 183 / ID2 | `(-0.0048,45.0381,132.5624,-0.0127,-0.0338,-0.0508)` | 10.600 / 11.969 | 459 / 2754 | PASS |
| Y+48.0 | `[1221,1929,1746,1746,1929,1221]` | `[1219,1928,1744,1745,1927,1220]` | 71 / ID2 | `(-0.0068,47.9848,132.5911,+0.0012,-0.0635,-0.0876)` | 13.764 / 14.054 | 503 / 3018 | PASS |
| Y+49.0 | `[1238,1969,1779,1779,1969,1238]` | `[1237,1968,1779,1779,1968,1236]` | 31 / ID2 | `(-0.0426,48.9985,132.5900,-0.0481,+0.0598,+0.0421)` | 12.346 / 13.889 | 523 / 3138 | PASS |
| Y-30.0 | `[1466,1068,1157,1157,1068,1466]` | `[1465,1066,1156,1156,1067,1465]` | 534 / ID1 | `(+0.0527,-30.0496,132.5619,-0.0202,-0.0312,+0.0599)` | 7.510 / 7.505 | 302 / 1812 | PASS |
| Y-44.0 | `[1910,1258,1397,1397,1258,1910]` | `[1909,1258,1395,1395,1257,1909]` | 90 / ID1 | `(-0.0165,-43.9767,132.5919,+0.0595,+0.0498,-0.0484)` | 12.883 / 14.581 | 495 / 2970 | PASS |
| Y-45.5 | `[1970,1285,1431,1431,1285,1970]` | `[1969,1283,1429,1430,1283,1968]` | 30 / ID1 | `(-0.0791,-45.5346,132.5474,-0.0280,+0.0165,-0.0079)` | 12.680 / 13.686 | 515 / 3090 | PASS |

The known J7 context-invalid teardown warning appeared after completed runs; final artifacts still report `motion_stop=false` and `fault_stop=false`.

Artifacts: [`results/J10_A8`](/home/lyapunov/bone_drill/results/J10_A8).

## Summary

- Maximum fully-qualified observed Y+: **+49.0 mm**.
- Maximum fully-qualified observed Y-: **-45.5 mm**.
- Y+ limiting actuators: ID2/ID5; margin trend 609 -> 183 -> 71 -> 31.
- Y- limiting actuators: ID1/ID6; margin trend 534 -> 90 -> 30.
- No mechanical, direction, tracking, HOME, or hardware fault limitation was observed.
- Current XY observed range: X `[-45.75,+45.75] mm`; Y `[-45.5,+49.0] mm`.
- Boundary classification: **absolute actuator bound censored; true operational boundary not reached**.

No recommended safe limit is assigned pending completion of all six DOF.
