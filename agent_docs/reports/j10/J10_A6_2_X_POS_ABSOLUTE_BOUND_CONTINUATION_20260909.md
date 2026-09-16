# J10-A6.2 X+ Absolute-Bound Continuation

## Offline endpoint projection

Analytical6PusKinematics scan used the J10 extended X+ gate. The authoritative actuator range was `[0, 2000]`; historical `[200, 1800]` was metadata only.

| X+ (mm) | predicted steps `[1..6]` | ID3 | absolute margin min | historical exceeded | IK | disposition |
|---:|---|---:|---:|:---:|:---:|---|
| 43.50 | `[1412,1786,1882,1156,1232,1581]` | 1882 | 118 | yes | PASS | prior PASS |
| 43.75 | `[1418,1795,1892,1159,1236,1588]` | 1892 | 108 | yes | PASS | offline |
| 44.00 | `[1423,1804,1901,1163,1240,1595]` | 1901 | 99 | yes | PASS | offline |
| 44.25 | `[1429,1813,1911,1167,1244,1602]` | 1911 | 89 | yes | PASS | offline |
| 44.50 | `[1435,1821,1921,1171,1249,1609]` | 1921 | 79 | yes | PASS | hardware PASS |
| 44.75 | `[1441,1830,1931,1174,1253,1616]` | 1931 | 69 | yes | PASS | offline |
| 45.00 | `[1446,1840,1941,1178,1258,1623]` | 1941 | 59 | yes | PASS | hardware PASS |
| 45.25 | `[1452,1849,1951,1182,1262,1630]` | 1951 | 49 | yes | PASS | offline |
| 45.50 | `[1458,1858,1961,1186,1267,1638]` | 1961 | 39 | yes | PASS | hardware PASS |
| 45.75 | `[1464,1867,1971,1190,1271,1645]` | 1971 | 29 | yes | PASS | hardware PASS |
| 46.00 | `[1470,1877,1982,1194,1276,1652]` | 1982 | 18 | yes | PASS | not executed: margin `<20` |
| 46.25 | `[1476,1886,1992,1198,1280,1660]` | 1992 | 8 | yes | PASS | not executed |
| 46.50 | `[1482,1896,2003,1202,1285,1667]` | 2003 | -3 | yes | PASS | absolute-range invalid |

Largest predicted-safe point is 46.25 mm by `[0,2000]` arithmetic, but automatic hardware progression stopped at 45.75 mm because the minimum margin fell below 20 steps at 46.0 mm. The first absolute-range-invalid point is 46.5 mm.

## Hardware candidates

All hardware runs used the latest A4 runner, `--j10-extended-task-space-gate`, and explicit `--maintenance-mode` because the historical envelope was exceeded. No point used an absolute endpoint command.

| X+ | predicted steps | final candidate feedback | abs margin / limiter | FK measurement median (X, Y, Z, Roll, Pitch, Yaw) | candidate reached / timeout (s) | HOME return (s) | UART / 0x29 | qualification |
|---:|---|---|---:|---|---:|---:|---:|:---:|
| 43.50 | `[1412,1786,1882,1156,1232,1581]` | `[1411,1784,1880,1155,1230,1580]` | 118 / ID3 | `(43.4891, -0.0994, 132.5824, -0.0635, -0.0125, -0.0633)` | 14.017 / 16.200 | 12.695 | 468 / 2808 | PASS |
| 44.50 | `[1435,1821,1921,1171,1249,1609]` | `[1433,1820,1919,1170,1247,1608]` | 79 / ID3 | `(44.4969, -0.1020, 132.5854, -0.0602, -0.0126, -0.0339)` | 12.501 / 16.850 | 13.471 | 519 / 3114 | PASS |
| 45.00 | `[1446,1840,1941,1178,1258,1623]` | `[1445,1838,1940,1176,1256,1622]` | 59 / ID3 | `(45.0919, +0.0101, 132.5118, +0.0185, -0.0416, +0.0065)` | 14.606 / 17.183 | 15.062 | 496 / 2976 | PASS |
| 45.50 | `[1458,1858,1961,1186,1267,1638]` | `[1456,1856,1960,1185,1265,1636]` | 39 / ID3 | `(45.5580, -0.0410, 132.5387, -0.0583, -0.0355, -0.0562)` | 12.883 / 17.517 | 14.062 | 539 / 3234 | PASS |
| 45.75 | `[1464,1867,1971,1190,1271,1645]` | `[1463,1865,1970,1189,1270,1643]` | 29 / ID3 | `(45.7421, -0.0160, 132.5752, +0.0503, -0.0023, +0.0650)` | 14.485 / 17.683 | 12.824 | 529 / 3174 | PASS |

Every executed point had a valid candidate measurement window, zero invalid tracking samples, zero fault/timeout, and feedback-confirmed HOME. HOME final feedback was within the 995-1005 step band at every point. The known J7 context-invalid teardown warning appeared after completed runs; it did not change qualification and `motion_stop=false`, `fault_stop=false` in each final artifact.

Artifacts are under [`results/J10_A6_2`](/home/lyapunov/bone_drill/results/J10_A6_2). The 43.5 mm prior artifact is under [`results/J10_A6`](/home/lyapunov/bone_drill/results/J10_A6).

## Result

- Maximum fully-qualified observed X+: **+45.75 mm**.
- Limiting actuator throughout: **ID3**. Absolute margin decreased 118 -> 79 -> 59 -> 39 -> 29 steps from 43.5 to 45.75 mm.
- FK task achievement tracked the requested X motion; Y/Z and attitude coupling remained small relative to the X displacement.
- No real operational or mechanical limitation was encountered.
- First unexecuted adaptive point: **+46.00 mm** (predicted minimum margin 18, below the automatic progression threshold).
- First absolute-range-invalid point: **+46.50 mm** (predicted ID3=2003).

**Boundary classification:** X+ true operational boundary not reached; current X+ result is exploratory/absolute-envelope censored.
