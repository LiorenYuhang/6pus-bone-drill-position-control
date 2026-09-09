# J10-A5 X+ Operational Boundary Closure

| X+ (mm) | predicted steps | actuator margin (ID1..ID6) | candidate reached | candidate timeout / reached | measurement | HOME duration | HOME |
|---:|---|---|---|---:|---|---:|---|
| 25 | `[1112,1300,1345,975,1017,1198]` | `[788,600,555,875,883,702]` | yes | 7.25 / 6.232 s | valid, 164 samples | 5.772 s | confirmed |
| 30 | `[1173,1404,1461,1007,1057,1279]` | `[727,496,439,893,843,621]` | yes | 9.183 / 5.451 s | valid, 117 samples | 6.762 s | confirmed |
| 35 | `[1248,1527,1597,1051,1110,1375]` | `[652,373,303,849,790,525]` | yes | 11.45 / 7.658 s | valid, 181 samples | 8.257 s | confirmed |
| 40 | `[1339,1671,1755,1108,1176,1489]` | `[561,229,145,792,724,411]` | yes | 14.083 / 10.107 s | valid, 132 samples | 10.253 s | confirmed |

All four points used the A4 candidate gate: five consecutive new feedback samples within the tracking band before measurement, followed by full-window command/feedback tracking validation. All runs used `j10_extended_task_space_gate`; maintenance mode remained disabled and was not required.

## Per-point results

X+35 mm: candidate final command `[1248,1527,1597,1051,1110,1375]`, final candidate feedback `[1247,1526,1596,1050,1109,1373]`; FK coupling relative to pre-home median was ΔX `+34.874 mm`, ΔY `−0.036 mm`, ΔZ `−0.023 mm`, ΔRoll `−0.025°`, ΔPitch `+0.048°`, ΔYaw `+0.154°`. HOME final feedback was `[996,1000,1000,1000,1001,1001]`; UART `372` cycles and `2232` six-axis `0x29` frames.

X+40 mm: candidate final command `[1339,1671,1755,1108,1176,1489]`, final candidate feedback `[1337,1670,1755,1106,1174,1488]`; FK coupling relative to pre-home median was ΔX `+39.919 mm`, ΔY `−0.092 mm`, ΔZ `−0.055 mm`, ΔRoll `−0.005°`, ΔPitch `+0.051°`, ΔYaw `+0.189°`. HOME final feedback was `[1000,1000,999,1000,1001,1001]`; UART `429` cycles and `2574` six-axis `0x29` frames.

Fault codes were `[0,0,0,0,0,0]` throughout both runs. No candidate timeout, HOME timeout, direction anomaly, or mechanical abnormality was observed. Both runs emitted the known J7 worker context-invalid shutdown warning after data collection; this is recorded as a teardown warning and did not invalidate candidate tracking or HOME confirmation.

## Trends and boundary classification

ID3 is limiting throughout. The minimum exploratory margin decreases monotonically: 555 steps at +25, 439 at +30, 303 at +35, and 145 at +40. FK task achievement tracks the commanded X displacement (`+24.84`, `+29.81`, `+34.87`, `+39.92 mm`) while cross-DOF coupling remains small (largest reported term is yaw `+0.189°`). HOME return duration increases with travel: `5.772`, `6.762`, `8.257`, and `10.253 s`.

The maximum fully-qualified observed X+ is **+40 mm**. This does not establish the physical mechanism boundary. The result is censored by the J10 +40 mm task-space gate and exploratory policy; no point beyond +40 mm was tested.

## Artifacts

- X+35 CSV: `/home/lyapunov/bone_drill/results/J10_A5/J10_workspace_x_pos350mm_20260909_121507.csv`
- X+35 metadata: `/home/lyapunov/bone_drill/results/J10_A5/J10_workspace_x_pos350mm_20260909_121507_metadata.json`
- X+35 runtime log: `/home/lyapunov/bone_drill/results/J10_A5/J10_x35_runtime_20260909_1214.log`
- X+40 CSV: `/home/lyapunov/bone_drill/results/J10_A5/J10_workspace_x_pos400mm_20260909_121805.csv`
- X+40 metadata: `/home/lyapunov/bone_drill/results/J10_A5/J10_workspace_x_pos400mm_20260909_121805_metadata.json`
- X+40 runtime log: `/home/lyapunov/bone_drill/results/J10_A5/J10_x40_runtime_20260909_1217.log`

Production diff is empty, no controller or production driver changes were made, and no commit was created.
