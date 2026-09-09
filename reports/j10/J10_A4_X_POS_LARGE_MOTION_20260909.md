# J10-A4 X+ Large-Motion Qualification & Re-validation

| X+ (mm) | predicted steps | command→target (s) | feedback→target (s) | candidate validity | HOME duration (s) | HOME |
|---:|---|---:|---:|---|---:|---|
| 0.5 | `[999, 1003, 1004, 997, 997, 1001]` | 0.185 | 0.542 | legacy candidate stage; strict window unavailable | 0.014 | legacy completed |
| 2.0 | `[998, 1012, 1015, 987, 990, 1004]` | 0.343 | 0.811 | legacy candidate stage; strict window unavailable | 0.479 | legacy completed |
| 5.0 | `[998, 1033, 1041, 971, 979, 1014]` | 0.540 | 0.729 | legacy candidate stage; strict window unavailable | 0.949 | legacy pending/completion metadata inconsistent |
| 10.0 | `[1008, 1079, 1096, 954, 970, 1041]` | 0.687 | 2.275 | legacy candidate stage; strict window unavailable | 1.990 | legacy completed |
| 15.0 | `[1030, 1138, 1164, 949, 974, 1080]` | 1.019 | — | legacy candidate stage; feedback remained >3 steps at capture | — | legacy completed |
| 20.0 | `[1065, 1211, 1247, 956, 989, 1132]` | 1.403 | 3.658 | legacy window entered before feedback target; invalid under A4 gate | 3.859 | True |
| 25.0 | `[1112, 1300, 1345, 975, 1017, 1198]` | 1.885 | 5.907 | A4 final valid: 164 samples, 0 invalid | 5.772 | True |
| 30.0 | `[1173, 1404, 1461, 1007, 1057, 1279]` | 2.477 | 5.221 | A4 final valid: 117 samples, 0 invalid | 6.762 | True |

## X+30 legacy root cause

The old runner fixed candidate transition at 3 s and entered measurement without requiring target feedback. X+20 feedback reached target at 3.658 s, after measurement began. X+30 never reached target during the captured candidate stage; final candidate feedback error was 119 steps and FK ΔX was about 19.7 mm. Feedback updated and fault codes remained zero, so the primary cause is the stage/measurement gate plus insufficient wait, not a demonstrated actuator or mechanical limit.

## J10 runner correction

The J10 experiment runner now derives candidate/HOME timeouts from maximum step travel using the observed 60 steps/s feedback rate and bounded 1.5 s margin. It requires five consecutive new feedback samples with command and feedback at target before measurement and validates tracking across the full window. Only J10 runner/tests changed; production diff is empty. `py_compile` passed and 18 tests passed.

## X+25 final verification

PASS: candidate reached in 6.232 s (derived timeout 7.25 s), 164 valid measurement samples with zero invalid samples, HOME confirmed in 5.772 s; final feedback `[996,1000,1000,999,1001,1000]`.

## X+30 final re-validation

PASS: candidate reached in 5.451 s (derived timeout 9.183 s), 117 valid measurement samples with zero invalid samples, HOME confirmed in 6.762 s; final feedback `[997,1000,1000,1001,1000,1001]`. FK coupling relative to pre-home median: ΔX +29.811 mm, ΔY −0.154 mm, ΔZ −0.021 mm, ΔRoll −0.056°, ΔPitch +0.052°, ΔYaw +0.061°.

Current maximum fully-qualified X+ is **30 mm**. This phase stopped at +30 mm; any +35/+40 mm work requires a separate authorized phase using the same gates.
