# J10-A6.1 Actuator-Limit Policy Cleanup

本轮仅修改 J10 experiment-layer runner/tests；未运行真实硬件、未发送运动命令、未修改 J7/J8/J9 production、未 commit。

## 新 J10 actuator policy

- `predicted_step < 0` 或 `> 2000`：J10 actuator LIMIT，禁止 motion。
- `0 <= predicted_step <= 2000`：不因历史 `[200,1800]` 或旧 exploratory `[100,1900]` 越界而 LIMIT。
- `absolute_margin_i = min(q_i - 0, 2000 - q_i)`；`absolute_margin_min` 为六轴最小值。
- `limiting_actuator` 继续记录最小 absolute margin 轴。
- `[200,1800]` 仅保留为 historical metadata；越界设置 `historical_envelope_exceeded=true` 和 `maintenance_mode_required=true`，但不改变 J10 qualification。
- `[100,1900]` 保留为 deprecated compatibility metadata，不再作为 qualification gate。
- 0/2000 是软件允许范围端点，不作为主动测试目标；软件只保证不发送越界 command。

## Regression results

| Target step | Qualification | Limit reason | Absolute margin | Historical exceeded | Maintenance required |
|---:|---|---|---:|---|---|
| 1790 | PASS |  | 210 | false | false |
| 1850 | PASS |  | 150 | true | true |
| 1901 | PASS |  | 99 | true | true |
| 1999 | PASS |  | 1 | true | true |
| 2000 | PASS |  | 0 | true | true |
| 2001 | LIMIT | `predicted_step_outside_absolute_range` | -1 | true | true |
| -1 | LIMIT | `predicted_step_outside_absolute_range` | -1 | true | true |

The regression uses a fake kinematics provider and performs no hardware I/O. The production motor-driver safety semantics remain unchanged, including its normal `[200,1800]` preflight and explicit maintenance-mode requirement.

## Artifact schema

Added `absolute_margin_1..6`, `absolute_margin_min`, and `actuator_margin_semantics=absolute_[0,2000]`. The compatibility `actuator_margin_1..6` fields now carry the same absolute margins. `exploratory_min/max` remain metadata only and are accompanied by `exploratory_envelope_deprecated=true` and `exploratory_limits_hard_gate=false`.

## Verification

- `py_compile`: PASS
- J10 unit/regression tests: **25 passed**
- Production diff audit (`pus6_motor_driver`, `pus6_control`, `pus6_kinematics`, `up`, `STM32`): empty
- Hardware motion: **0**
- Commit: none

## Verdict

**J10-A6.1 PASS.** J10 qualification is now governed by absolute `[0,2000]` actuator range; historical and former exploratory envelopes are metadata/maintenance policy only. No X+44/X+45 hardware motion was run.
