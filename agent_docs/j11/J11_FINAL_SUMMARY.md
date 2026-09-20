# J11 Final Summary

**状态**：**J11 FINAL FREEZE READY**；Controller development CLOSED；pending Task 4 final Git freeze

## 目标与结果

J11 目标是在既有 6PUS IK、Driver、六轴 Servo、几何与安全链不变的条件下，验证并收口动基座姿态稳定方案。

- **M1（已冻结）**：Base-IMU full SO(3) Feedforward；Platform IMU 仅用于 Observer / Validation。
- **M2（最终 retained）**：Base-IMU SO(3) Feedforward + Platform-IMU Classical P/PD Attitude Feedback。
- **最终反馈形式**：Roll = P，Pitch = PD，Yaw = PD。

## 最终控制器

```text
Kp = [0.05, 0.05, 0.025]
Ki = [0.0, 0.0, 0.0]
Kd = [0.0, 0.0015, 0.0015] s
Derivative LPF fc = 5.0 Hz
Residual limits = +/-0.5 deg / axis
```

Roll-D `Kd_roll = 0.0010 s` 已完成 qualified repeat，但未保留；Roll-D tuning CLOSED。

## 关键归一化对比

| 指标 | M1 | M2 final | 观测下降 |
|---|---:|---:|---:|
| SO(3) G_Peak | 0.7353 | 0.3063 | ~58.3% |
| SO(3) G_RMS | 0.3713 | 0.2575 | ~30.6% |

两组均为人工 Base disturbance，输入不完全一致；以上是 representative normalized hardware characterization，不是严格同输入传递函数比较。完整 Roll/Pitch/Yaw/SO(3) 表、计算口径与边界见 [`J11_FINAL_CONTROL_COMPARISON.md`](../../docs/j11/J11_FINAL_CONTROL_COMPARISON.md)。

## 关键数据

- M1 canonical：`results/j11/runtime_validation/20260918_112354/J11_DISTURB_QUAL_feedforward_powered.csv`
- M1 summary：`results/j11/runtime_validation/20260918_112354/J11_DISTURB_QUAL_feedforward_summary.json`
- M2 retained representative：`results/j11/m2_pid/20260920_095850/j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_telemetry_powered.csv`
- M2 summary：`results/j11/m2_pid/20260920_095850/j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_summary_powered.json`

## Demo

```bash
bash bone_ws/scripts/run_j11_demo_feedforward.sh
bash bone_ws/scripts/run_j11_demo_feedforward_pd.sh
```

- **Demo A — Feedforward Only：PASS**（CONTROL ACTIVE、人工轻微移动、Ctrl+C/SIGINT 受控结束、自动 OPERATING_HOME、硬件输出停止、正常退出）。
- **Demo B — Feedforward + P/PD：PASS ON REPEAT**。首次 Demo B 因 motor CH340 USB-UART 瞬时 disconnect / re-enumeration 导致 `motion_write_failure` 提前结束，归类为 **runtime USB-UART transport interruption**（非 controller defect）；相同 controller / gains 的随后 repeat 正常完成。详见 [`J11_DEMO_B_USB_UART_FAULT_AUDIT.md`](./J11_DEMO_B_USB_UART_FAULT_AUDIT.md)。

## 收口与下一阶段

J11 controller development **CLOSED**，FINAL FREEZE READY，等待 Task 4 执行 final Git freeze（commit / tag / push）。J12 将比较 J11 Classical P/PD baseline 与 Platform attitude ADRC residual，研究可重复的外部偏心负载/力扰动抑制。
