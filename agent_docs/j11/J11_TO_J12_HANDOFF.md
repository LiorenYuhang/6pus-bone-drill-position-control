# J11 -> J12 Handoff

**J11 状态**：CLOSED / **FINAL FREEZE READY**（pending Task 4 Git freeze）

**执行前置**：Task 2 两个 Demo hardware smoke test 已完成 — Demo A PASS；Demo B PASS ON REPEAT（首次因 motor CH340 USB-UART 瞬时 disconnect 提前结束，见 [`J11_DEMO_B_USB_UART_FAULT_AUDIT.md`](./J11_DEMO_B_USB_UART_FAULT_AUDIT.md)）。

## 1. J11 交付基线

J12 的 classical baseline 为：

```text
Base IMU -> SO(3) Feedforward
Platform IMU -> Classical P/PD attitude residual
compose R_BP_cmd -> IK -> existing six-axis servo

Kp = [0.05, 0.05, 0.025]
Ki = [0.0, 0.0, 0.0]
Kd = [0.0, 0.0015, 0.0015] s
fc = 5.0 Hz
residual limits = +/-0.5 deg / axis
Roll = P, Pitch = PD, Yaw = PD
```

权威结果与数据索引见 [`J11_FINAL_CONTROL_COMPARISON.md`](../../docs/j11/J11_FINAL_CONTROL_COMPARISON.md)。

## 2. J12 研究目标

J12：**External Payload / Force Disturbance + ADRC Disturbance Rejection**。

- Baseline：J11 final Classical P/PD controller。
- Candidate：Base IMU SO(3) Feedforward + Platform attitude ADRC residual。
- 下层保持：`IK -> existing six-axis servo`。
- 不预设 ADRC 一定优于 classical baseline，结论必须来自同 protocol 的 A/B evidence。

## 3. 不重新打开的范围

除非出现可复现的真实 blocking issue，J12 不重新打开：

- J7 / J8；
- Driver / UART / motor protocol；
- IK、geometry、OPERATING_HOME；
- J10 workspace；
- M1 frozen implementation；
- J11 gains 与 Roll-D tuning；
- IMU calibration；
- hardware safety thresholds。

## 4. 外部负载与扰动力矩设计

重物若放在 Platform 几何中心，主要增加竖直负载，不一定形成明显姿态扰动力矩。J12 优先使用：

- known payload mass `m`；
- known eccentric mounting position `r`；
- 可计算的重力扰动力矩 `tau_d = r x m g`。

这样可以形成可重复的姿态扰动，而不是依赖未知手动输入。

## 5. A/B 实验约束

J11 Classical P/PD 与 J12 ADRC 的比较必须保持：

```text
same payload
same mounting position
same initial condition
same disturbance protocol
```

比较指标、异常停止、未达标 run 与原始 telemetry 均应完整保留。若实际输入仍不完全一致，应继续使用 normalized metrics 并明确解释边界。

## 6. J12 开始前 Gate

1. ✅ Feedforward Demo hardware smoke test — PASS。
2. ✅ Feedforward + P/PD Demo hardware smoke test — PASS ON REPEAT（首次为 transient USB-UART transport interruption，非 controller defect）。
3. 冻结 J11 final 文档、wrapper 与 retained config（Task 4 Git freeze）后，方可开始 J12 controller development。
