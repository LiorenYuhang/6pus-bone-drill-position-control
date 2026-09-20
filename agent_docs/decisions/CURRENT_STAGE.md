# 6PUS 当前开发阶段状态卡 (Current Stage State Sheet)

**最后更新时间**: 2026-09-20

**当前状态**: **J11 FINAL FREEZE READY** / Controller development CLOSED / pending **Task 4 Final Git Freeze**

**下一研究阶段**: **J12 ADRC**

---

## 1. 阶段导航 (Stage Status)

* **当前推进阶段**: **J11 FINAL FREEZE READY**
  - **最终控制器**: Base-IMU SO(3) Feedforward + Platform-IMU Classical P/PD Attitude Feedback
  - **最终参数**: `Kp=[0.05,0.05,0.025]`, `Kd=[0,0.0015,0.0015] s`, `Ki=[0,0,0]`, `fc=5 Hz`
  - **实际反馈**: Roll=P, Pitch=PD, Yaw=PD
  - **Roll-D**: `Kd_roll=0.0010 s` qualified repeat 已完成但未保留；`Kd_roll=0`，调参 CLOSED
  - **Demo 状态**:
    1. **Demo A — Feedforward Only: PASS**
    2. **Demo B — Feedforward + P/PD: PASS ON REPEAT**
    3. **首次 Demo B anomaly**: TRANSIENT MOTOR USB-UART TRANSPORT LOSS（motor CH340 `usb 3-2` 瞬时 disconnect/re-enumeration → `motion_write_failure`）— **NON-BLOCKING ENGINEERING OBSERVATION**，见 [`agent_docs/j11/J11_DEMO_B_USB_UART_FAULT_AUDIT.md`](../j11/J11_DEMO_B_USB_UART_FAULT_AUDIT.md)
  - **下一执行步骤**: **TASK 4 — FINAL GIT FREEZE**（commit / tag / push；需用户明确确认）
  - **下一研究阶段**: **J12 External Payload / Force Disturbance + ADRC**
  - **权威入口**:
    - [`agent_docs/j11/J11_FINAL_SUMMARY.md`](../j11/J11_FINAL_SUMMARY.md)
    - [`docs/j11/J11_FINAL_CONTROL_COMPARISON.md`](../../docs/j11/J11_FINAL_CONTROL_COMPARISON.md)
    - [`agent_docs/j11/J11_TO_J12_HANDOFF.md`](../j11/J11_TO_J12_HANDOFF.md)
    - [`agent_docs/j11/J11_DEMO_B_USB_UART_FAULT_AUDIT.md`](../j11/J11_DEMO_B_USB_UART_FAULT_AUDIT.md)
  - **冻结结论仍有效**:
    - **`J11-M1 FEEDFORWARD ATTITUDE HOLD = PASS / FROZEN`**
    - **`J11 DRIVER RUNTIME = QUALIFIED FOR CONTROL RESEARCH`**
    - **`STOP ALL DRIVER / UART OPTIMIZATION`**
  - **最新实机验证**:
    1. **`J11 FINAL STATIC HOME 300s QUALIFICATION = PASS`** (2026-09-18 11:15:27):
       - 完整运行 300.0s，`hardware_write_count`: 14,944；
       - Two-Tier Safety Policy 生效：17 次暂态告警均安全恢复，无控制发散；
       - Hard STOP = 0，Overcurrent = 0（峰值 10mA），Motor fault = 0，>1.0s stale = 0；
       - HOME 跟踪误差 $\le 2\text{ steps}$（$\le 0.05\text{ mm}$）。
    2. **`J11 LOW-SPEED BASE DISTURBANCE QUALIFICATION = PASS`** (2026-09-18 11:23:54):
       - 120s 会话正常收敛完成，`hardware_write_count`: 5,929；
       - Base 多次动态扰动（最大 SO(3) 综合偏转 $13.54^\circ$），六轴真实产生平滑补偿运动（808 ~ 1203 steps）；
       - 平台 Tilt 跟踪误差稳定在 $1.55^\circ$ (RMS)，姿态抗扰趋势显著；
       - Hard STOP = 0，Overcurrent = 0（峰值 156mA <= 500mA），Motor fault = 0，>1.0s stale = 0，max stall ≈ 231ms。
  - **最终安全策略 (Two-Tier Safety Policy)**:
    - $\le 283.3\text{ms}$: **NORMAL**（正常 50Hz 伺服）；
    - $> 283.3\text{ms} \sim \le 1.0\text{s}$: **WARNING / HOLD**（警告，保持最近安全目标，等待通信恢复）；
    - $> 1.0\text{s}$: **HARD STOP**（真正通信失联立即熔断）；
    - 过流 $\ge 500\text{mA}$、硬件故障码 $\ne 0$、行程极限违背保持立即硬停机；
    - J7/J8 frozen baseline 默认行为严格保持不变。
  - **当前安全红线与禁令**:
    - **严禁继续开展任何 Driver / UART / 串口调度优化**（底层已完全满足闭环控制科研要求）；
    - 全力转向博士课题控制主线：J11-M2 Dual-IMU PID 残差姿态稳定。
* **已正式 PASS & 冻结的 Gates (Frozen Milestones)**:
  * **J11-M1 (Base-IMU 全 SO(3) 前馈实机闭环与驱动层资格认证)**: $\mathbf{PASS\ /\ FROZEN}$ (`v1.0-j11-m1-feedforward-hardware-qualified`)
  * **J1 (单总线半双工 921600 8N1 底层驱动协议)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J2 (六轴同步通信、0x29 伴随反馈与基准性能)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J3 (IK / 执行器闭环就位验证)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J4 (连续笛卡尔轨迹控制管线与安全策略)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J5 (装配前软件与传感器基线)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J6 (总装后现场调试与 HOME/PARK 定位基线)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J7 (六轴 50Hz Servo 伺服运动控制实机基线)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J8 (连续笛卡尔伺服、Design-B 0x29 伴随反馈与全自由度实机验证 -- 当前版本)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J9 (双 H30 IMU 传感集成、姿态解算与惯性表征)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J10 (HOME 单轴 6DOF 实机安全运行工作空间表征)**: $\mathbf{PASS\ /\ FROZEN}$
    - **表征对象**: HOME 单轴激励下的实机安全运行工作空间（非耦合 6D 几何全集）；
    - **实机测定范围**:
      - $X$: `[-45.75, +45.75] mm`（端点接近截断停机，余量 29 步）；
      - $Y$: `[-45.50, +49.00] mm`（端点接近截断停机，正向余量 31 步，负向余量 30 步）；
      - $Z$: `[-25.00, +25.00] mm`（六执行器 common-mode 完整行程 50.00 mm）；J10-A12 的笛卡尔采样覆盖曾在 +24.50 mm 处按端点接近协议停止，`STORAGE_PARK` 证据约为 `-24.983 mm`，均属于实验覆盖记录；
      - $\text{Roll}$: `[-35.0, +37.5] deg`（过流保护阈值停机：负向 `[-35.0, -37.5]°`，正向 `[+37.5, +40.0]°`）；
      - $\text{Pitch}$: `[-30.0, +35.0] deg`（过流保护阈值停机：负向 `[-30.0, -35.0]°`，正向 `[+35.0, +40.0]°`）；
      - $\text{Yaw}$: `[-90.0, +90.0] deg`（合格观察极限；+95° FK 数值解算/跟踪超时，+105° 观察到支链交叉缠绕物理风险，严禁重跑）；
    - **生产代码零修改**: 核心包 `pus6_motor_driver`、`pus6_kinematics`、`pus6_control` 保持冻结零变更。

---

## 2. 权威依据文档 (Authoritative References)

1. **J11 阶段设计与基线文档**: [`agent_docs/stages/J11/README.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J11/README.md)
2. **J10 收口与交接文档**: [`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J10/J10_FINAL_SUMMARY.md) / [`agent_docs/stages/J10/J10_TO_J11_HANDOFF.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J10/J10_TO_J11_HANDOFF.md)
3. **J9 收口与交接文档**: [`agent_docs/stages/J9/J9_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J9/J9_FINAL_SUMMARY.md) / [`agent_docs/stages/J9/J9_TO_J10_HANDOFF.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J9/J9_TO_J10_HANDOFF.md)
4. **J8 最终硬件准入报告**: [`agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md)
5. **唯一日常实机操作指南**: [`docs/整体学习参考/6PUS_SERVO_OPERATION_GUIDE.md`](../../docs/整体学习参考/6PUS_SERVO_OPERATION_GUIDE.md)
6. **ROS 2 架构与教学教程**: [`docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](../../docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)
7. **J7 基线设计与决策**: [`agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md)
