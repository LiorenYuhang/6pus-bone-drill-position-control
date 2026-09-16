# 6PUS 当前开发阶段状态卡 (Current Stage State Sheet)

**最后更新时间**: 2026-09-09
**当前冻结状态**: $\mathbf{J10\ 6DOF\ Axis\text{-}Wise\ Operational\ Safe\ Workspace\ Measurement\ COMPLETE}$
*(官方 Release Tag: `v1.0-j8-active-servo-hardware-qualified`)*

---

## 1. 阶段导航 (Stage Status)

* **当前推进阶段**: **$\mathbf{J11\ PID\ 3DOF\ World\text{-}Attitude\ Stabilization\ /\ Anti\text{-}Vibration}$ (J11-1 推进中，物理标定 Calibration 暂缓并移出 J11)**
  - **核心目标**: Base 承受人工施加的 Roll/Pitch/Yaw 姿态扰动时，利用 Platform H30 姿态反馈与任务空间 PID 控制器，使 Platform 尽可能保持世界坐标系初始参考姿态；
  - **传感器职责**: Platform H30 为主反馈传感器；Base H30 仅用于扰动输入记录、日志及防抖性能对比评定（OFF vs ON），不进入 J11 PID 控制律；
  - **范围边界**: 仅处理 3DOF 姿态，XYZ 平动防抖不在本阶段范围内；
  - **执行底座**: 严格复用 J8 冻结生产链路（Cartesian -> IK -> Shaper -> 50Hz Servo Driver），零修改底层核心；
  - **标定说明**: 原计划的执行器与运动学参数标定（Actuator & Kinematic Calibration）正式延后（DEFERRED），移至后续独立阶段，不再占用 J11 编号；
  - **当前安全红线**: J11-1 期间 $\mathbf{hardware\_write\_count = 0}$，禁止真实电机运动。
* **已正式 PASS & 冻结的 Gates (Frozen Milestones)**:
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
5. **唯一日常实机操作指南**: [`6PUS_SERVO_OPERATION_GUIDE.md`](file:///home/lyapunov/bone_drill/6PUS_SERVO_OPERATION_GUIDE.md)
6. **ROS 2 架构与教学教程**: [`6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](file:///home/lyapunov/bone_drill/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)
7. **J7 基线设计与决策**: [`agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md)
