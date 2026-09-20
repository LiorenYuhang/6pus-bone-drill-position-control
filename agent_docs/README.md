# 6PUS 并联机器人权威文档导航库 (agent_docs)

> 本文档为 6PUS 并联机器人研发框架的**权威文档体系导航地图**。  
> 面向开发者与 Agent，提供清晰的目录分类说明、阶段结论快速检索入口与文档维护准则。项目级系统介绍请参阅根目录 [`README.md`](../README.md) 与 [`README_CN.md`](../README_CN.md)。

---

## 1. 目录架构与分类职责

```text
agent_docs/
├── stages/        # 分阶段实施方案、阶段性闭环总结与跨阶段交接规范 (Handoff)
├── reports/       # 深度实测分析报告、包络闭环分析与物理可辨识性研究
├── decisions/     # 架构决策记录 (ADR)、底层伺服设计与硬件准入结论
├── frozen/        # 历史里程碑冻结基线清单与代码/数据指纹 (Frozen Manifests)
├── archive/       # 历史遗留操作手册与过时参考资料 (保留追溯性，禁止逆向覆盖)
├── architecture/  # ROS 2 工作空间系统架构图与核心数据流规划
└── research/      # 算法预研路线、防抖控制选型与文献技术调研
```

### 分类用途说明：
- **`stages/` (阶段实施与交接)**：记录机器人各研发阶段（J1~J11）的详细技术方案、前置安全检查规程、阶段验收总结（`*_FINAL_SUMMARY.md`）以及阶段向后传递的交接清单（`*_TO_*_HANDOFF.md`）。
- **`reports/` (实测分析报告)**：存放人类与 Agent 撰写的高价值工程实测分析与实验归纳（如 J10 六轴单自由度安全包络闭环报告、J9.1 平台力臂与惯导平移耦合分析）。机读原始数据归入 `results/`。
- **`decisions/` (架构决策与准入)**：记录不可轻易推翻的关键系统决策（ADR）、底层通信时序审计以及正式里程碑的硬件准入结论（如 `J8_FINAL_HARDWARE_QUALIFICATION.md`）。
- **`frozen/` (冻结基线清单)**：记录历史封顶里程碑的只读代码与配置清单（如装配前 J5 冻结基线）。
- **`archive/` (历史归档)**：保存已过时但具有工程追溯价值的旧版操作指南与演进草稿。

---

## 2. 阶段正式结论与关键入口检索表 (Stage Master Index)

当需要查阅某阶段的正式结论、硬件准入状态或交接技术指标时，请通过下表直接检索：

| 阶段代号 | 阶段核心目标 | 关键结论与权威文档入口 | 状态 |
| :---: | :--- | :--- | :---: |
| **J1** | 电机只读通信与多机基线 | [`up/motor/uart_multi/AGENTS.md`](../up/motor/uart_multi/AGENTS.md) (ID1~ID6 串口基线) | **FROZEN** |
| **J2** | 解析几何逆运动学验证 | 方案：[`agent_docs/stages/J2/J2-C_ROS2_SIX_AXIS_VECTOR_REAL_HARDWARE_PLAN.md`](./stages/J2/J2-C_ROS2_SIX_AXIS_VECTOR_REAL_HARDWARE_PLAN.md) (解析逆解与多轴方案) | **FROZEN** |
| **J3** | FakeBus 调度与单槽覆盖 | 方案：[`agent_docs/stages/J3/J3_IK_REAL_HARDWARE_INTEGRATION_PLAN.md`](./stages/J3/J3_IK_REAL_HARDWARE_INTEGRATION_PLAN.md) (Latest-Value-Wins 机制) | **FROZEN** |
| **J4** | 连续轨迹整形与安全门禁 | 总结：[`agent_docs/frozen/J4_FINAL_SUMMARY.md`](./frozen/J4_FINAL_SUMMARY.md)<br>方案：[`agent_docs/stages/J4/J4_ROS2_CONTINUOUS_CARTESIAN_CONTROL_PLAN.md`](./stages/J4/J4_ROS2_CONTINUOUS_CARTESIAN_CONTROL_PLAN.md) | **FROZEN** |
| **J5** | 装配前统一时间轴与数值正解 | [`agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md`](./frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md) | **FROZEN** |
| **J6** | 机械总装与零位回归 | [`agent_docs/j6/J6_FINAL_SUMMARY.md`](./j6/J6_FINAL_SUMMARY.md) (HOME/PARK 定位验收) | **FROZEN** |
| **J7** | 六轴 50Hz 伺服驱动通信 | [`agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md`](./decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md) | **FROZEN** |
| **J8** | 笛卡尔空间连续实机运动 | [`agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md`](./decisions/J8_FINAL_HARDWARE_QUALIFICATION.md) (9阶段视频演示准入) | **FROZEN** |
| **J9** | 双 H30 IMU 传感集成与姿态验证 | [`agent_docs/stages/J9/J9_FINAL_SUMMARY.md`](./stages/J9/J9_FINAL_SUMMARY.md)<br>交接：[`agent_docs/stages/J9/J9_TO_J10_HANDOFF.md`](./stages/J9/J9_TO_J10_HANDOFF.md) | **FROZEN** |
| **J10** | 6DOF 单轴实机安全运行包络 | 总结：[`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](./stages/J10/J10_FINAL_SUMMARY.md)<br>报告库：[`agent_docs/reports/j10/`](./reports/j10/) (15篇单轴报告)<br>交接：[`agent_docs/stages/J10/J10_TO_J11_HANDOFF.md`](./stages/J10/J10_TO_J11_HANDOFF.md) | **CLOSED** |
| **J11** | 世界坐标系姿态防抖闭环 | 最终总结：[`agent_docs/j11/J11_FINAL_SUMMARY.md`](./j11/J11_FINAL_SUMMARY.md)<br>最终中文对比：[`docs/j11/J11_FINAL_CONTROL_COMPARISON.md`](../docs/j11/J11_FINAL_CONTROL_COMPARISON.md)<br>详细调参档案：[`agent_docs/j11/J11_M1_M2_CONTROL_COMPARISON.md`](./j11/J11_M1_M2_CONTROL_COMPARISON.md)<br>交接：[`agent_docs/j11/J11_TO_J12_HANDOFF.md`](./j11/J11_TO_J12_HANDOFF.md)<br>USB fault 审计：[`agent_docs/j11/J11_DEMO_B_USB_UART_FAULT_AUDIT.md`](./j11/J11_DEMO_B_USB_UART_FAULT_AUDIT.md)<br>**Final controller**: `Kp=[0.05,0.05,0.025], Kd=[0,0.0015,0.0015] s, Ki=0, fc=5 Hz; Roll=P, Pitch=PD, Yaw=PD` | **FINAL FREEZE READY**<br>*(pending Task 4 Git freeze)* |

---

## 3. 跨阶段交接规范 (Handoff Navigation)

跨阶段交接文档用于确保上一阶段的物理事实、不可动基线与安全限制完整传递至下一阶段：
- **J9 $\to$ J10 交接**：[`agent_docs/stages/J9/J9_TO_J10_HANDOFF.md`](./stages/J9/J9_TO_J10_HANDOFF.md)（交付 IMU 静态/动态标定与姿态精度限制）。
- **J10 $\to$ J11 交接**：[`agent_docs/stages/J10/J10_TO_J11_HANDOFF.md`](./stages/J10/J10_TO_J11_HANDOFF.md)（交付 J10 单轴实测极限，明确 J8 冻结控制黑盒与标定延后约束）。
- **J11 $\to$ J12 交接**：[`agent_docs/j11/J11_TO_J12_HANDOFF.md`](./j11/J11_TO_J12_HANDOFF.md)（交付最终 Classical P/PD baseline、外部偏心负载协议与 ADRC A/B 边界）。

---

## 4. 文档维护准则

1. **唯一事实源 (Single Source of Truth)**：
   - 机构几何参数与物理换算系数唯一来源于 [`bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml`](../bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)。
   - 坐标系定义、原点位置（`OPERATING_HOME = [1000]*6`）、软件零位属于不可篡改项目级不变量。
2. **文档与数据隔离**：
   - 用户需要主动阅读的文档（学习/操作/教程/最终技术结果）置于 `docs/`；
   - Agent 内部工作文档（stage 历史、决策、调参历史、交接、故障审计）置于 `agent_docs/`；
   - 自动化测试与实机运行生成的 CSV、JSON、日志等数据资产必须置于 `results/`。
3. **真实性与证据一致性**：
   - 严禁将 Mock/仿真测试表述为 50Hz 物理硬件实测结果。
   - 所有指标与结论必须能在 `results/` 权威数据集中找到对应的实验日志与时间戳。
