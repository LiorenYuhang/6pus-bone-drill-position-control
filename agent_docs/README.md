# 6PUS 并联机器人权威文档导航库 (agent_docs)

> 本文档为 6PUS 并联机器人研发框架的**权威文档体系导航地图与 Agent 推荐阅读索引**。
> 面向开发者与 Agent，提供清晰的目录分类说明、阶段结论检索入口与推荐检索路径。项目级系统概览请参阅根目录 [`README.md`](../README.md) 与 [`README_CN.md`](../README_CN.md)。

---

## 1. Agent 推荐读取顺序 (Agent Reading Workflow)

为防止 Agent 默认加载全量历史阶段产生上下文冗余或读取陈旧状态，所有 Agent 必须遵循以下由顶向下的精准检索链：

```text
1. 根目录全局安全与工程准则: /home/lyapunov/bone_drill/AGENTS.md
   ↓
2. 本导航索引地图: agent_docs/README.md
   ↓
3. 当前阶段唯一状态卡: agent_docs/decisions/CURRENT_STAGE.md
   ↓
4. 当前基线正式总结与交接:
   - 权威总结: agent_docs/stages/J14/J14_FINAL_SUMMARY.md
   - 权威交接: agent_docs/stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md
   ↓
5. 生产级驱动与 Demo 使用:
   - 生产架构: docs/J14_PRODUCTION_DRIVER.md
   - Demo 入口: docs/J14_DEMO_ENTRYPOINTS.md
   ↓
6. (按需查阅) 历史阶段专区: agent_docs/stages/J*/
```

---

## 2. 目录架构与分类职责

```text
agent_docs/
├── stages/        # 按阶段统一归档的技术方案、阶段总结 (FINAL_SUMMARY) 与交接规范 (HANDOFF)
│   ├── J1 ~ J10/  # J1至J10历史阶段方案与总结
│   ├── J11/       # J11 姿态稳定闭环方案、调参档案与手板记录 (已合并)
│   ├── J12/       # J12 经典控制高动态性能与物理负载摸底
│   ├── J13/       # J13 一阶 LADRC 姿态残差控制与偏心负载验证
│   └── J14/       # J14 三进程驱动解耦、长时通信可靠性与 Demo 全生命周期
├── reports/       # 深度实测分析报告、包络闭环分析与物理可辨识性研究
├── decisions/     # 架构决策记录 (ADR)、底层伺服设计与硬件准入结论 (含 CURRENT_STAGE.md)
├── frozen/        # 历史里程碑冻结基线清单与代码/数据指纹 (Frozen Manifests)
├── archive/       # 历史遗留操作手册与过时参考资料 (保留追溯性，禁止逆向覆盖)
├── architecture/  # ROS 2 工作空间系统架构图与核心数据流规划
└── research/      # 算法预研路线、防抖控制选型与文献技术调研
```

---

## 3. 阶段正式结论与关键入口检索表 (Stage Master Index)

| 阶段代号 | 阶段核心目标 | 关键结论与权威文档入口 | 状态 |
| :---: | :--- | :--- | :---: |
| **J1** | 电机只读通信与多机基线 | [`up/motor/uart_multi/AGENTS.md`](../up/motor/uart_multi/AGENTS.md) (ID1~ID6 串口基线) | **FROZEN** |
| **J2** | 解析几何逆运动学验证 | 方案：[`agent_docs/stages/J2/J2-C_ROS2_SIX_AXIS_VECTOR_REAL_HARDWARE_PLAN.md`](./stages/J2/J2-C_ROS2_SIX_AXIS_VECTOR_REAL_HARDWARE_PLAN.md) | **FROZEN** |
| **J3** | FakeBus 调度与单槽覆盖 | 方案：[`agent_docs/stages/J3/J3_IK_REAL_HARDWARE_INTEGRATION_PLAN.md`](./stages/J3/J3_IK_REAL_HARDWARE_INTEGRATION_PLAN.md) | **FROZEN** |
| **J4** | 连续轨迹整形与安全门禁 | 总结：[`agent_docs/frozen/J4_FINAL_SUMMARY.md`](./frozen/J4_FINAL_SUMMARY.md) | **FROZEN** |
| **J5** | 装配前统一时间轴与数值正解 | 总结：[`agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md`](./frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md) | **FROZEN** |
| **J6** | 机械总装与零位回归 | 总结：[`agent_docs/stages/J6/J6_PRE_ASSEMBLY_PREPARATION_SUMMARY.md`](./stages/J6/J6_PRE_ASSEMBLY_PREPARATION_SUMMARY.md) | **FROZEN** |
| **J7** | 六轴 50Hz 伺服驱动通信 | 设计：[`agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md`](./decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md) | **FROZEN** |
| **J8** | 笛卡尔空间连续实机运动 | 准入：[`agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md`](./decisions/J8_FINAL_HARDWARE_QUALIFICATION.md) | **FROZEN** |
| **J9** | 双 H30 IMU 传感集成与姿态验证 | 总结：[`agent_docs/stages/J9/J9_FINAL_SUMMARY.md`](./stages/J9/J9_FINAL_SUMMARY.md) | **FROZEN** |
| **J10** | 6DOF 单轴实机安全运行包络 | 总结：[`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](./stages/J10/J10_FINAL_SUMMARY.md) | **CLOSED** |
| **J11** | 世界坐标系姿态防抖闭环 | 权威总结：[`agent_docs/stages/J11/J11_FINAL_SUMMARY.md`](./stages/J11/J11_FINAL_SUMMARY.md)<br>调参档案：[`agent_docs/stages/J11/J11_M1_M2_CONTROL_COMPARISON.md`](./stages/J11/J11_M1_M2_CONTROL_COMPARISON.md)<br>交接：[`agent_docs/stages/J11/J11_TO_J12_HANDOFF.md`](./stages/J11/J11_TO_J12_HANDOFF.md)<br>**Final controller**: `Kp=[0.05,0.05,0.025], Kd=[0,0.0015,0.0015] s, Ki=0, fc=5 Hz; Roll=P, Pitch=PD, Yaw=PD` | **FROZEN**<br>(`v1.0-j11-attitude-stabilization-frozen`) |
| **J12** | 经典前馈+P/PD动态性能与物理负载摸底 | 权威总结：[`agent_docs/stages/J12/J12_FINAL_SUMMARY.md`](./stages/J12/J12_FINAL_SUMMARY.md)<br>交接：[`agent_docs/stages/J12/J12_TO_J13_HANDOFF.md`](./stages/J12/J12_TO_J13_HANDOFF.md)<br>**Scope**: J12-1 静态/构型负载、J12-2 组合姿态筛选、J12-3 手持高动态表征与生产 1500mA 电流保护对齐 | **CLOSED / FROZEN**<br>(`v1.0-j12-classical-dynamic-performance-frozen`) |
| **J13** | 一阶线性自抗扰姿态残差控制实机闭环与偏心验证 | 权威总结：[`agent_docs/stages/J13/J13_FINAL_SUMMARY.md`](./stages/J13/J13_FINAL_SUMMARY.md)<br>交接：[`agent_docs/stages/J13/J13_TO_J14_HANDOFF.md`](./stages/J13/J13_TO_J14_HANDOFF.md)<br>**Final controller**: 3-axis First-Order LADRC, $b_0=[20,10,6], \omega_c=[0.8,1.0,0.25], \omega_o=[2.5,4.0,1.2]$, clamp $\pm 0.5^\circ$ | **CLOSED / ENGINEERING QUALIFIED**<br>(`v1.0-j13-ladrc-attitude-residual-control-qualified`) |
| **J14-A** | 长时运行与通信可靠性攻关 | 权威总结：[`agent_docs/stages/J14/J14_FINAL_SUMMARY.md`](./stages/J14/J14_FINAL_SUMMARY.md)<br>交接：[`agent_docs/stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md`](./stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md)<br>**Production Architecture**: Parent (SingleThreadedExecutor) + UART Child (独占串口) + Feedback Bridge (独占反馈 IPC)<br>**Hardware Qualification**: Q3-R1D Classical PASS / Q3-R2 LADRC PASS / 3 Demo HUMAN HARDWARE CONFIRMATION PASS | **CLOSED / FROZEN**<br>(`v1.0-j14-a-three-process-runtime-reliability-qualified`) |

---

## 4. 跨阶段交接规范 (Handoff Navigation)

跨阶段交接文档用于确保上一阶段的物理事实、不可动基线与安全限制完整传递至下一阶段：
- **J9 $	o$ J10 交接**：[`agent_docs/stages/J9/J9_TO_J10_HANDOFF.md`](./stages/J9/J9_TO_J10_HANDOFF.md)
- **J10 $	o$ J11 交接**：[`agent_docs/stages/J10/J10_TO_J11_HANDOFF.md`](./stages/J10/J10_TO_J11_HANDOFF.md)
- **J11 $	o$ J12 交接**：[`agent_docs/stages/J11/J11_TO_J12_HANDOFF.md`](./stages/J11/J11_TO_J12_HANDOFF.md)
- **J12 $	o$ J13 交接**：[`agent_docs/stages/J12/J12_TO_J13_HANDOFF.md`](./stages/J12/J12_TO_J13_HANDOFF.md)
- **J13 $	o$ J14 交接**：[`agent_docs/stages/J13/J13_TO_J14_HANDOFF.md`](./stages/J13/J13_TO_J14_HANDOFF.md)
- **J14 $	o$ Next Stage 交接**：[`agent_docs/stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md`](./stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md)

---

## 5. 文档维护准则

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
