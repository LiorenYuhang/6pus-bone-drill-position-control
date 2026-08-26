# 6PUS 并联机器人权威文档导航库 (agent_docs)

> **全局文档索引与维护地图**：本文档为 6PUS 并联机器人研发框架的权威文档索引。用于规范仓库内所有设计决策、阶段记录、历史冻结基线与实验报告的检索与阅读指引。

---

## 1. 文档体系导航地图 (Document Architecture Map)

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                   A. Current Recommended Docs (当前核心入口)            │
│  • README.md / README_CN.md (项目主页与最新状态)                        │
│  • 6PUS_SERVO_OPERATION_GUIDE.md (唯一权威日常实机操作指南)   │
│  • 6PUS_ROS2_CODE_LEARNING_TUTORIAL.md (12 章全栈架构与开发教程)        │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                 B. Current Technical Decisions (当前架构与技术决策)      │
│  • agent_docs/decisions/CURRENT_STAGE.md (阶段状态跟踪卡)                │
│  • agent_docs/decisions/J7_C0 ~ J7_C9 (J7 50Hz Servo 架构设计与时序审计)│
│  • agent_docs/architecture/BONE_WS_ROS2_ARCHITECTURE_PLAN.md (ROS 2)    │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                 C. Frozen Baselines (历史已封顶冻结基线，不可回写)       │
│  • J7 Frozen Tag: v1.0-j7-servo-hardware-frozen (当前 50Hz 伺服基线)    │
│  • J6 Frozen: agent_docs/j6/ (总装后定位模式 HOME/PARK 基线)            │
│  • J5 Frozen: agent_docs/frozen/ (装配前软件、时间轴与台架基线)         │
│  • J4/J3/J2/J1 Frozen: agent_docs/frozen/ & agent_docs/stages/          │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                 D. Stage & Historical Docs (分阶段过程资料与归档)       │
│  • agent_docs/stages/J2/ ~ J6/ (分阶段计划与测试规程)                   │
│  • hardware/ & up/ (历史独立硬件测试与传感器开发资料)                   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 核心文档分类与状态索引 (Master Document Index)

| 文档路径 | 核心用途与内容 | 所属阶段 | 状态类型 | 推荐阅读顺序 |
| :--- | :--- | :---: | :---: | :---: |
| [`README_CN.md`](file:///home/lyapunov/bone_drill/README_CN.md) / [`README.md`](file:///home/lyapunov/bone_drill/README.md) | GitHub 中英文主页，系统概览、架构图、J7 冻结状态与构建测试指南 | J7 (当前) | **Current** | **1** |
| [`6PUS_SERVO_OPERATION_GUIDE.md`](file:///home/lyapunov/bone_drill/6PUS_SERVO_OPERATION_GUIDE.md) | **唯一权威实机操作指南**：J7 50Hz 伺服推荐入口、J6 定位 fallback、安全门禁与故障排查 | J7 / J6 | **Current** | **2** |
| [`6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](file:///home/lyapunov/bone_drill/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md) | **12 章技术教程**：双链路架构、J7 50Hz 传输、解析逆解、LM 数值正解、时间同步与 TDD 体系 | J7 / J5 | **Current** | **3** |
| [`agent_docs/decisions/CURRENT_STAGE.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/CURRENT_STAGE.md) | 全局阶段演化状态跟踪卡，记录已冻结里程碑与下一步计划 | J7 (当前) | **Current** | **4** |
| [`agent_docs/architecture/BONE_WS_ROS2_ARCHITECTURE_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/architecture/BONE_WS_ROS2_ARCHITECTURE_PLAN.md) | ROS 2 双链路解耦架构顶层设计与节点划分规划 | J4 / J5 | **Current** | **5** |
| [`agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md) | J7 50Hz Servo 伺服运动控制架构设计规范 | J7 | **Current** | 6 |
| [`agent_docs/decisions/J7_C2_UART_TIMING_BUDGET_AUDIT.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J7_C2_UART_TIMING_BUDGET_AUDIT.md) | J7 单总线半双工 UART 时序预算与 20ms 周期边界审计 | J7 | **Current** | 7 |
| [`agent_docs/decisions/J7_C5_SIX_AXIS_SERVO_TRANSPORT_IMPLEMENTATION.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J7_C5_SIX_AXIS_SERVO_TRANSPORT_IMPLEMENTATION.md) | J7 六轴 Servo 纯写批次下发传输层技术实现 | J7 | **Current** | 8 |
| [`agent_docs/decisions/J7_C6_FEEDBACK_SCHEDULER_IMPLEMENTATION.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J7_C6_FEEDBACK_SCHEDULER_IMPLEMENTATION.md) | J7 轮询反馈调度器（1轴/周期 0x30 状态查询）实现说明 | J7 | **Current** | 9 |
| [`agent_docs/j6/J6_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/j6/J6_FINAL_SUMMARY.md) | J6 机械总装调试、零位映射与 HOME/PARK 定位模式 10 循环验收总结 | J6 | **Frozen** | 10 |
| [`agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md`](file:///home/lyapunov/bone_drill/agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md) | J5 装配前基线冻结发布归档清单与 SHA256 校验指纹 | J5 | **Frozen** | 11 |
| [`agent_docs/frozen/J5_FINAL_TEST_REPORT.md`](file:///home/lyapunov/bone_drill/agent_docs/frozen/J5_FINAL_TEST_REPORT.md) | J5 装配前 12 项系统级测试验收总报告 | J5 | **Frozen** | 12 |
| [`agent_docs/frozen/J5_SOFTWARE_ENVIRONMENT.md`](file:///home/lyapunov/bone_drill/agent_docs/frozen/J5_SOFTWARE_ENVIRONMENT.md) | J5 软件依赖、ROS 2 Jazzy、Python 及 By-ID 串口规格书 | J5 | **Frozen** | 13 |
| [`agent_docs/stages/J6/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J6/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md) | 机械总装后 9 阶段现场调试规程 | J6 | **Historical** | 14 |
| [`agent_docs/stages/J4/J4_ROS2_CONTINUOUS_CARTESIAN_CONTROL_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J4/J4_ROS2_CONTINUOUS_CARTESIAN_CONTROL_PLAN.md) | J4 连续笛卡尔轨迹控制规划方案 | J4 | **Historical** | 15 |
| [`agent_docs/research/future/J5_FUTURE_OPTIMIZATION.md`](file:///home/lyapunov/bone_drill/agent_docs/research/future/J5_FUTURE_OPTIMIZATION.md) | 未来研究主题备忘（ESKF 融合、DOB 主动抑振） | Future | **Historical** | 16 |

---

## 3. 分类维护规则 (Maintenance Principles)

1. **Current / Operator-facing**：保持与最新硬件基线严格一致（当前为 J7 50Hz Servo）。
2. **Frozen Historical Baseline**：`agent_docs/frozen/` 中的历史文档原则上保持内容不变，不将 J7 新结论倒写进 J5/J6 历史资料。
3. **Stage-Specific & Decisions**：`decisions/` 与 `stages/` 记录开发演进与技术方案，新增决策直接按 `Jn_` 规范归档。
4. **统一参数真值源**：所有运动学与物理参数唯一来源于 [`bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)。
