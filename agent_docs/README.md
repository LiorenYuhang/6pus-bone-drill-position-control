# 6PUS 并联机器人权威文档导航库 (agent_docs)

> **全局文档索引与维护地图**：本文档为 6PUS 并联机器人研发框架的权威文档索引。用于规范仓库内所有设计决策、阶段记录、历史冻结基线与实验报告的检索与阅读指引。

---

## 1. 文档体系导航地图 (Document Architecture Map)

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                   A. Current Recommended Docs (当前核心入口)            │
│  • README.md / README_CN.md (项目主页与最新状态)                        │
│  • 6PUS_SERVO_OPERATION_GUIDE.md (唯一权威日常实机操作指南)             │
│  • 6PUS_ROS2_CODE_LEARNING_TUTORIAL.md (12 章全栈架构与开发教程)        │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                 B. Current Technical Decisions (当前架构与技术决策)      │
│  • agent_docs/decisions/CURRENT_STAGE.md (阶段状态跟踪卡)                │
│  • agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md (J8 硬件准入)│
│  • agent_docs/decisions/J7_C0 ~ J7_C9 (J7 50Hz Servo 架构设计与时序审计)│
│  • agent_docs/architecture/BONE_WS_ROS2_ARCHITECTURE_PLAN.md (ROS 2)    │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                 C. Frozen Baselines (历史已封顶冻结基线，不可回写)       │
│  • J8 Frozen Tag: v1.0-j8-active-servo-hardware-qualified (当前基线)    │
│  • J7 Frozen Tag: v1.0-j7-servo-hardware-frozen (50Hz 伺服驱动基线)     │
│  • J6 Frozen: agent_docs/j6/ (总装后定位模式 HOME/PARK 基线)            │
│  • J5 Frozen: agent_docs/frozen/ (装配前软件、时间轴与台架基线)         │
│  • J4/J3/J2/J1 Frozen: agent_docs/frozen/ & agent_docs/stages/          │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                 D. Stage & Historical Docs (分阶段过程资料与归档)       │
│  • agent_docs/stages/J2/ ~ J10/ (分阶段计划与测试规程，含 J10 收口与 J11 交接)                  │
│  • hardware/ & up/ (历史独立硬件测试与传感器开发资料)                   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 核心文档分类与状态索引 (Master Document Index)

| 文档路径 | 核心用途与内容 | 所属阶段 | 状态类型 | 推荐阅读顺序 |
| :--- | :--- | :---: | :---: | :---: |
| [`README_CN.md`](file:///home/lyapunov/bone_drill/README_CN.md) / [`README.md`](file:///home/lyapunov/bone_drill/README.md) | GitHub 中英文主页，系统概览、架构图、J10 收口状态与构建测试指南 | J10 (Current) | **Current** | **1** |
| [`6PUS_SERVO_OPERATION_GUIDE.md`](file:///home/lyapunov/bone_drill/6PUS_SERVO_OPERATION_GUIDE.md) | **唯一权威实机操作指南**：J8 笛卡尔/伺服推荐入口、集中式状态分类器、安全门禁与故障排查 | J8 / J7 | **Current** | **2** |
| [`agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md) | **J8 最终实机准入报告**：Design-B 伴随反馈、两级看门狗、三轮验证与 9 阶段实机展示报告 | J8 | **Current** | **3** |
| [`6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](file:///home/lyapunov/bone_drill/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md) | **12 章技术教程**：双链路架构、J7/J8 50Hz 传输、解析逆解、LM 数值正解、时间同步与 TDD 体系 | J8 / J7 / J5 | **Current** | **4** |
| [`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J10/J10_FINAL_SUMMARY.md) | **J10 6DOF 工作空间实测总结**：HOME 单轴 6 自由度实测极限、端点截断规则与过流/缠绕机理分层 | J10 | **Current** | **5** |
| [`agent_docs/stages/J10/J10_TO_J11_HANDOFF.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J10/J10_TO_J11_HANDOFF.md) | **J10 至 J11 标定交接文档**：标定核心任务、需复核指标、J10B 触发条件与边界禁令 | J10 / J11 | **Current** | **6** |
| [`agent_docs/decisions/CURRENT_STAGE.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/CURRENT_STAGE.md) | 全局阶段演化状态跟踪卡，记录已冻结里程碑与下一步计划 | J10 (Current) | **Current** | **7** |
| [`agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md) | J7 50Hz Servo 伺服运动控制架构设计规范 | J7 | **Historical** | 8 |
| [`agent_docs/j6/J6_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/j6/J6_FINAL_SUMMARY.md) | J6 机械总装调试、零位映射与 HOME/PARK 定位模式 10 循环验收总结 | J6 | **Frozen** | 9 |
| [`agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md`](file:///home/lyapunov/bone_drill/agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md) | J5 装配前基线冻结发布归档清单与 SHA256 校验指纹 | J5 | **Frozen** | 10 |

---

## 3. 分类维护规则 (Maintenance Principles)

1. **Current / Operator-facing**：保持与最新硬件基线严格一致（当前为 J9 IMU Integration & Measurement Qualification，运行基线仍为 J8 冻结伺服）。
2. **Frozen Historical Baseline**：`agent_docs/frozen/` 中的历史文档原则上保持内容不变。
3. **Stage-Specific & Decisions**：`decisions/` 与 `stages/` 记录开发演进与技术方案，新增决策直接按 `Jn_` 规范归档。
4. **统一参数真值源**：所有运动学与物理参数唯一来源于 [`bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)。
