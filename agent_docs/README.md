# 6PUS 并联机器人 Agent 阶段方案与架构设计文档库 (agent_docs)

本文档库为 6PUS 并联骨科手术机器人所有历史、当前与后续开发阶段的**权威架构设计、阶段规划、技术规范与交接总结**归档中心。

---

## 1. 核心架构与参考方案索引

| 阶段 / 文档名称 | 定位与核心内容 | 当前状态 |
| :--- | :--- | :---: |
| [`BONE_WS_ROS2_ARCHITECTURE_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/BONE_WS_ROS2_ARCHITECTURE_PLAN.md) | **全局 ROS2 架构总览**：分层拓扑、话题/服务契约、参数管理与安全状态机 | 权威基线 |
| [`CURRENT_STAGE.md`](file:///home/lyapunov/bone_drill/agent_docs/CURRENT_STAGE.md) | **轻量级当前阶段状态卡**：记录当前推进阶段、最后 PASS Gate、当前待验证 Gate 与下一步 | 动态维护 |
| [`J2-A_ROS2_READONLY_REAL_HARDWARE_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/J2-A_ROS2_READONLY_REAL_HARDWARE_PLAN.md) | **J2-A 六机只读通信实机验证方案**：60s 连续数据采集与 0-write 验证 | COMPLETE / PASS |
| [`J2-B_ROS2_SINGLE_MOTOR_REAL_HARDWARE_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/J2-B_ROS2_SINGLE_MOTOR_REAL_HARDWARE_PLAN.md) | **J2-B 单机微动 (ID4) 实机验证方案**：`+10 step` 微位移与闭环特性 | COMPLETE / PASS |
| [`J2-B2_ROS2_SINGLE_MOTOR_POSITION_TRACKING_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/J2-B2_ROS2_SINGLE_MOTOR_POSITION_TRACKING_PLAN.md) | **J2-B2 单机连续跟踪特性方案**：阶跃响应与时延分析 | COMPLETE / PASS |
| [`J2-C_ROS2_SIX_AXIS_VECTOR_REAL_HARDWARE_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/J2-C_ROS2_SIX_AXIS_VECTOR_REAL_HARDWARE_PLAN.md) | **J2-C 六机同步矢量运动方案**：六轴同时微动与同步到达特性 | COMPLETE / PASS |
| [`J2-F0_ROS2_READONLY_COMMUNICATION_PERFORMANCE_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/J2-F0_ROS2_READONLY_COMMUNICATION_PERFORMANCE_PLAN.md) | **J2-F0 只读实时通信表征方案**：单总线线时、抖动与 Auditor 审计架构 | PLANNING / PASS |
| [`J3_IK_REAL_HARDWARE_INTEGRATION_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/J3_IK_REAL_HARDWARE_INTEGRATION_PLAN.md) | **J3 IK/执行器闭环与预装配验证记录**：$X/Y/Z/\text{Roll}/\text{Pitch}/\text{Yaw}/\text{M13}$ | COMPLETE / PASS（不等于总装机械 FK） |
| [`J4_ROS2_CONTINUOUS_CARTESIAN_CONTROL_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/J4_ROS2_CONTINUOUS_CARTESIAN_CONTROL_PLAN.md) | **J4 连续笛卡尔控制管线架构**：50Hz 调度、状态滤波整形器、Watchdog 与实机爬坡方案 | PASS / FROZEN |
| [`J5_B3_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_B3_FINAL_SUMMARY.md) | **J5-B3 双 IMU 相对姿态算法与实时链路冻结总结**；同板共刚体验证边界 | PASS / FROZEN |
| [`J5_C2_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_C2_FINAL_SUMMARY.md) | **J5-C1/C2 软件冻结总结**：Timed q、Fixed-Lag、FK、A-F 实时消融与装配入口 | PASS / FROZEN（软件） |
| [`J5_FUTURE_OPTIMIZATION.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_FUTURE_OPTIMIZATION.md) | **J5 未来优化/备忘**：Hardware SYNC、外参、杆臂、融合候选 | 非当前开发任务 |
| [`ROS2_UART_EXISTING_CODE_AUDIT.md`](file:///home/lyapunov/bone_drill/agent_docs/ROS2_UART_EXISTING_CODE_AUDIT.md) | **历史 UART 代码审计与接管记录**：协议复用与架构边界梳理 | 归档 |

---

## 2. 维护规则与规范

1. **统一入口**：后续所有由 Agent（ChatGPT / Codex / Claude / Antigravity）生成的 stage plan、architecture spec、review report 或 handoff summary 均**默认存放在本 `agent_docs/` 目录**。
2. **职责边界**：
   - 根目录 [`AGENTS.md`](file:///home/lyapunov/bone_drill/AGENTS.md) 维护全局 Agent 规则与安全纪律；
   - 根目录 [`6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`](file:///home/lyapunov/bone_drill/6PUS_REAL_HARDWARE_OPERATION_GUIDE.md) 维护面向实验人员的实机操作总入口；
   - 本目录专注于技术方案细节，避免同一事实在多处反复冗余记录。
