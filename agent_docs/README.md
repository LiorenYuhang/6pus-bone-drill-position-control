# 6PUS 并联机器人 Agent 文档库 (agent_docs)

本目录保存 6PUS 并联骨科手术机器人的架构、阶段方案、冻结基线、设计决策和研究路线文档。目录按文档职责组织，阶段编号不再直接混排在 `agent_docs/` 根目录。

## 1. 目录结构

```text
agent_docs/
├── README.md                         # 本索引与迁移映射
├── architecture/                     # 系统架构与 ROS2 架构文档
├── stages/
│   ├── J2/                           # J2 通信与执行器验证阶段
│   ├── J3/                           # J3 IK 与执行器集成阶段
│   ├── J4/                           # J4 连续笛卡尔控制阶段
│   ├── J5/                           # J5 装配前软件与台架验证阶段
│   └── J6/                           # J6 装配后 commissioning 阶段
├── frozen/                            # Frozen Manifest、冻结基线与冻结验证证据
├── decisions/                         # 当前状态指针与设计/架构决策记录
└── research/                          # 未来研究主题与研究路线备忘
```

## 2. 稳定入口

- 全局 Agent 规则：根目录 `AGENTS.md`。
- 实机操作入口：根目录 `6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`。
- 当前项目状态：`decisions/CURRENT_STAGE.md`。
- 全局 ROS2 架构：`architecture/BONE_WS_ROS2_ARCHITECTURE_PLAN.md`。
- 冻结基线索引：`frozen/` 内的 Frozen Manifest 和冻结发布文档。
- J6 装配后权威规程：`stages/J6/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`。

## 3. 分类维护规则

1. `architecture/` 只保存系统架构、ROS2 架构和长期接口边界文档。
2. `stages/Jn/` 保存对应阶段的计划、执行记录、审计和工具链准备文档。
3. `frozen/` 保存冻结版本、基线清单和与冻结版本绑定的验收证据；冻结文件不得因目录迁移而改写正文。
4. `decisions/` 保存动态状态指针、架构审计和设计决策记录。
5. `research/` 保存尚未成为当前工程基线的未来研究主题和路线备忘。

## 4. 本次迁移映射

本次迁移使用 `git mv` 保留文件历史。阶段文件按 J2/J3/J4/J5/J6 分组，冻结总结和基线文件统一移入 `frozen/`，`CURRENT_STAGE.md` 移入 `decisions/`。仓内非冻结文档和脚本中的旧路径已更新为新 canonical path；冻结文档正文中的历史路径保持不变，以保持冻结证据和 SHA256 指纹。

后续新增文档应直接放入对应分类目录，不再将新的阶段文档、冻结清单或设计决策放在 `agent_docs/` 根目录。
