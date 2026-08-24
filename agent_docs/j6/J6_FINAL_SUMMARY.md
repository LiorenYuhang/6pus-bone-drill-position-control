# J6 HOME/PARK 最终验证摘要

## 1. 范围

本摘要冻结 J6 电机级 HOME/PARK commissioning 基线。范围是六轴独立启动/收拢脚本、现有电机驱动与安全门禁，以及已授权的真实硬件重复验证；不包含完整 6PUS 平台笛卡尔轨迹、IK、ROS 2 连续控制或 J5 冻结资产的变更。

详细的门禁、审计和十循环程序见：

- [J6 HOME/PARK 基线与十循环程序](J6_HOME_PARK_VALIDATED_BASELINE_AND_10_CYCLE_PROCEDURE_CN.md)
- [J6 HOME commissioning gate 更新](J6_HOME_COMMISSIONING_GATE_UPDATE_CN.md)

## 2. 已验证配置

| 项目 | 冻结值 |
| --- | --- |
| standalone runner | `bone_ws/scripts/run_j3_startup_homing.py` |
| 控制模式寄存器 `0x25` | `0`（Positioning Mode） |
| commissioning speed | `50` |
| standalone 控制周期 | `10 Hz` |
| HOME 目标（`0x29`） | `[1000,1000,1000,1000,1000,1000]` |
| PARK 目标（`0x29`） | `[0,0,0,0,0,0]` |
| 目标安全范围 | `[0,2000]` steps |

配置阶段只写模式及其他配置寄存器；轨迹阶段才写 `0x29` 目标位置。反馈中的小范围零偏由反馈容差处理，不改变命令限位。

## 3. HOME/PARK 定义与实机结果

- **HOME / OPERATING_HOME**：六轴目标 `[1000]*6`，作为后续姿态控制的软件工作基准。
- **PARK / STORAGE_PARK**：六轴目标 `[0]*6`，仅用于受控收拢、停放和关机。
- 在上述 mode=0、speed=50、standalone 10 Hz 配置下，HOME 与 PARK 均已在真实硬件上重复通过超过 10 个循环。
- 观测到的跟踪误差最大值不超过 2 steps；通信、目标下发和停止确认均正常。

以上结论是用户提供的真实硬件验证结果；本次文档整理未连接或驱动硬件。

## 4. 速度试验边界

| speed | HOME/PARK 结果 | 结论 |
| ---: | --- | --- |
| 50 | PASS，重复通过（>10 cycles） | 当前 commissioning 基线 |
| 60 | 不稳定，可能触发 tracking stop | 不纳入基线 |
| 100/200 | 不稳定，可能触发 tracking stop | 不纳入基线 |

speed=50 是已验证、可重复的 commissioning 工作点，不是电机最大能力或额定速度上限。

## 5. 已知限制

1. standalone runner 当前为 10 Hz；它是 J6 点到点 commissioning 工具，不代表 ROS 2 50 Hz 控制链的实机性能。
2. speed=60 及以上未形成稳定基线；不得在未单独授权和设计实验前提高速度。
3. 本验证只覆盖电机级 HOME/PARK，不宣称完整机构装配后的笛卡尔精度、IK/FK 精度或 IMU 融合性能。
4. 任何 fault、overcurrent、通信失败、无效状态或 tracking watchdog STOP 都必须按现有安全流程处理，不得通过放宽门限规避。

## 6. 下一阶段建议

先在 Mock/FakeBus 中将 standalone 基线与 ROS 2 控制管线做接口对照和回归，再由负责人单独授权分阶段的完整机构验证。后续阶段应沿用 `OPERATING_HOME` / `STORAGE_PARK` 定义，并保持协议、安全限位、几何和 J5 冻结资产不变。
