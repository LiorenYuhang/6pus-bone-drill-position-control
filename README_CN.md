# 6PUS 骨科并联机器人防抖控制与感知框架

> [English](README.md) | [中文]
> **面向 6-PUS 并联骨科手术机器人的 ROS 2 实时感知与主动防抖控制系统**

---

## 1. 项目简介

**6PUS-Anti-Vibration** 是面向骨科机器人研究的开源 ROS 2 感知与运动控制框架，适用于 6-PUS（移动副-万向节-球副）并联机构 / Stewart 平台。系统结合笛卡尔运动、轨迹整形与高频多传感器观测，实现主动姿态稳定控制。当前能力、硬件验证与限制见下文。

---

## 2. 当前工程开发状态

### 2.1 主动姿态稳定控制

平台的控制目标是在 Base（基座）运动时，使 Platform（动平台）尽量保持参考姿态。三种方案均已在真实 6PUS 硬件上完成实现与运行：

| 方案 | 方法 |
| --- | --- |
| **Base SO(3) Feedforward** | Base IMU 提供基座姿态，用于计算几何补偿；Platform IMU 观察响应并用于验证，不参与残差反馈。 |
| **Feedforward + Classical P/PD** | 在共同前馈链路上增加 Platform IMU 姿态残差反馈。当前保留形式为 **Roll=P、Pitch/Yaw=PD、Ki=0**。 |
| **Feedforward + First-order LADRC** | 保留共同前馈；三轴独立线性扩张状态观测器（LESO）估计等效集总扰动，用于一阶 LADRC 残差补偿。 |

技术路线为 **几何补偿 → 姿态残差反馈 → 扰动观测残差控制**。**P/PD 和 LADRC 是同一前馈基础上的两种残差控制方案。**

硬件证据涵盖纯前馈/P/PD 对比、LADRC 空载资格验证，以及偏心负载下 Classical P/PD/LADRC 运行。这些证据支持实现与对比表征，不构成三种方案的统一排名，也未证明 LADRC 全面优于经典控制。历史来源见 2.5。

### 2.2 运行与通信可靠性

运行可靠性工程围绕 UART 阻塞、TX timeout、反馈陈旧、启动丢包与 ROS executor 调度饥饿进行优化：

- **三进程架构**：Parent 负责 ROS 指令入口与协调；UART Child 独占电机串口；Feedback Bridge 消费反馈 IPC 并发布 ROS 反馈。**Compatibility Relay 是外部适配器，不属于核心三个进程。**
- **通信隔离**：串口执行、反馈发布与 Parent 回调分离；监测反馈连续性、间隔与龄期；诊断处理与关键通信路径解耦。
- **协调启动**：通过就绪握手协调伺服启动；Parent 使用 `SingleThreadedExecutor`，处理主动订阅负载下已识别的 IPC 租约更新饥饿问题。
- **生命周期处理**：当前路径支持超时检测、受控暂停与停机。正常 Ctrl+C 退出执行返回 HOME → 六轴暂停 → 清理；故障退出执行故障检测 → 暂停 → 清理，跳过返回 HOME。自动恢复与继续控制尚不支持，也未获资格认证。

| 已记录的真实硬件资格验证 | 范围 |
| --- | --- |
| **600s 静态资格验证** | 静态伺服运行与反馈连续性；记录中存在调度重锚事件。 |
| **Classical P/PD：60s 主动会话** | 名义控制器会话，包含预检、参考捕获与 ACTIVE 运行。 |
| **LADRC：60s 主动会话** | 名义控制器会话，包含预检、参考捕获与 ACTIVE 运行。 |

这些验证限定于已记录的条件与窗口。静态资格通过不扩大主动控制验证时长；反馈连续不代表无时序抖动。

### 2.3 运动与感知基础

- 六轴 UART 伺服通信，协调执行器指令与伴随反馈。
- 解析逆运动学（IK）、数值正运动学（FK）与有状态加加速度限制轨迹整形。
- 双 IMU 姿态感知、时间对齐与 SO(3) 相对姿态处理。
- `OPERATING_HOME` 是运行参考，各轴约为 **1000 steps**；主动姿态稳定在启动确认此参考后开始。`STORAGE_PARK` 用于存储/断电状态；返回 HOME 与停机按生命周期管理，不作为机械回零过程。
- HOME 附近局部 command-to-feedback/FK/IMU 跟踪与 HOME 基准单轴包络表征。

已记录的硬件运动、感知与表征证据见 2.5。

### 2.4 验证范围与已知限制

- **工作空间**：HOME 基准单轴包络不等于完整耦合六维工作空间，也不代表任意组合姿态均已认证。
- **精度**：FK/feedback/IMU 跟踪与执行器分辨率不等于外部绝对定位精度或重复定位精度；后两者仍未完成资格验证。
- **对比**：各次人工 Base 扰动不同，不能建立严格控制器排名。
- **控制裕量**：强扰动下存在残差饱和与执行器跟踪滞后；扰动补偿不代表消除所有扰动。
- **稳定性**：LESO 状态仅确认在**已记录的运行窗口内有界**，不构成一般理论稳定性证明。
- **恢复**：当前支持超时检测、暂停与停机，不包含自动恢复与控制接管恢复；调度重锚与通信恢复是不同机制。

### 2.5 演示与技术文档

三种 Demo 模式使用共同生产运行架构。[Demo 操作指南](./docs/j14/J14_DEMO_ENTRYPOINTS.md)记录了启动、ACTIVE 运行、正常归 HOME 停机、暂停确认与资源释放的人工实机确认，并提供操作前提与命令。

| 模式 | 入口脚本 |
| --- | --- |
| **Base SO(3) Feedforward** | [run_j14_demo_feedforward.sh](./bone_ws/scripts/run_j14_demo_feedforward.sh) |
| **Feedforward + Classical P/PD** | [run_j14_demo_feedforward_pd.sh](./bone_ws/scripts/run_j14_demo_feedforward_pd.sh) |
| **Feedforward + First-order LADRC** | [run_j14_demo_feedforward_ladrc.sh](./bone_ws/scripts/run_j14_demo_feedforward_ladrc.sh) |

技术入口：[用户文档](./docs/)、[生产驱动操作说明](./docs/j14/J14_PRODUCTION_DRIVER.md)、[纯前馈/P/PD 对比与指标](./docs/j11/J11_FINAL_CONTROL_COMPARISON.md)、[HOME 附近跟踪证据](./agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md)与[原始证据索引](./results/README.md)。

**历史来源基线（Historical source baselines）**：以下阶段标识用于保留证据来源，不代表独立的当前能力。历史总结保留原测试条件与状态表述，不用于定义当前生产能力。

| 历史来源基线 | 来源 / 版本 |
| --- | --- |
| 运动表征 | [J10](./agent_docs/stages/J10/J10_FINAL_SUMMARY.md) — `v1.0-j10-axis-wise-workspace-qualified` |
| 纯前馈资格验证 | [J11-M1](./agent_docs/stages/J11/J11_M1_FINAL_SUMMARY.md) — `v1.0-j11-m1-feedforward-hardware-qualified` |
| 经典 P/PD 基线 | [J11](./agent_docs/stages/J11/J11_FINAL_SUMMARY.md) — `v1.0-j11-attitude-stabilization-frozen` |
| 经典控制动态/负载表征 | [J12](./agent_docs/stages/J12/J12_FINAL_SUMMARY.md) — `v1.0-j12-classical-dynamic-performance-frozen` |
| LADRC 工程资格验证 | [J13](./agent_docs/stages/J13/J13_FINAL_SUMMARY.md) — `v1.0-j13-ladrc-attitude-residual-control-qualified` |
| 运行可靠性基线（CLOSED / FROZEN） | [J14-A](./agent_docs/stages/J14/J14_FINAL_SUMMARY.md) — `v1.0-j14-a-three-process-runtime-reliability-qualified` |

[内部文档索引](./agent_docs/README.md)保留阶段历史、详细分析与交接入口。

---

## 3. 代码与文档体系结构 (Repository Layout)

项目严格遵行代码、人读技术知识与机读数据资产三者职责边界隔离：

```text
/home/lyapunov/bone_drill/
├── bone_ws/                               <-- [ROS 2 工作空间] 源代码、功能包、测试用例与运行脚本
│   ├── src/
│   │   ├── pus6_msgs/                     # 自定义 ROS 2 消息接口 (TimedJointFeedback, FkStateStamped 等)
│   │   ├── pus6_kinematics/               # 解析逆解、LM 数值正解与 6pus_geometry.yaml 唯一几何真值源
│   │   ├── pus6_motor_driver/             # RS485 串口驱动、50Hz 伺服传输与两级安全看门狗
│   │   ├── pus6_control/                  # 有状态加加速度连续轨迹整形器与控制器
│   │   ├── pus6_bringup/                  # 系统级 Launch 启动编排与参数配置
│   │   └── h30_imu/                       # Yesense H30 IMU C++ 驱动与 SO(3) 相对姿态解算
│   └── scripts/                           # 可复现的实机/仿真运行脚本与验收工具链
│
├── agent_docs/                            <-- [Agent 内部工作文档] stage 历史、决策、调参历史、交接、故障审计
│   ├── stages/                            # 阶段实施方案、验收总结与跨阶段交接规范 (Handoff)
│   ├── reports/                           # 深度实测分析报告 (J10 工作空间分析、J9.1 动平台力臂分析)
│   ├── decisions/                         # 架构决策记录 (ADR) 与硬件准入测试报告
│   ├── frozen/                            # 只读冻结基线清单与代码校验指纹
│   └── archive/                           # 历史遗留操作手册与过时参考资料
│
├── results/                               <-- [机读实验与运行数据] 自动化测试与实机运行生成的数据证据
│   ├── README.md                          # 数据集索引与实验证据导航
│   ├── stewart3_j8_figure6_20260916_.../  # 权威 J8 笛卡尔九阶段实机运动全量时序 (Stewart-3 图 6)
│   ├── j7/                                # 权威 J7 预检与 50Hz 伺服基准 JSON
│   ├── j9_1/                              # 权威 J9.1 双 IMU 姿态与力臂耦合实测数据集
│   └── j10/                               # 权威 J10 六自由度安全包络实测量测数据
│
├── hardware/                              <-- [硬件与几何数据] 电机选型表、电气接线图与 3D 机械结构
├── up/                                    <-- [UART 底层验证基线] 独立多电机串口测试台与基线驱动
├── STM32/                                 <-- [固件] 下位机工程源码
├── docs/                                 <-- [用户文档] 用户主动查看的学习/操作/教程/技术结果
└── AGENTS.md                              <-- [全局 Agent 规则] 操作规范、安全红线与工程不变量
```

> **职责边界原则**：
> - `docs/` = 用户主动需要查看的文档（学习、操作、教程、最终技术结果与汇报入口）。
> - `agent_docs/` = Agent 内部工作文档（stage 历史、决策、调参历史、交接、故障审计），用户正常不需要查看。
> - `results/` = 机器程序运行、测试和实机实验产出的 CSV、遥测 JSON 与运行原始日志。

---

## 4. 推荐阅读与学习路径 (Recommended Reading Order)

对于首次接触本项目的工程师、研究者或智能体，建议按如下次序阅读：

1. **宏观概览与基线**：首先通读本文档 [`README_CN.md`](./README_CN.md)（或 [`README.md`](./README.md)），掌握系统整体架构与当前能力。
2. **ROS 2 全栈技术教程**：研读 [`docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](./docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)，掌握双链路解耦设计、闭式逆解、LM 数值正解及 50Hz 伺服驱动的详细实现。
3. **权威文档检索地图**：参考 [`agent_docs/README.md`](./agent_docs/README.md)，快速定位具体阶段的技术方案、交接规范与硬件准入报告。
4. **物理实机操作与安全**：若需操作物理样机，务必在操作前核对 [`docs/整体学习参考/6PUS_SERVO_OPERATION_GUIDE.md`](./docs/整体学习参考/6PUS_SERVO_OPERATION_GUIDE.md)。
5. **权威实验证据库**：查阅 [`results/README.md`](./results/README.md)，加载并验证物理样机真实飞行数据（如 J8 连续运动数据集）。

---

## 5. 离线公开复现指南 (Offline Public Reproducibility)

第三方研究人员无需拥有真实 6PUS 样机，即可在纯软件离线环境中构建代码、执行全量单元测试与分析真实硬件数据：

### 1. 环境准备
- **操作系统**：Ubuntu 24.04 LTS
- **ROS 2 版本**：Jazzy Jalisco
- **Python 版本**：$\ge 3.10$

```bash
# 克隆仓库
git clone https://github.com/lyapunov/bone_drill.git
cd bone_drill

# 引入 ROS 2 环境变量
source /opt/ros/jazzy/setup.bash
```

### 2. 编译工作空间
```bash
cd bone_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 执行离线单元与集成测试 (Mock 模式)
在 Mock / FakeBus 模式下运行软件测试：
```bash
# 运行 bone_ws 下的软件包测试
colcon test --ctest-args -R test
colcon test-result --all

# 运行 J8 图 6 纯旁路记录器单元测试
pytest scripts/test_j8_figure6_recorder.py -v
```

### 4. 查阅已记录的硬件证据

通过[原始证据索引](./results/README.md)查找已有数据集与分析入口。[控制方案对比](./docs/j11/J11_FINAL_CONTROL_COMPARISON.md)与[HOME 附近跟踪总结](./agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md)给出相关指标定义与证据边界。

---

## 6. 核心安全红线与实机操作守则

- **默认零运动**：所有节点与测试脚本默认启动于 Mock / FakeBus 模式。
- **真机授权原则**：使用 `--real-hardware` 或 `hardware_mode:=active` 驱动物理串口前，必须确认运行参考、硬件急停可靠，并取得操作人员明确确认。主动姿态稳定使用 `OPERATING_HOME`，不使用 `STORAGE_PARK`。
- **写指令零自动重试**：RS485 上的运动写指令严禁发生超时自动重试，通信异常时触发单次暂停（0x1A）并安全停机。
