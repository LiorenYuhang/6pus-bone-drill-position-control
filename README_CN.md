# 6PUS-Anti-Vibration

> [English](README.md) | 中文  
> **中文项目名称**：6PUS 并联平台主动抑振研究框架

---

## 1. 项目简介 (Overview)

**6PUS-Anti-Vibration** 是一个基于 ROS 2 的高精度、高确定性、硬实时并联机器人运动控制与多传感器融合感知框架，专为 **6-PUS（移动副-胡克铰-球铰 / Stewart 构型）并联机器人机构**设计，主要面向精密骨科手术操作及高动态环境下的**主动扰动抑制与振动消除**。

系统核心特点包括：
* **6PUS 并联运动学体系**：闭式解析逆运动学求解器（Analytical IK）与带解析雅可比矩阵的 Levenberg-Marquardt 高效数值正运动学求解器（Numerical FK）；
* **主动抑振与连续控制**：三阶加加速度连续整形（$C^2$ Jerk-limited Shaper）、Latest-Value-Wins (LVW) 低延迟缓冲与高确定性串口指令流式下发；
* **双链路解耦 ROS 2 架构**：控制下行链路（50Hz 控制指令）与状态观测上行链路（200Hz 遥测解算）在物理与线程级严格解耦；
* **双 H30 IMU 纳秒/微秒时钟感知**：双路 Yesense H30 惯导模块硬件微秒采样时间戳（0x51/0x52 字段）、软件时钟频偏消除、Session-Start 启动陀螺自适应零偏估计以及李群 $\text{SO}(3)$ 相对姿态双边积分；
* **高可靠串口通信与观测链**：单总线半双工 RS485 921600 8N1 驱动、Batch-Atomicity 全局总线锁、Host `CLOCK_MONOTONIC` 统一时间轴、Fixed-Lag 严格零外推时钟对齐重构与单线程独立 FK 观测节点。

---

## 2. 当前阶段状态 (Project Status)

**当前冻结里程碑**：$\mathbf{J5\ PRE\text{-}ASSEMBLY\ BASELINE\ FROZEN\ v1.0}$  
*(官方 Release Tag: `v1.0-j5-pre-assembly-frozen`)*

装配前软件全链路与独立执行器台架验证已全部封顶冻结：
* **六轴执行器串口通信**：外层 10.0Hz 轮询，单次六轴串行通讯耗时均值 18.18ms，ID1 $\to$ ID6 扫描跨度均值 15.18ms，Batch 零交错；
* **READ_ONLY 0 写入物理审计**：60.0s 真实硬件连续只读监听，严格断言 `hardware_writes = 0`，零硬件侵入；
* **HOME / PARK 独立台架验证**：6 轴独立电缸以 50 step/s 完成一键顶升 HOME（`[1000]*6` 步，50.0mm）与关机收拢 PARK（`[0]*6` 步），稳态定位误差 $\le 2\text{ steps}$；
* **双 IMU 硬件微秒时间戳**：双路 100.00% 具备 0x51 硬件采样时间戳（$\Delta t = 5000\,\mu\text{s}$ 恒定步长），0 丢包；
* **相对姿态与自动零偏估计**：开机 6.0s 自动收集 1200 样本完成 5% 截尾均值零偏收敛（`COMPLETE`），122Hz 实时发布相对四元数，同板 $80.8^\circ/\text{s}$ 大动态共模抑制 RMS $0.2767^\circ$；
* **统一时间轴与 Timed Telemetry**：`TimedJointFeedback` 统一绑定 Host `CLOCK_MONOTONIC` 接收完成时刻；
* **Fixed-Lag 严格零外推状态重构**：1500 次连续查询达成 97.07% SYNCED，微观通信抖动下严格拒绝外推并触发 `WAIT_FUTURE_SAMPLE`；
* **FK 实时观测节点**：单线程独立执行器（`SingleThreadedExecutor`）以 90.6Hz 稳定输出正解，几何残差均值 $< 10^{-11}\text{ m}$；
* **全量单元测试基准**：工作空间核心功能包测试集 `pytest bone_ws/src -v` **162 项测试 100% PASS**（耗时 37.02s）。

---

## 3. 系统架构 (Architecture)

```text
┌───────────────────────────────────────────────────────────────────────┐
│                       上位规划层 / 笛卡尔目标指令                     │
└───────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼  (/pus6/target_cartesian_pose)
┌───────────────────────────────────────────────────────────────────────┐
│                       运动学与控制层 (Kinematics & Control)           │
│  • pus6_kinematics_node : 50Hz 纯几何闭式解析逆解 (Analytical IK)     │
│  • pus6_control_node    : Stateful Jerk-Limited 三阶轨迹平滑整形      │
└───────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼  (/pus6/command_joint_steps, LVW)
┌───────────────────────────────────────────────────────────────────────┐
│                       电机驱动层 (Motor Driver Layer)                 │
│  • pus6_motor_driver_node: 6 级安全硬门禁、Slew 自愈与 Batch 全局锁   │
│  • RS485 半双工单总线 (921600 8N1, 0x29 伴随反馈状态回传)            │
└───────────────────────────────────────────────────────────────────────┘
             │                                              │
             ▼ (电机遥测数据流)                             ▼ (物理硬件)
┌──────────────────────────────────────┐       ┌────────────────────────┐
│      状态观测与主动抑振感知层        │       │         硬件层         │
│  • TimedJointFeedback 发布器         │       │  • 6 轴一体化推杆电缸  │
│  • FixedLagJointSynchronizer 零外推  │       │  • Base H30 IMU 模块   │
│  • Pus6FkNode 单线程正运动学解算     │       │  • Platform H30 IMU    │
│  • h30_relative_attitude_node (SO3)  │       │  • 6PUS 并联机械本体   │
└──────────────────────────────────────┘       └────────────────────────┘
```

---

## 4. 仓库目录结构 (Repository Structure)

```text
6PUS-Anti-Vibration/
├── agent_docs/                     # 项目权威设计、阶段基线与测试报告文档
│   ├── CURRENT_STAGE.md            # 全局阶段演变状态卡 (J5 Frozen 声明)
│   ├── J5_FROZEN_MANIFEST.md       # J5 冻结清单与 J6 路线规划
│   ├── J5_FINAL_TEST_REPORT.md     # 12 项系统级测试验收总报告
│   ├── J5_SOFTWARE_ENVIRONMENT.md  # 操作系统、ROS 2、Python 及 By-ID 串口规格书
│   ├── J5_TUTORIAL_EXECUTION_AUDIT.md # 教程 12 章代码与接口一致性审计报告
│   ├── J5_PRE_ASSEMBLY_FROZEN_v1.0.md # 最终冻结发布归档清单与 SHA256 指纹
│   ├── PRE_ASSEMBLY_DOCUMENT_AND_HARDWARE_ACCEPTANCE.md # 硬件台架验收报告
│   └── 6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md # 机械总装后 9 阶段现场调试规程
│
├── bone_ws/                        # 核心 ROS 2 工作空间源码
│   ├── src/
│   │   ├── pus6_msgs/              # 自定义 ROS 2 接口定义 (TimedJointFeedback 等)
│   │   ├── pus6_kinematics/        # 解析逆解、LM 正解求解器、FixedLag 同步器
│   │   │   └── config/6pus_geometry.yaml # 生产环境唯一权威几何与安全参数真值源
│   │   ├── pus6_motor_driver/      # RS485 串口驱动、六电机控制器与安全前检门禁
│   │   ├── pus6_control/           # 三阶轨迹整形器与关节轨迹控制器
│   │   ├── pus6_bringup/           # 系统级 Launch 启动文件与参数配置
│   │   └── h30_imu/                # C++ Yesense H30 IMU 驱动与相对姿态解算节点
│   └── scripts/                    # 已验证的实机执行脚本与性能基准工具
│
├── hardware/                       # 电机手册、传感器资料、电路原理图与机构 CAD 模型
├── up/                             # 历史独立硬件测试集与传感器独立开发工具链
│
├── README.md                       # GitHub 英文展示首页
├── README_CN.md                    # GitHub 中文展示首页
├── 6PUS_REAL_HARDWARE_OPERATION_GUIDE.md    # 唯一权威实机操作与命令速查指南
└── 6PUS_ROS2_CODE_LEARNING_TUTORIAL.md      # 12 章全栈技术教学与系统解析教程
```

---

## 5. 开发演进路线 (Development Roadmap)

* **J1 -- J4 阶段（已完成并冻结）**：
  * J1: 单总线半双工 RS485 921600 驱动协议与 CRC 校验；
  * J2: 六轴同步通信、0x29 伴随状态反馈与性能基准；
  * J3: 解析逆解与独立电缸点到点 S 曲线 PARK/HOME 闭环就位；
  * J4: 连续笛卡尔轨迹整形、Latest-Value-Wins 缓冲与 UART batch-atomicity 全局锁。
* **J5 阶段（装配前基线冻结 -- 当前版本）**：
  * 纳秒统一时间轴（`CLOCK_MONOTONIC`）、Fixed-Lag 严格零外推多轴同步重构、解析雅可比 LM 正解观测链、双 H30 硬件微秒采样时间戳与开机自动陀螺零偏估计。
* **J6 阶段（机械总装与现场调试 -- 下一步入口）**：
  * 依据 `6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md` 开展 9 阶段现场调试：机械静态检查 $\to$ READ_ONLY 审计 $\to$ 零位映射 $\to$ 极小单轴微动方向确认 $\to$ 机构可达性 $\to$ 小幅协调运动 $\to$ 最终几何参数冻结 $\to$ J5-C2-HW 真实机构正解精度验收 $\to$ 双 IMU 最终空间外参标定（J5-B2）。
* **未来研究规划**：
  * Dual-IMU / 6PUS-FK 误差状态卡尔曼滤波（ESKF）位姿与线加速度融合；
  * 基于扰动观测器（DOB）与加速度反馈的主动高频振动抑制控制；
  * 基于强化学习/自适应控制的骨科机器人微米级阻抗与力控操作。

---

## 6. 权威文档导航入口 (Documentation)

* [英文项目主页 (English README)](README.md)
* [J5 最终冻结发布归档清单 (J5 Frozen Manifest)](agent_docs/J5_PRE_ASSEMBLY_FROZEN_v1.0.md)
* [J5 阶段测试验收总报告 (J5 Final Test Report)](agent_docs/J5_FINAL_TEST_REPORT.md)
* [软件环境与硬件接口规格书 (Software Environment)](agent_docs/J5_SOFTWARE_ENVIRONMENT.md)
* [机械总装后 9 阶段现场调试规程 (Commissioning Plan)](agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md)
* [实机操作与命令速查指南 (Hardware Operation Guide)](6PUS_REAL_HARDWARE_OPERATION_GUIDE.md)
* [ROS 2 项目全栈教学教程 (ROS 2 Code Learning Tutorial)](6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)

---

## 7. 环境准备与全量回归复现

```bash
# 1. 载入 ROS 2 Jazzy 环境
source /opt/ros/jazzy/setup.bash

# 2. 编译并载入工作空间
cd /home/lyapunov/bone_drill/bone_ws
colcon build --symlink-install
source /home/lyapunov/bone_drill/bone_ws/install/setup.bash

# 3. 运行工作空间全量单元测试 (162 项)
pytest /home/lyapunov/bone_drill/bone_ws/src -v
```
