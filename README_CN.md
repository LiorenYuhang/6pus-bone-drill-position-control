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

**当前冻结里程碑**：$\mathbf{J7\ SIX\text{-}AXIS\ 50Hz\ SERVO\ HARDWARE\ BASELINE\ FROZEN\ v1.0}$  
*(官方 Release Tag: `v1.0-j7-servo-hardware-frozen`)*

六轴 50Hz Servo 伺服运动控制基线与全行程实机验证已全部完成并正式冻结：
* **六轴 50Hz 连续目标流**：50.00Hz 循环频率（周期均值 $20.00\,\text{ms} \pm 0.10\,\text{ms}$），单总线原子化纯写 0x29 批次下发（均值 $3.38\,\text{ms}$，最大 $\le 6.83\,\text{ms}$）；
* **非阻塞轮询状态观测 (Round-Robin)**：每周期轮询 1 轴 0x30 状态（单轴采样率 $\approx 8.33\,\text{Hz}$），反馈延迟均值 $6.84\,\text{ms}$，运行期取消 0x25 模式回读；
* **PARK $\leftrightarrow$ HOME 全行程实机验证**：1000 步（$25.0\,\text{mm}$）五次多项式 S 曲线平滑过渡与驻留保持（$v_{\text{peak}} = 100\,\text{steps/s}$，总计 2326 周期，时长 46.50s）；
* **与 J6 定位模式定量 A/B 实测比对**：
  * 目标刷新率从 10Hz 提高到 50Hz（目标刷新率提高 5 倍）；
  * 全行程最大离散跟踪误差：$19\,\text{步} \to 11\,\text{步}$；
  * 全行程综合 RMS 跟踪误差从 $7.53\,\text{步}$ 降至 $2.89\,\text{步}$（降低 61.6%）；
  * 峰值冲击电流从 $145\,\text{mA}$ 降至 $77\,\text{mA}$（降低 46.9%）；
  * 平均电流从 $34.2\,\text{mA}$ 降至 $29.9\,\text{mA}$；
  * 电流抖动明显降低（标准差降低 $\sim 48\%$），有效抑制电机加减速冲击；
  * 速度阶跃与微停顿事件减少 $>50\%$，实机摄像头观察运动平滑度显著优于原定位模式；
  * PARK 最终回位保持高精度（误差 $\le 1\,\text{步}$，各轴回位离散度 0 步）；
* **安全机制与急停**：0 故障（Fault=0）、0 超时（Timeout=0）、全六轴 0x1A 干净停机确认（Stop ACK=True）；
* **全量单元测试基准**：工作空间核心功能包测试集 `pytest bone_ws/src` **304 项测试 100% PASS**（耗时 43.75s）。

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
│  • pus6_control_node    : 有状态三阶加加速度连续整形控制器            │
└───────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼  (/pus6/command_joint_steps, LVW)
┌───────────────────────────────────────────────────────────────────────┐
│                       电机驱动层 (Motor Driver Layer)                 │
│  • J7 50Hz 六轴 Servo Transport（单总线纯写批次下发）                 │
│  • J7 轮询反馈调度器（每周期 1 轴 0x30 状态非阻塞查询）              │
│  • pus6_motor_driver_node: 6 级安全硬门禁与 Batch 总线锁              │
│  • RS485 半双工单总线 (921600 8N1)                                    │
└───────────────────────────────────────────────────────────────────────┘
             │                                              │
             ▼ (电机遥测数据流)                             ▼ (物理硬件)
┌──────────────────────────────────────┐       ┌────────────────────────┐
│        观测与感知层 (Sensing)        │       │         硬件层         │
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
│   ├── decisions/                  # 决策记录、时序预算与阶段状态卡
│   ├── frozen/                     # 历史归档冻结基线清单与报告
│   └── stages/                     # 分阶段调试规程
│
├── bone_ws/                        # 核心 ROS 2 工作空间源码
│   ├── src/
│   │   ├── pus6_msgs/              # 自定义 ROS 2 接口定义 (TimedJointFeedback 等)
│   │   ├── pus6_kinematics/        # 解析逆解、LM 正解求解器、FixedLag 同步器
│   │   │   └── config/6pus_geometry.yaml # 生产环境唯一权威几何与安全参数真值源
│   │   ├── pus6_motor_driver/      # RS485 串口驱动、J7 50Hz Servo 传输与反馈调度器
│   │   ├── pus6_control/           # 三阶轨迹整形器与关节轨迹控制器
│   │   ├── pus6_bringup/           # 系统级 Launch 启动文件与参数配置
│   │   └── h30_imu/                # C++ Yesense H30 IMU 驱动与相对姿态解算节点
│   └── scripts/                    # 已验证的实机执行脚本与性能基准工具
│
├── hardware/                       # 硬件手册、传感器资料、电路原理图与机构 CAD 模型
├── up/                             # 历史独立硬件测试集与传感器独立开发工具链
│
├── README.md                       # GitHub 英文展示首页
├── README_CN.md                    # GitHub 中文展示首页
├── 6PUS_SERVO_OPERATION_GUIDE.md    # 唯一权威实机操作与命令速查指南
└── 6PUS_ROS2_CODE_LEARNING_TUTORIAL.md      # 12 章全栈技术教学与系统解析教程
```

---

## 5. 开发演进路线 (Development Roadmap)

* **J1 -- J4 阶段（已完成并冻结）**：
  * J1: 单总线半双工 RS485 921600 驱动协议与 CRC 校验；
  * J2: 六轴同步通信、0x29 伴随状态反馈与性能基准；
  * J3: 解析逆解与独立电缸点到点 S 曲线 PARK/HOME 闭环就位；
  * J4: 连续笛卡尔轨迹整形、Latest-Value-Wins 缓冲与 UART batch-atomicity 全局锁。
* **J5 阶段（装配前基线已冻结）**：
  * 纳秒统一时间轴（`CLOCK_MONOTONIC`）、Fixed-Lag 严格零外推多轴同步重构、解析雅可比 LM 正解观测链、双 H30 硬件微秒采样时间戳与开机自动陀螺零偏估计。
* **J6 阶段（总装调试基线已冻结）**：
  * 定位模式（mode=0, 10Hz, speed=50）六轴独立 HOME (`[1000]*6`) / PARK (`[0]*6`) 启动与收拢调试，通过连续 10 循环实机测试。
* **J7 阶段（六轴 50Hz 伺服模式实机基线已冻结 -- 当前版本）**：
  * 高频 50Hz 六轴 Servo 模式（mode=1），非阻塞轮询反馈（$\approx 8.33\,\text{Hz}$/轴），全行程 PARK $\leftrightarrow$ HOME S 曲线实测，A/B 对比证实 RMS 跟踪误差下降 61.6%、峰值冲击电流下降 46.9%、运动平滑性显著提升。
* **未来研究规划**：
  * Dual-IMU / 6PUS-FK 误差状态卡尔曼滤波（ESKF）位姿与线加速度融合；
  * 基于扰动观测器（DOB）与加速度反馈的主动高频振动抑制控制；
  * 基于强化学习/自适应控制的骨科机器人微米级阻抗与力控操作。

---

## 6. 权威文档导航入口 (Documentation)

* [英文项目主页 (English README)](README.md)
* [J5 最终冻结发布归档清单 (J5 Frozen Manifest)](agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md)
* [J5 阶段测试验收总报告 (J5 Final Test Report)](agent_docs/frozen/J5_FINAL_TEST_REPORT.md)
* [软件环境与硬件接口规格书 (Software Environment)](agent_docs/frozen/J5_SOFTWARE_ENVIRONMENT.md)
* [机械总装后 9 阶段现场调试规程 (Commissioning Plan)](agent_docs/stages/J6/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md)
* [实机操作与命令速查指南 (Hardware Operation Guide)](6PUS_SERVO_OPERATION_GUIDE.md)
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

# 3. 运行工作空间全量单元测试 (304 项)
pytest /home/lyapunov/bone_drill/bone_ws/src -v
```
