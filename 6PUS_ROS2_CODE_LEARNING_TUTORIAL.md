# 6PUS 并联手术机器人 ROS 2 项目全栈源码教学与系统解析教程

> **适用对象**：希望通过真实 6PUS 并联骨科手术机器人项目系统掌握 ROS 2 架构设计、多轴电机高频伺服通信驱动、并联机构解析逆解与数值正解、在线轨迹平滑与安全控制全栈技术的工程师与研究生。  
> **当前冻结基线**：$\mathbf{J8\ ACTIVE\ SERVO\ HARDWARE\ QUALIFIED\ v1.0}$ (`v1.0-j8-active-servo-hardware-qualified`)。  
> **测试回归语境说明**：ROS 2 工作空间核心功能包全量单元与集成测试为 **349 项 100% PASS**（执行 `pytest src/h30_imu src/pus6_bringup src/pus6_control src/pus6_kinematics src/pus6_motor_driver`，耗时约 54s）。  
> **物理装配与实机验证状态**：完整 6PUS 并联机器人已完成机械总装与现场电气调试，通过 J6 定位调试基线、J7 50Hz 伺服驱动基线，并在 J8 阶段完成连续笛卡尔伺服控制闭环与 9 阶段实机展示轨迹全量硬件准入考核。

---

## 目录

- [Part 0: J8 最终系统全景与架构设计哲学 (J8 System Architecture & Dual-Pipeline Philosophy)](#part-0-j8-最终系统全景与架构设计哲学-j8-system-architecture--dual-pipeline-philosophy)
  - [0.1 6PUS 机器人系统定位与控制/观测双链路解耦哲学](#01-6pus-机器人系统定位与控制观测双链路解耦哲学)
  - [0.2 工作空间 (`bone_ws`) 6 大功能包全景与依赖关系](#02-工作空间-bone_ws-6-大功能包全景与依赖关系)
  - [0.3 全局唯一参数真值源：`6pus_geometry.yaml` 与类型化配置](#03-全局唯一参数真值源-6pus_geometryyaml-与类型化配置)
  - [0.4 ROS 2 核心概念与现代机器人全栈开发映射](#04-ros-2-核心概念与现代机器人全栈开发映射)
- [Part I: 从 J1 到 J8 的实机工程演进教学 (Engineering Evolution: J1 → J8)](#part-i-从-j1-到-j8-的实机工程演进教学-engineering-evolution-j1--j8)
  - [第 1 章 (J1): RS485 半双工通信底层与自定义二进制协议设计](#第-1-章-j1-rs485-半双工通信底层与自定义二进制协议设计)
  - [第 2 章 (J2): 六轴同步通信、批次原子性与 ROS 2 电机驱动节点](#第-2-章-j2-六轴同步通信批次原子性与-ros-2-电机驱动节点)
  - [第 3 章 (J3): 并联机构闭式解析逆运动学 (IK) 与绝对位置闭环](#第-3-章-j3-并联机构闭式解析逆运动学-ik-与绝对位置闭环)
  - [第 4 章 (J4): 连续空间运动、三阶 Jerk 轨迹整形与 QoS/LVW 架构](#第-4-章-j4-连续空间运动三阶-jerk-轨迹整形与-qoslvw-架构)
  - [第 5 章 (J5): 纳秒统一时间轴、Fixed-Lag 状态重构与 LM 数值正解 (FK)](#第-5-章-j5-纳秒统一时间轴fixed-lag-状态重构与-lm-数值正解-fk)
  - [第 6 章 (J6): 现场总装调试、零位映射与 HOME/PARK 定位模式基线](#第-6-章-j6-现场总装调试零位映射与-homepark-定位模式基线)
  - [第 7 章 (J7): 50Hz 伺服模式演进、五次 S 曲线与运动平滑度量化飞跃](#第-7-章-j7-50hz-伺服模式演进五次-s-曲线与运动平滑度量化飞跃)
  - [第 8 章 (J8): Design-B 伴随反馈、两级看门狗、集约化启动契约与全自由度实机准入](#第-8-章-j8-design-b-伴随反馈两级看门狗集约化启动契约与全自由度实机准入)
- [Part II: J8 当前生产源码全模块精读与参考 (Source Code Reference by Module)](#part-ii-j8-当前生产源码全模块精读与参考-source-code-reference-by-module)
  - [2.1 `pus6_msgs`: 强类型数据契约与单调时钟协议](#21-pus6_msgs-强类型数据契约与单调时钟协议)
  - [2.2 `pus6_kinematics`: 逆解、正解、几何加载器与状态同步器](#22-pus6_kinematics-逆解正解几何加载器与状态同步器)
  - [2.3 `pus6_control`: 有状态轨迹整形器与控制器节点](#23-pus6_control-有状态轨迹整形器与控制器节点)
  - [2.4 `pus6_motor_driver`: 驱动协议、传输层、安全分类器与电机驱动节点](#24-pus6_motor_driver-驱动协议传输层安全分类器与电机驱动节点)
  - [2.5 `pus6_bringup`: 系统级 Launch 编排与参数配置体系](#25-pus6_bringup-系统级-launch-编排与参数配置体系)
  - [2.6 `h30_imu`: 双 H30 惯导微秒硬件时间戳与李群姿态解算](#26-h30_imu-双-h30-惯导微秒硬件时间戳与李群姿态解算)
- [Part III: 【全链路实战透视】追踪一帧笛卡尔控制指令的完整生命周期 (End-to-End Deep Trace)](#part-iii-全链路实战透视追踪一帧笛卡尔控制指令的完整生命周期-end-to-end-deep-trace)
  - [3.1 下行控制链：Cartesian Target → IK → Shaper → Driver Preflight → UART 0x29](#31-下行控制链cartesian-target--ik--shaper--driver-preflight--uart-0x29)
  - [3.2 物理执行与伴随反馈：RS485 总线交互与 0x29 StatusFeedback 提取](#32-物理执行与伴随反馈rs485-总线交互与-0x29-statusfeedback-提取)
  - [3.3 上行观测链：TimedJointFeedback → FixedLag (tau=160ms) → LM FK → FkStateStamped](#33-上行观测链timedjointfeedback--fixedlag-tau160ms--lm-fk--fkstatestamped)
- [Part IV: 实机调试、故障复盘与测试驱动开发 (Hardware Debugging, Post-Mortems & TDD)](#part-iv-实机调试故障复盘与测试驱动开发-hardware-debugging-post-mortems--tdd)
  - [4.1 标志性工业级工程踩坑与根因深度复盘 (9 大 Post-Mortem 案例)](#41-标志性工业级工程踩坑与根因深度复盘-9-大-post-mortem-案例)
  - [4.2 为什么必须独占 SingleThreadedExecutor：A~F 分层消融实测与复盘](#42-为什么必须独占-singlethreadedexecutoraf-分层消融实测与复盘)
  - [4.3 测试驱动开发 (TDD) 体系与 349 项回归测试规范](#43-测试驱动开发-tdd-体系与-349-项回归测试规范)
  - [4.4 软件代数自洽 vs 物理机械精度的认知红线](#44-软件代数自洽-vs-物理机械精度的认知红线)
- [附录 (Appendix)](#附录-appendix)
  - [附录 A: 全系统 ROS 2 节点、话题、服务与消息类型全景速查表](#附录-a-全系统-ros-2-节点话题服务与消息类型全景速查表)
  - [附录 B: J1 ~ J8 核心特性与技术指标演进对比矩阵](#附录-b-j1--j8-核心特性与技术指标演进对比矩阵)
  - [附录 C: 常用 ROS 2 / 驱动调试与硬件实机操作 CLI 指令速查](#附录-c-常用-ros-2--驱动调试与硬件实机操作-cli-指令速查)

---

# Part 0: J8 最终系统全景与架构设计哲学 (J8 System Architecture & Dual-Pipeline Philosophy)

## 0.1 6PUS 机器人系统定位与控制/观测双链路解耦哲学

并联骨科手术机器人对控制系统的**确定性 (Determinism)**、**安全性 (Safety)** 与 **状态可观测性 (Observability)** 有着极其严苛的工业级要求：

1. **控制下行链路 (Control Pipeline)**：必须保证从 50Hz 笛卡尔空间参考轨迹到 50Hz 物理电机串口指令的流式下发具备极低的调度抖动、严格的 $C^2$ 加速度连续性以及绝对安全的运动学包络硬门禁；
2. **状态观测链路 (Observation Pipeline)**：必须将 6 轴异步串行扫描到达的电机遥测重构对齐到统一纳秒时间轴上，并以 50Hz/200Hz 独立解算平台当前末端位姿，提供给上层状态监控与未来多传感器融合。

如果将逆解计算、轨迹平滑、串口通信、状态同步和正解解算混在同一个进程或同一个控制循环中，串口通信的长尾波动（如 Linux 调度与硬件往返抖动）会直接破坏规划器的定时周期，而正解迭代的不确定耗时也会污染控制下发的确定性。因此，本项目采用**控制链路与观测链路严格解耦、并行运行的双链路架构**。

```text
                                  6PUS J8 系统双数据流全景架构
                                  
  [ 上位规划流 / 笛卡尔目标 (50Hz) ]
             │
             ▼  (/pus6/target_cartesian_pose)
  ┌────────────────────────┐
  │  pus6_kinematics_node  │ ── 50Hz 纯几何闭式解析逆解 (Analytical IK)
  └────────────────────────┘
             │
             ▼  (/pus6/desired_joint_deltas)
  ┌────────────────────────┐
  │   pus6_control_node    │ ── 50Hz StatefulJerkLimitedShaper (三阶加加速度连续整形)
  └────────────────────────┘
             │
             ▼  (/pus6/command_joint_steps, QoS: BestEffort, KeepLast(1))
  ┌────────────────────────┐
  │ pus6_motor_driver_node │ ── 50Hz Design-B J7/J8 ServoTransport (0x29 批次下发 + 伴随反馈提取)
  │ (两级看门狗 + 启动分类)│    + Vector Slew Recovery 自愈 + 6 级安全硬门禁
  └────────────────────────┘
             │
             ▼  (RS485 / UART 半双工单总线 921600 8N1, 6x 0x29 目标写入帧)
  ========================= [ 6PUS 物理机构 (ID1 ~ ID6 一体化推杆电缸) ] =========================
             │
             ▼  (0x29 伴随返回 StatusFeedback 帧，6 轴完整性 100% 提取)
  ┌────────────────────────┐
  │ TimedJointFeedback Pub │ ── 组装 TimedJointFeedback 消息 (打上 Host CLOCK_MONOTONIC 接收时间戳)
  └────────────────────────┘
             │
             ▼  (/pus6/joint_feedback/timed)
  ┌────────────────────────┐
  │      pus6_fk_node      │ ── FixedLagJointSynchronizer (严格零外推状态重建, tau = 160ms)
  │ (独立 SingleThreaded)  │    + Analytical Jacobian LM 数值正解 (Warm-Start + SO(3) 测地线门禁)
  └────────────────────────┘
             │
             ├───────────────────────────────┐
             ▼                               ▼
    /pus6/fk/state (FkStateStamped)     /pus6/fk/pose (geometry_msgs/PoseStamped)
```

> [!IMPORTANT]
> **控制链与 FK 观测链的并行解耦原则**：
> * **控制链** 是单向向下驱动物理电机的指令流：`Cartesian Target → IK → Jerk Filter → Driver Preflight → UART 0x29 Write`；
> * **FK 观测链** 是单向向上重构平台姿态的遥测流：`0x29 StatusFeedback → Timed Feedback → Fixed-Lag → Numerical FK → FkStateStamped`；
> * **FK 观测链绝不作为 J4/J8 控制器的反馈输入，也不进入下行控制环路**。两者完全独立解耦，彻底杜绝了正解数值迭代波动对控制周期的干扰。

---

## 0.2 工作空间 (`bone_ws`) 6 大功能包全景与依赖关系

工作空间位于 `/home/lyapunov/bone_drill/bone_ws`，由 6 个专职功能包协同构成：

```text
bone_ws/
├── src/
│   ├── pus6_msgs/          # 自定义强类型 ROS 2 消息接口包 (时间合同与正解状态)
│   ├── pus6_motor_driver/  # 电机二进制通信协议、底层驱动节点与硬件安全包
│   ├── pus6_kinematics/    # 6PUS 权威几何配置、解析逆运动学与实时数值正运动学包
│   ├── pus6_control/       # 连续轨迹三阶整形滤波、外环控制与状态机包
│   ├── pus6_bringup/       # 统一 Launch 编排、参数配置与系统集成包
│   └── h30_imu/            # 双 H30 IMU 驱动、微秒时间戳与相对姿态李群估计包
├── build/                  # 编译构建中间文件
├── install/                # 节点可执行文件与环境引导脚本 (setup.bash)
├── log/                    # 运行时生成的实验遥测与调试日志
└── scripts/                # 实机/仿真运行脚本与离线验收工具链
```

#### 功能包职责与依赖矩阵：

| 功能包名称 | 核心职责 | 输入 (Sub / Ingress) | 输出 (Pub / Egress) | 硬件依赖 |
| :--- | :--- | :--- | :--- | :---: |
| **`pus6_msgs`** | 统一数据合同：定义带纳秒单调时钟的高精度遥测与正解消息 | 无 (纯接口定义) | `TimedJointFeedback`, `TimedJointCommand`, `FkStateStamped` | 无 |
| **`pus6_motor_driver`** | 独占物理串口通信，维护 Latest-Value-Wins 缓冲，执行两级看门狗与安全分类 | `/pus6/command_joint_steps` | `/pus6/joint_states`, `/pus6/six_axis_telemetry`, `/pus6/joint_feedback/timed` | 依赖 UART 硬件 |
| **`pus6_kinematics`** | 提供唯一几何真值源；运行解析逆解节点；运行独立单线程数值正解节点 | `/pus6/target_cartesian_pose`, `/pus6/joint_feedback/timed` | `/pus6/desired_joint_deltas`, `/pus6/fk/state`, `/pus6/fk/pose` | 无 (纯数学与 ROS) |
| **`pus6_control`** | 50Hz 三阶状态滤波整形，保证速度/加速度/加加速度饱和限幅与 $C^2$ 连续 | `/pus6/desired_joint_deltas`, `/pus6/joint_states` | `/pus6/command_joint_steps`, `/pus6/control_diagnostics` | 无 (纯算法) |
| **`pus6_bringup`** | 加载系统参数 YAML，协同编排启动各节点与 RViz 可视化 | Launch 启动参数 | 启动系统节点图 (Node Graph) | 编排层 |
| **`h30_imu`** | 双 H30 姿态传感器接口，完成时钟频偏消除与共模抑制相对姿态解算 | 串口 IMU 二进制流 | `/imu/base/data`, `/imu/platform/data`, 相对姿态四元数 | 依赖 IMU 硬件 |

---

## 0.3 全局唯一参数真值源：`6pus_geometry.yaml` 与类型化配置

在工业级机器人系统中，**严禁在各个功能包或源码文件中散落硬编码几何参数与换算系数**。本项目确立了全局唯一参数真值源 (Single Source of Truth)：

* **权威配置文件**：[`pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)
* **类型化加载器**：[`pus6_kinematics/geometry_config.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/geometry_config.py) 中的 `load_geometry_config()`

```text
                               6pus_geometry.yaml (权威唯一来源)
                                              │
                                              ▼
                             geometry_config.load_geometry_config()
                                              │
         ┌───────────────────┬────────────────┴───────────────────┬───────────────────┐
         ▼                   ▼                                    ▼                   ▼
 pus6_kinematics        pus6_control                      pus6_motor_driver      J7/J8 Scripts
(analytical_6pus.py) (joint_trajectory_controller.py)       (safety.py)        & Offline Tools
 • 静/动平台半径       • 步数转换比例 (40000 step/m)       • 绝对硬件限位 [0,2000] • 默认串口与波特率
 • 连杆长与安装角     • 工作零位 OPERATING_HOME           • 软件限位 [0,1800]     • 停放位与零位定义
 • 标称高度 z_nom     • 停放位 STORAGE_PARK               • 正常运动包络 [200,1800] • 启动测量带 [-5, 5]
```

---

## 0.4 ROS 2 核心概念与现代机器人全栈开发映射

| ROS 2 核心概念 | 通俗定义 | 本项目中的具体实现与源码映射 |
| :--- | :--- | :--- |
| **Package (功能包)** | 软件复用与编译的基本单元 | `pus6_msgs`, `pus6_motor_driver`, `pus6_kinematics`, `pus6_control`, `pus6_bringup`, `h30_imu` |
| **Node (节点)** | 独立执行的计算任务进程 | `Pus6MotorDriverNode`, `Pus6KinematicsNode`, `Pus6ControlNode`, `Pus6FkNode` |
| **Topic (话题)** | 单向发布-订阅数据管道 | `/pus6/command_joint_steps` (下行控制), `/pus6/joint_feedback/timed` (上行观测), `/pus6/fk/state` (正解状态) |
| **Message (消息)** | 强类型数据载体 | `TimedJointFeedback`, `TimedJointCommand`, `FkStateStamped`, `JointState`, `PoseStamped` |
| **Service (服务)** | 双向同步/异步请求响应 | `/pus6/startup_home` (就位), `/pus6/controlled_park` (受控收拢), `/pus6/pause_all` (急停受控停止) |
| **Timer (定时器)** | 周期性回调触发器 | 50Hz 控制主循环 (`pus6_control_node`), 50Hz 伺服下发线程 (`motor_driver_node`), 50Hz/200Hz 正解查询 (`fk_node`) |
| **Callback Group** | 执行器内的回调隔离机制 | 驱动节点内划分 `sub_cb_group`, `worker_cb_group`, `poll_cb_group` 三大互斥组 |
| **QoS (服务质量)** | 通信时效与队列控制策略 | 下行控制指令采用 **Latest-Value-Wins (KeepLast(1), BestEffort, Volatile)** 彻底杜绝历史堆积 |

---

# Part I: 从 J1 到 J8 的实机工程演进教学 (Engineering Evolution: J1 → J8)

## 第 1 章 (J1): RS485 半双工通信底层与自定义二进制协议设计

### 1.1 学习目标与物理背景
掌握工业级半双工 RS485 总线下自定义二进制通信协议的设计与实现。理解 921600 bps 波特率下微秒级时序预算与累加和校验的数学实现。

* **主要源码**：[`pus6_motor_driver/pus6_motor_driver/protocol.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py)
* **ROS 2 概念**：纯 Python 硬件驱动抽象，与上层 ROS 中间件解耦。
* **控制/机器人学概念**：寄存器映射模型、半双工收发转换时延、累加和校验算法。

### 1.2 二进制通信帧格式详解

```text
+----------------+----------------+----------------+----------------+----------------+----------------+----------------+
|  Byte 0 ~ 1    |     Byte 2     |     Byte 3     |     Byte 4     |  Byte 5 ~ 6    |  Byte 7 ~ N-2  |    Byte N-1    |
|   帧头 (Header) | 数据长度 Length | 目标 ID (ID)   | 命令字 (CMD)   | 寄存器地址 Addr | 负载 (Payload) | 校验和 (CRC)   |
+----------------+----------------+----------------+----------------+----------------+----------------+----------------+
| 0x55 0xAA (下发)|  3 + len(data) |   0x01 ~ 0x06  |  0x30/0x31/0x32|  小端 LE16      |  小端 16位数据  |  累加和低8位   |
| 0xAA 0x55 (回传)|      0x0F      |   0x01 ~ 0x06  |  0x30 / 0x32   |  0x0000/0x0029 |  12字节状态数据 |  累加和低8位   |
+----------------+----------------+----------------+----------------+----------------+----------------+----------------+
```

1. **帧头 (Frame Header)**：
   * 主机发送命令：`0x55, 0xAA` (`CMD_FRAME_HEADER`)
   * 电机回传应答：`0xAA, 0x55` (`RSP_FRAME_HEADER`)
2. **长度字段 (Length)**：值为 `3 + len(payload)`（包含 CMD 1字节 + Addr 2字节 + 数据体长度）。
3. **累加和校验算法 (`calculate_checksum`)**：
   $$	ext{Checksum} = \left( \sum_{k=2}^{N-2} 	ext{Byte}[k] ight) \ \& \ 	ext{0xFF}$$

### 1.3 核心寄存器表与语义

| 寄存器宏定义 | 地址 (16位小端) | 读/写 | 物理含义 | 数据格式与单位 |
| :--- | :---: | :---: | :--- | :--- |
| `ID_ADDR` | `0x0016` | R/W | 电机硬件总线 ID | `uint16`, 范围 1 ~ 6 |
| `SAVE_PARAMS` | `0x001C` | W | 保存参数至 EEPROM | 写入 1 触发保存 |
| `SPEED_LIMIT` | `0x0028` | R/W | 内部梯形限速寄存器 | `uint16`, 对应 step/s (默认 240) |
| **`TARGET_POS`** | `0x0029` | R/W | **目标位置写入寄存器** | `int16`, 对应整步 raw step (0~2000) |
| **`ACTUAL_POS`** | `0x002A` | R | **实际位置反馈寄存器** | `int16`, 当前编码器整步值 |
| `CURRENT_VAL` | `0x002B` | R | 实际工作电流 | `uint16`, 单位毫安 (mA) |
| `ACTUAL_FORCE`| `0x002C` | R | 轴向力传感器读数 | `int16`, 单位克 (g) |
| `TEMPERATURE` | `0x002E` | R | 电机驱动板温度 | `int8`, 单位摄氏度 (°C) |
| `ERROR_CODE`  | `0x002F` | R | 硬件故障代码 | `uint8`, 0 表示正常，非0为故障 |
| `PAUSE_BRAKE` | `0x001A` | W | 受控停止锁定 | 写入 1 立即受控停机 |

---

## 第 2 章 (J2): 六轴同步通信、批次原子性与 ROS 2 电机驱动节点

### 2.1 学习目标与物理现实
掌握单总线半双工拓扑下的“串行非原子现实 (Serial Non-Atomic Reality)”。理解为什么 6 台电机依次问答会导致时间分散，以及如何在 ROS 2 驱动层通过 Batch-Atomicity 全局锁杜绝通信交错。

* **主要源码**：[`pus6_motor_driver/pus6_motor_driver/j7_real_uart_adapter.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_real_uart_adapter.py), [`pus6_motor_driver/pus6_motor_driver/motor_driver_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py)
* **ROS 2 概念**：`rclpy.node.Node`, `sensor_msgs/msg/JointState`, 回调组并发隔离。
* **控制/机器人学概念**：多轴串行扫描时间跨度 (Temporal Dispersion)、总线互斥锁机制。

### 2.2 串行非原子现实与 Batch Atomicity 全局锁

由于单总线串行依次下发（ID1 $	o$ ID2 $	o \dots 	o$ ID6），单次 6 轴写入**不是同时生效的硬件锁存事务**。当 Motor 6 收到指令并应答时，Motor 1 已经先运行了约 $18.18	ext{ ms}$。同理，6 轴反馈数据也存在约 $18.18	ext{ ms}$ 的时间相位差。

在多线程或带后台轮询的架构中，必须严防后台遥测轮询 (Poll) 帧插入到 6 轴控制批次中间：
* **加固方案**：在驱动传输层中引入 6 轴批次全局锁（`with self._bus_lock_ctx:`），并在 Active 控制模式下抑制独立 Poll 轮询；
* **回归实证**：实机连续动态回归证明批次重叠数为 **0 / 392 (100.00% 批次原子性无交错)**。

---

## 第 3 章 (J3): 并联机构闭式解析逆运动学 (IK) 与绝对位置闭环

### 3.1 学习目标与数学优势
掌握 6-PUS 并联机构逆运动学的闭式几何推导。理解为什么并联机构逆解具有唯一闭式解析解，并掌握笛卡尔空间位姿到 6 轴整步的精确映射。

* **主要源码**：[`pus6_kinematics/pus6_kinematics/analytical_6pus.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py), [`pus6_kinematics/config/6pus_geometry.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml)
* **ROS 2 概念**：`geometry_msgs/msg/PoseStamped`, YAML 参数类型化解析。
* **控制/机器人学概念**：定长连杆球铰约束、闭式二次方程解析求解、工作空间可达性检查、`OPERATING_HOME` 零位映射。

```text
                     动平台 (Platform, Radius d = 27.5mm)
                       /   |   |   |   |                         /    |   |   |   |    \  (6 根定长碳纤维连杆 e = 84.0mm)
                     /     |   |   |   |                        S1      S2  S3  S4  S5     S6 (上端球铰 Spherical Joints, Bi)
                    │       │   │   │   │      │
                   U1      U2  U3  U4  U5     U6 (下端万向铰 Universal Joints, Ai)
                    │       │   │   │   │      │
                   q1      q2  q3  q4  q5     q6 (主动移动副 Prismatic Sliders, 沿 Z 轴升降)
                    │       │   │   │   │      │
                  ================================ 静基座 (Base, Radius c = 40.0mm)
```

### 3.2 闭式逆解数学推导

1. **输入量**：动平台中心在基座坐标系下的位姿 $\mathbf{P} = [x, y, z]^	op$，姿态角 $[lpha, eta, \gamma]^	op$（Roll, Pitch, Yaw，ZYX 欧拉角约定）；
2. **输出量**：6 轴滑块绝对高度向量 $\mathbf{q} = [q_1, q_2, q_3, q_4, q_5, q_6]^	op$。

动平台上第 $i$ 个球铰点在基座坐标系下的空间坐标为：
$$\mathbf{B}_i = \mathbf{P} + \mathbf{R}_{	ext{ZYX}}(lpha, eta, \gamma) \mathbf{b}_i$$
基座滑块铰链坐标为 $\mathbf{A}_i = [a_{ix}, a_{iy}, q_i]^	op$。根据连杆定长约束 $\|\mathbf{B}_i - \mathbf{A}_i\|^2 = L_0^2$：
$$(B_{ix} - a_{ix})^2 + (B_{iy} - a_{iy})^2 + (B_{iz} - q_i)^2 = L_0^2$$

取滑块位于动平台下方的物理工作分支，得到 **唯一闭式解析逆运动学解**：
$$q_i = B_{iz} - \sqrt{L_0^2 - (B_{ix} - a_{ix})^2 - (B_{iy} - a_{iy})^2}$$

### 3.3 步数比例与工作零位转换

* **标称高度**：$z_{	ext{nom}} pprox 0.1326123	ext{ m}$ ($132.6123	ext{ mm}$)；
* **工作零位 (`OPERATING_HOME`)**：$q_{	ext{raw}} = [1000, 1000, 1000, 1000, 1000, 1000]	ext{ step}$，对应推杆中位伸出量 **$50.0	ext{ mm}$**（行程范围 $25.0\sim 75.0	ext{ mm}$ 的中心点），相对控制坐标为 $[0, 0, 0, 0, 0, 0]	ext{ m}$；
* **停放位 (`STORAGE_PARK`)**：$q_{	ext{raw}} = [0, 0, 0, 0, 0, 0]	ext{ step}$，推杆全缩回（最短位置 $25.0	ext{ mm}$）；
* **整步转换公式**：
  $$q_{	ext{raw}, i} = 1000 + 	ext{round}\left((q_i - q_{	ext{nom}, i}) 	imes 	ext{steps\_per\_meter}ight) \quad (	ext{其中 } 	ext{steps\_per\_meter} = 40000.0)$$

---

## 第 4 章 (J4): 连续空间运动、三阶 Jerk 轨迹整形与 QoS/LVW 架构

### 4.1 学习目标与控制平滑性
理解空间多自由度连续控制中加加速度连续性对机构减振的决定性意义。掌握三阶 Jerk-Limited 状态滤波器的微分积分实现与 Single-Slot Latest-Value-Wins (LVW) 架构。

* **主要源码**：[`pus6_control/pus6_control/joint_trajectory_controller.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py), [`pus6_control/pus6_control/control_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py)
* **ROS 2 概念**：QoS (BestEffort, KeepLast(1)), Single-Slot 缓冲, Timer 循环。
* **控制/机器人学概念**：$C^2$ 加速度连续性、三阶实极点状态微分方程持续积分、速度/加速度/加加速度物理饱和限幅。

### 4.2 三阶 Jerk-Limited 状态整形微分方程

滤波器配置为三阶实极点 $s = -\omega_n$（带宽 $\omega_n = 20.0	ext{ rad/s}$）：
$$H(s) = rac{\omega_n^3}{(s + \omega_n)^3} = rac{\omega_n^3}{s^3 + 3\omega_n s^2 + 3\omega_n^2 s + \omega_n^3}$$

定义连续状态向量 $\mathbf{x} = [q, \dot{q}, \ddot{q}]^	op$。在每个控制周期 $dt$：
$$\dddot{q}(t) = \omega_n^3 \left( q_{	ext{target}}(t) - q(t) ight) - 3\omega_n^2 \dot{q}(t) - 3\omega_n \ddot{q}(t)$$

在积分推进过程中强制施加三级物理饱和：
* **速度限幅**：$|\dot{q}| \le 0.005	ext{ m/s} \ (200	ext{ step/s})$
* **加速度限幅**：$|\ddot{q}| \le 0.050	ext{ m/s}^2 \ (2000	ext{ step/s}^2)$
* **加加速度限幅**：$|\dddot{q}| \le 0.500	ext{ m/s}^3 \ (20000	ext{ step/s}^3)$

> [!IMPORTANT]
> **$C^2$ 连续性铁律**：当外部目标突变时，滤波器**绝不清零导数**（$\dot{q}$ 和 $\ddot{q}$ 保持连续积分），从而在数学上保证了输出位置的二阶可导，受力完全平滑无冲击。

---

## 第 5 章 (J5): 纳秒统一时间轴、Fixed-Lag 状态重构与 LM 数值正解 (FK)

### 5.1 学习目标与观测挑战
理解为什么并联机构正解必须数值求解。掌握 Linux Host `CLOCK_MONOTONIC` 统一时间轴、Fixed-Lag 严格零外推状态重构以及基于解析雅可比的 Levenberg-Marquardt (LM) 数值正解算法。

* **主要源码**：[`pus6_msgs/msg/TimedJointFeedback.msg`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg), [`pus6_kinematics/pus6_kinematics/state_synchronizer.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/state_synchronizer.py), [`pus6_kinematics/pus6_kinematics/fk_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py)
* **ROS 2 概念**：自定义消息类型、单调时钟时间戳合同、`SingleThreadedExecutor` 独占调度。
* **控制/机器人学概念**：$6	imes 6$ 解析雅可比矩阵构造、LM 自适应阻尼迭代、Warm-Start 暖启动、$	ext{SO}(3)$ 测地线距离跳变门禁。

### 5.2 Fixed-Lag 严格零外推双向夹逼对齐算法

为了消除串行扫描引起的相位畸变，系统引入固定延迟同步器 `FixedLagJointSynchronizer`（J8 推荐参数 $	au = 160	ext{ ms}$）：
在查询时刻 $t_{	ext{query}} = t_{	ext{now}} - 	au$：
1. **深度检查**：所有 6 个队列长度必须 $\ge 2$；
2. **时间跨度检查 (Bracketing Check)**：
   $$orall i \in \{1..6\}, \quad t_{i, 	ext{oldest}} \le t_{	ext{query}} \le t_{i, 	ext{newest}}$$
3. **二分查找与线性插值**：对每个轴查找满足 $t_i^{(k-1)} \le t_{	ext{query}} \le t_i^{(k)}$ 的相邻两帧，计算插值系数：
   $$lpha_i = rac{t_{	ext{query}} - t_i^{(k-1)}}{t_i^{(k)} - t_i^{(k-1)}} \in [0.0, 1.0]$$
   $$\hat{q}_i(t_{	ext{query}}) = q_i^{(k-1)} + lpha_i ig(q_i^{(k)} - q_i^{(k-1)}ig)$$
4. **严格 0 外推**：任一轴未满足夹逼条件立即拒绝发布，绝不使用速度外推预测伪数据。

### 5.3 6×6 解析雅可比矩阵与 LM 迭代

定义支链单位方向矢量 $\mathbf{u}_i = rac{\mathbf{p}_i - \mathbf{s}_i}{\|\mathbf{p}_i - \mathbf{s}_i\|}$。残差偏导数构成雅可比矩阵 $\mathbf{J} \in \mathbb{R}^{6 	imes 6}$ 的第 $i$ 行：
$$\mathbf{J}_i = \left[ \mathbf{u}_i^	op, \quad \mathbf{u}_i^	op \left(rac{\partial \mathbf{R}}{\partial lpha}\mathbf{b}_iight), \quad \mathbf{u}_i^	op \left(rac{\partial \mathbf{R}}{\partial eta}\mathbf{b}_iight), \quad \mathbf{u}_i^	op \left(rac{\partial \mathbf{R}}{\partial \gamma}\mathbf{b}_iight) ight]$$

LM 迭代公式为：
$$\Delta \mathbf{x} = - ig(\mathbf{J}^	op \mathbf{J} + \lambda \mathbf{I}ig)^{-1} \mathbf{J}^	op \mathbf{f}(\mathbf{x})$$

解算收敛后，通过李群 $	ext{SO}(3)$ 测地线距离门禁校验：
$$	heta_{	ext{geodesic}} = rccos\left(	ext{clip}\left(rac{	ext{Tr}(\mathbf{R}_{	ext{last}}^	op \mathbf{R}_{	ext{curr}}) - 1}{2}, -1.0, 1.0ight)ight) \le 0.15	ext{ rad} \ (pprox 8.59^\circ)$$

---

## 第 6 章 (J6): 现场总装调试、零位映射与 HOME/PARK 定位模式基线

### 6.1 学习目标与工程背景
掌握 6PUS 并联机构机械总装后的现场调试流程。理解定位模式 (`mode=0`，10Hz 轮询) 下的推杆中位映射、启动预检及连续 10 循环可靠性验证，明确 J6 定位模式作为系统可靠 Fallback 基线的工程定位。

* **主要源码**：[`scripts/run_j7_servo_home_park.py`](file:///home/lyapunov/bone_drill/bone_ws/scripts/run_j7_servo_home_park.py), `agent_docs/j6/`
* **ROS 2 概念**：底层调试脚本与高层 ROS 2 节点的架构隔离。
* **控制/机器人学概念**：机械总装行程标定 ($25.0\sim 75.0	ext{ mm}$)、电机梯形速度规划、定位模式作为系统安全托底基线。

### 6.2 现场调试验收与 10 循环基线
在 J6 阶段，平台完成了：
1. 6 根推杆物理编号 (ID1~ID6) 与静/动平台球铰几何方位的现场一对一映射核对；
2. 绝对中位标定：推杆总行程 50mm，全缩回为 25.0mm (PARK, raw=0)，行程中点伸出 50.0mm (HOME, raw=1000)；
3. 定位模式连续 10 循环 PARK $\leftrightarrow$ HOME 往复测试（100% PASS，0 故障），确立了系统最底层的可靠性基线。

---

## 第 7 章 (J7): 50Hz 伺服模式演进、五次 S 曲线与运动平滑度量化飞跃

### 7.1 学习目标与卡顿根因
深入理解从定位模式 (`mode=0`) 演进到伺服模式 (`mode=1`) 的物理动因。掌握 50Hz 纯写批次下发时序预算与五次多项式 S 曲线平滑过渡。

* **主要源码**：[`pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py), [`scripts/run_j7_servo_home_park.py`](file:///home/lyapunov/bone_drill/bone_ws/scripts/run_j7_servo_home_park.py)
* **ROS 2 概念**：高频控制定时器与时钟抖动预算。
* **控制/机器人学概念**：五次多项式插值、消除电机梯形加减速微停顿、A/B 实测定量对比。

### 7.2 J6 定位模式 vs J7 伺服模式定量 A/B 实测对比

在相同行程（PARK $\leftrightarrow$ HOME 1000 步 / 25.0mm）下进行实测对比：

```text
┌──────────────────────────────────────┬──────────────────┬──────────────────┬─────────────────────┐
│ 评测指标                             │ J6 定位模式 (10Hz)│ J7 伺服模式 (50Hz)│ 性能飞跃 / 改善幅度 │
├──────────────────────────────────────┼──────────────────┼──────────────────┼─────────────────────┤
│ 目标刷新频率 (Target Dispatch Rate)  │ 10.0 Hz          │ 50.0 Hz          │ 提升 5.0 倍         │
│ 全行程最大离散跟踪误差 (Max Err)     │ 19 步            │ 11 步            │ 降低 42.1%          │
│ 综合 RMS 跟踪误差 (RMS Error)        │ 7.53 步          │ 2.89 步          │ 降低 61.6%          │
│ 峰值冲击电流 (Peak Current)          │ 145 mA           │ 77 mA            │ 降低 46.9%          │
│ 平均工作电流 (Mean Current)          │ 34.2 mA          │ 29.9 mA          │ 降低 12.6%          │
│ 电流标准差 / 电气波动 (Std Dev)      │ 32.8 mA          │ 17.1 mA          │ 降低 47.9%          │
│ 速度阶跃 / 微停顿事件 (Stutter)      │ 频繁可见 (阶跃)  │ 彻底消除 (平滑)  │ 摄像头观测平滑度飞跃 │
│ 最终回位重复性误差 (Return Error)    │ <= 1 步 (0.025mm)│ <= 1 步 (0.025mm)│ 保持亚毫米级高精度   │
└──────────────────────────────────────┴──────────────────┴──────────────────┴─────────────────────┘
```

---

## 第 8 章 (J8): Design-B 伴随反馈、两级看门狗、集约化启动契约与全自由度实机准入

### 8.1 学习目标与终极闭环
掌握 J8 连续笛卡尔 50Hz 伺服运动控制与伴随反馈闭环的完整实现。理解 Design-B 伴随反馈架构、两级看门狗机制、集中式启动状态分类器及 9 阶段实机展示轨迹。

* **主要源码**：[`pus6_motor_driver/pus6_motor_driver/safety.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py), [`pus6_motor_driver/pus6_motor_driver/motor_driver_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py), [`scripts/run_j8_final_video_demonstration.py`](file:///home/lyapunov/bone_drill/bone_ws/scripts/run_j8_final_video_demonstration.py)
* **ROS 2 概念**：生产级安全状态机、两级看门狗解耦、实时硬件在环准入。
* **控制/机器人学概念**：Design-B 0x29 伴随反馈、两级看门狗、`VALID_PARK` 启动契约、9 阶段全自由度实机准入。

### 8.2 Design-B 伴随反馈架构
* **ACTIVE 模式主反馈**：在 ACTIVE 运行模式下，直接提取 50Hz 0x29 目标写入伴随返回的 `StatusFeedback` 作为主反馈源；
* **关键控制路径净化**：常规阻塞式 0x30 轮询完全移出 50Hz 关键控制路径，消除了单总线上的交互抖动；
* **协议基线**：[`protocol.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py) 与 `v1.0-j7-servo-hardware-frozen` 保持 100% 零 diff。

### 8.3 两级安全看门狗体系
为了彻底解耦 UART 通信丢包与标准 Linux 操作系统的调度抖动，系统构建了两级看门狗：
1. **Tier 1 (周期级 6/6 反馈完整性看门狗)**：
   - 每个执行的 0x29 周期必须收到全部 6 轴响应；
   - 若任意轴连续 $N = 4$ 个执行周期未收到反馈，立即触发通信故障并执行受控停机。
2. **Tier 2 (调度器截止期安全上限)**：
   - 监测执行周期之间的实际墙上时钟间隔；
   - 阈值设为 **$263.324	ext{ ms}$**，作为操作系统假死或极端调度的非实时安全兜底上限（容忍 Python GC 与执行器正常抖动）。

### 8.4 集约化 PARK / HOME 启动分类契约
位于 [`pus6_motor_driver/safety.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py) 的 `classify_startup_state()`：
* **暂定 PARK 测量带**：$[-5, +5]	ext{ steps}$ ($[-0.125, +0.125]	ext{ mm}$)；
* **`VALID_PARK` 判定**：全 6 轴健康 (`fault == 0`)、3 样本稳定（漂移 $\le 2$ 步）且读数落入 $[-5, +5]$ 测量带：
  - 遥测保留原始未篡改读数（如 `[-4, -5, -2, -3, -4, 0]`）；
  - 规划种子严格设为 **`[0, 0, 0, 0, 0, 0]`**，保证第 0 周期指令严格 $\ge 0$；
* **严格拒绝异常状态**：任意混合或超限负向状态判定为 `MIXED_UNCLASSIFIED`，直接拒绝运动规划，严禁通用脚本级无差别截断。

### 8.5 J8 最终 9 阶段实机展示轨迹与准入考核指标

| 阶段 | 展示动作 | 幅值 / 目标 | 实测沉降残差 / 结果 | 状态 |
| :--- | :--- | :--- | :--- | :--- |
| **Phase 0** | **Cartesian HOME 沉降保持** | $z_0 = 132.612	ext{ mm}$ (3.0s) | $X{=}+0.033, Y{=}-0.019, Z{=}+132.565	ext{ mm}$ | **PASS** |
| **Phase 1** | **X 方向水平平移展示** | $\pm 8.0	ext{ mm}$ | 残差 $\Delta X = -0.138	ext{ mm}$ | **PASS** |
| **Phase 2** | **Y 方向水平平移展示** | $\pm 8.0	ext{ mm}$ | 残差 $\Delta Y = -0.258	ext{ mm}$ | **PASS** |
| **Phase 3** | **Z 方向竖直升降展示** | $\pm 8.0	ext{ mm}$ | 残差 $\Delta Z = -0.044	ext{ mm}$ | **PASS** |
| **Phase 4** | **Roll 空间翻滚旋转展示** | $\pm 8.0^\circ$ | 残差 $\Delta	ext{Roll} = -0.092^\circ$ | **PASS** |
| **Phase 5** | **Pitch 空间俯仰旋转展示** | $\pm 8.0^\circ$ | 残差 $\Delta	ext{Pitch} = -0.087^\circ$ | **PASS** |
| **Phase 6** | **Yaw 空间偏航旋转展示** | $\pm 8.0^\circ$ | 残差 $\Delta	ext{Yaw} = -0.404^\circ$ | **PASS** |
| **Phase 7** | **复合 6 自由度连续空间运动** | $XYZ \pm 5.0	ext{ mm}$, $	ext{Roll/Pitch} \pm 3.0^\circ$, $	ext{Yaw} \pm 5.0^\circ$ (20.0s @ 0.1Hz) | 空间连续平滑跟踪，零失步零堵转 | **PASS** |
| **Phase 8** | **最终 HOME 回位与重复性核验** | $z_0 = 132.612	ext{ mm}$ (4.0s) | 迟滞 $dX{=}+0.115, dY{=}+0.237, dZ{=}+0.045	ext{ mm}$ | **PASS** |

#### 最终实机硬件指标：
* **总运行时长**：$137.128	ext{ s}$（执行 $6,898$ 个 50Hz 伺服周期）；
* **持续实测刷新率**：**$50.30	ext{ Hz}$**（配置目标 50.0 Hz）；
* **6/6 反馈完整率**：**$41,388 / 41,388$ ($100.0\%$)**；
* **通信/硬件异常计数**：0 缺帧、0 调度超时、0 UART 超时、0 电机故障、0 STOP 停机。

---

# Part II: J8 当前生产源码全模块精读与参考 (Source Code Reference by Module)

## 2.1 `pus6_msgs`: 强类型数据契约与单调时钟协议
* [`TimedJointFeedback.msg`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg)：定义 6 轴关节高度、整步、工作电流、故障码以及 `per_axis_rx_done_monotonic_ns[6]` 硬件观测时间戳；
* [`TimedJointCommand.msg`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/TimedJointCommand.msg)：定义带序列号与单调发送时刻的关节下发控制指令；
* [`FkStateStamped.msg`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_msgs/msg/FkStateStamped.msg)：包含算法查询时刻、同步关节高度、LM 迭代次数、定长残差范数与 $	ext{SO}(3)$ 门禁标志。

## 2.2 `pus6_kinematics`: 逆解、正解、几何加载器与状态同步器
* [`geometry_config.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/geometry_config.py)：负责类型化加载 `6pus_geometry.yaml` 单一真值源；
* [`analytical_6pus.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py)：包含闭式解析逆解 `inverse_kinematics` 与解析雅可比 LM 数值正解 `forward_kinematics_with_metrics`；
* [`state_synchronizer.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/state_synchronizer.py)：实现 `FixedLagJointSynchronizer`，在 `threading.RLock` 保护下执行严格零外推双向夹逼对齐；
* [`kinematics_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/kinematics_node.py)：50Hz 逆解计算节点；
* [`fk_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py)：独占 `SingleThreadedExecutor` 的 50Hz/200Hz 实时数值正解观测节点。

## 2.3 `pus6_control`: 有状态轨迹整形器与控制器节点
* [`joint_trajectory_controller.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py)：实现 `StatefulJerkLimitedShaper`，进行三阶状态持续积分与 $C^2$ 限幅；
* [`control_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py)：50Hz 控制主循环节点，发布 `/pus6/command_joint_steps`。

## 2.4 `pus6_motor_driver`: 驱动协议、传输层、安全分类器与电机驱动节点
* [`protocol.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py)：二进制封包、解包与累加和校验实现；
* [`safety.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py)：实现 `classify_startup_state()`、暂定测量带与规划种子解析；
* [`j7_six_axis_servo_transport.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py)：六轴 0x29 批次下发与伴随反馈提取传输层；
* [`motor_driver_node.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py)：驱动主节点，维护 LVW 缓冲、执行 Slew Recovery 与 Preflight 门禁。

## 2.5 `pus6_bringup`: 系统级 Launch 编排与参数配置体系
* [`pus6_system.launch.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/launch/pus6_system.launch.py)：统一编排启动逆解、控制、驱动与正解节点；
* [`system_params.yaml`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/config/system_params.yaml)：系统级控制与时序参数配置文件。

## 2.6 `h30_imu`: 双 H30 惯导微秒硬件时间戳与李群姿态解算
* 包含 C++ 节点驱动，读取双路 Yesense H30 硬件微秒时间戳 (0x51/0x52)，消除软件时钟频偏并进行 $	ext{SO}(3)$ 相对姿态积分。

---

# Part III: 【全链路实战透视】追踪一帧笛卡尔控制指令的完整生命周期 (End-to-End Deep Trace)

```text
                                  J8 50Hz 闭环双数据流端到端交互时序
                                  
 [ 上位轨迹规划器 ]                   [ pus6_kinematics_node ]               [ pus6_control_node ]
        │                                       │                                      │
        │ 1. 50Hz PoseStamped                   │                                      │
        ├──────────────────────────────────────►│                                      │
        │                                       │ 2. 50Hz desired_joint_deltas         │
        │                                       ├─────────────────────────────────────►│
        │                                       │                                      │ 3. 50Hz command_joint_steps
        │                                       │                                      ├────────────────────────┐
        │                                       │                                      │                        │
        │                                       │                                      │                        ▼
 [ pus6_fk_node (单线程) ]              [ pus6_motor_driver_node ]                  [ 物理电缸 ID1..ID6 ]
        │                                       │ ◄─────────────────────────────────────────────────────────────┘
        │                                       │ 4. Single-Slot 缓冲 + Slew 恢复 + Preflight 门禁
        │                                       │ 5. 50Hz 0x29 批次下发 (带 _bus_lock_ctx 批次锁)
        │                                       ├─────────────────────────────────────►│
        │                                       │ 6. 0x29 StatusFeedback 伴随状态返回  │
        │                                       │◄─────────────────────────────────────┤
        │                                       │ 7. 记录 CLOCK_MONOTONIC 接收完成时刻  │
        │ 8. TimedJointFeedback 遥测流          │
        │◄──────────────────────────────────────┤
        │ 9. 压入 FixedLag 历史队列 (tau=160ms) │
        │ 10. 50Hz/200Hz 定时器以 t_query 查询  │
        │ 11. 双向夹逼线性内插生成 q_sync       │
        │ 12. 解析雅可比 LM 数值正解迭代        │
        │ 13. SO(3) 测地线跳变门禁校验          │
        │ 14. 发布 /pus6/fk/state 与 fk/pose    │
        ▼                                       ▼
```

### 3.1 下行控制链 (Control Pipeline)
1. **笛卡尔目标发布**：规划器以 50Hz 发布 `/pus6/target_cartesian_pose`；
2. **几何闭式逆解**：`pus6_kinematics_node` 校验空间包络后，调用 `Analytical6PusKinematics.inverse_kinematics()` 生成 6 轴相对高度差分量 `/pus6/desired_joint_deltas`；
3. **三阶 Jerk 轨迹整形**：`pus6_control_node` 通过 `StatefulJerkLimitedShaper` 积分平滑，叠加 `OPERATING_HOME` (1000 步) 输出 `/pus6/command_joint_steps`；
4. **驱动门禁与自愈**：`pus6_motor_driver_node` 存入 Single-Slot 容器，必要时触发矢量斜率缩放自愈，通过 6 级安全门禁。

### 3.2 物理执行与伴随反馈 (Physical Execution)
5. **50Hz 批次写入**：在全局锁保护下，向 ID1 ~ ID6 连续下发 0x29 目标指令；
6. **伴随反馈提取**：电缸在执行动作的同时返回 0x29 伴随状态帧，驱动层记录 Host `CLOCK_MONOTONIC` 接收单调时间戳。

### 3.3 上行观测链 (Observation Pipeline)
7. **Timed 遥测封装**：驱动层组装 `TimedJointFeedback` 发布到 `/pus6/joint_feedback/timed`；
8. **Fixed-Lag 状态重构**：`Pus6FkNode` 在 $	au = 160	ext{ ms}$ 下执行严格零外推双向夹逼内插，重构出严格同步的关节向量 $\mathbf{q}_{	ext{sync}}$；
9. **解析雅可比 LM 正解与门禁**：以暖启动迭代求解 6DoF 位姿，通过 $	ext{SO}(3)$ 测地线距离门禁后发布 `/pus6/fk/state` 与 `/pus6/fk/pose`。

---

# Part IV: 实机调试、故障复盘与测试驱动开发 (Hardware Debugging, Post-Mortems & TDD)

## 4.1 标志性工业级工程踩坑与根因深度复盘 (9 大 Post-Mortem 案例)

### 1. 双命令入口竞态与覆写翻倍 Bug (J4-F1B-1)
* **[现象]**：50Hz 规划流注入驱动层，理论覆写约 250 次，实测记录达 746 次（翻倍）。
* **[根因]**：控制节点同时发布浮点数与整步两个话题，两路回调并发抢锁写入同一个单槽缓冲。
* **[修复]**：收口为单一权威入口，仅保留 `/pus6/command_joint_steps`。

### 2. 总线竞争假设排除与物理通信长尾定位 (J4-F1B-2)
* **[现象]**：动态运动时 UART 批次耗时 P95 恶化至 48.7ms。
* **[根因]**：埋点测试证实 `uart_lock_wait_ms = 0`，排除了软件锁竞争假设，根因为半双工硬件物理往返耗时波动。
* **[修复]**：保持 50Hz 伺服模式，保留写流活跃保护门禁，容忍物理硬件耗时。

### 3. 安全前检死锁 (Latch-Out Deadlock) (J4-F1D)
* **[现象]**：瞬态串口延时使目标推进超过 20 步门禁，后续波峰帧持续被拒绝，系统陷入停滞。
* **[根因]**：前检拒绝后命令锚点冻结，上层目标继续推进导致差值持续超限。
* **[修复]**：实施 **Time-Aware Vector Slew Recovery**，按时间裕量同比例缩放 6 轴矢量追赶。

### 4. Delivery-Uncertain 悬空状态机与物理重锚 (J4-F1E)
* **[现象]**：串口读超时后命令基准与物理位置脱节。
* **[修复]**：建立 Delivery-Uncertain 状态机：超时触发受控停止 $	o$ 读取 6 轴快照 $	o$ 校验合法性 $	o$ 物理重锚 $	o$ Slew 恢复。

### 5. UART 批次交错治理与 Batch Atomicity 落地 (J5-C1)
* **[现象]**：多线程下 Worker 写入与后台读取相互穿插，扫描跨度拉长。
* **[修复]**：引入 6 轴批次全局互斥锁（`_bus_lock_ctx`），活跃控制期间彻底抑制独立 Poll 轮询（392/392 零重叠）。

### 6. Eager Import 导致的 `pus6_msgs` 依赖断裂问题
* **[现象]**：未 source 环境时独立纯数学脚本报 `ModuleNotFoundError`。
* **[修复]**：算法包 `__init__.py` 采用惰性导入 (Lazy Import)，解除对上层 ROS 生成消息的过早依赖。

### 7. `rclpy` 多线程执行器回调饥饿与单线程执行器收敛 (J5-C2-RT-SW)
* **[现象]**：200Hz 定时器在多线程执行器下触发频率暴跌至 25~38Hz，停顿达数百毫秒。
* **[根因]**：Ready 队列调度机制导致高频定时器发生回调饥饿 (Callback Starvation)。
* **[修复]**：`Pus6FkNode` 独占独立的 `SingleThreadedExecutor`，调度稳定恢复至 200.09Hz (P95 5.14ms)。

### 8. J8-E9 两级看门狗演进：消除 feedback_age 调度误停机
* **[现象]**：在 Design B 50Hz 闭环中，偶尔触发旧版 `feedback_age_exceeded_ID1` 导致驱动误停机。
* **[根因]**：旧版单轴绝对时间超期看门狗未解耦 Linux 调度抖动。
* **[修复]**：重构为两级看门狗：Tier 1 (连续 4 周期缺帧通信保护) + Tier 2 (263.324ms 调度截止期上限)。

### 9. J8-F1 启动负步数与集约化状态分类器演进
* **[现象]**：冷启动时编码器微小负向读数（如 `-4` 步）触发安全门禁拦截。
* **[根因]**：将传感器零漂测量值错误等同于非法负向控制指令。
* **[修复]**：在 `safety.py` 引入 $[-5, +5]$ 测量带与 `VALID_PARK` 状态分类器，保留真实遥测，锚定规划种子 `[0]*6`，指令空间严格 $\ge 0$。

---

## 4.2 为什么必须独占 SingleThreadedExecutor：A~F 分层消融实测与复盘

消融实验 [`bone_ws/scripts/run_j5_c2_rt_sw_c_ablation.py`](file:///home/lyapunov/bone_drill/bone_ws/scripts/run_j5_c2_rt_sw_c_ablation.py) 实测数据：

| 层级 | 测试内容 | callbacks/10s | 实际均值 Hz | period P50 / P95 / P99 (ms) | callback 耗时 P50 / P95 / P99 (ms) |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **A** | 空 200 Hz 定时器 | 2000 | 199.98 | 5.001 / 5.184 / 5.369 | 0.001 / 0.002 / 0.003 |
| **B** | + 30 Hz 模拟反馈发布 | 2001 | 200.06 | 5.001 / 5.199 / 5.428 | 0.001 / 0.002 / 0.003 |
| **C** | + Fixed-Lag 同步，不做 FK | 2001 | 200.07 | 5.001 / 5.304 / 5.635 | 0.045 / 0.091 / 0.115 |
| **D** | + 数值 FK，不发布 ROS 话题 | 2000 | 199.97 | 4.999 / 5.321 / 5.707 | 0.920 / 1.461 / 1.837 |
| **E** | 完整 Pus6FkNode 节点 | 2000 | 199.98 | 4.999 / 5.270 / 5.674 | 1.079 / 1.663 / 2.014 |
| **F** | 完整节点 + 异常故障注入 | 2000 | 199.97 | 4.999 / 5.162 / 5.338 | 1.133 / 1.802 / 2.574 |

---

## 4.3 测试驱动开发 (TDD) 体系与 349 项回归测试规范

```text
                                  6PUS 分层测试验证体系
                                  
  ┌─────────────────────────┐
  │ Real-Time Benchmarks    │ ── A~F 分层消融性能测试 (验证 200Hz 周期 P95/P99 与执行器调度)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ Read-Only Audits        │ ── J8-C 实机只读审计 (验证 0 硬件写入、0 故障、真实 UART 遥测)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ Replay / Synthetic Test │ ── 离线回放数据流，验证 FK 收敛与 SO(3) 门禁
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ Mock Integration Tests  │ ── FakeBus 多电机仿真 (验证 6 级门禁、两级看门狗、VALID_PARK)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ Unit Tests (数学/协议)   │ ── pytest 纯代数测试 (验证 CRC、解析逆解、解析雅可比 LM、YAML 配置)
  └─────────────────────────┘
```

> [!IMPORTANT]
> **测试数量与收集范围规范**：
> 执行 `pytest src/h30_imu src/pus6_bringup src/pus6_control src/pus6_kinematics src/pus6_motor_driver` $	o$ **349 项 100% PASS**（覆盖 5 大功能包全部单元与集成测试）。

---

## 4.4 软件代数自洽 vs 物理机械精度的认知红线

* **严格认知界限**：所有软件单元测试与仿真回放测试（包括 IK↔FK 1000 样本平移均值残差 $5.83	imes 10^{-9}	ext{ mm}$、姿态均值残差 $1.21	imes 10^{-7\circ}$）**仅证明正逆解在纯数学模型与代码实现上的自洽无 bug**；
* **严禁过度宣称**：上述数值指标**绝不代表真实物理机器人的加工制造精度、球铰装配间隙或末端定位精度**。真实机械精度必须经由外部高精度光学跟踪仪实测标定。

---

# 附录 (Appendix)

## 附录 A: 全系统 ROS 2 节点、话题、服务与消息类型全景速查表

| 节点名称 (Node) | 话题/服务名称 (Topic/Service) | 方向 | 消息/服务类型 | 默认 QoS / 频率 | 核心语义 |
| :--- | :--- | :---: | :--- | :--- | :--- |
| `pus6_kinematics_node` | `/pus6/target_cartesian_pose` | Sub | `geometry_msgs/PoseStamped` | Reliable, KeepLast(10) | 上位 50Hz 笛卡尔目标位姿输入 |
| `pus6_kinematics_node` | `/pus6/desired_joint_deltas` | Pub | `std_msgs/Float64MultiArray` | BestEffort, 50Hz | 6 轴相对高度差分量 (m) |
| `pus6_control_node` | `/pus6/command_joint_steps` | Pub | `std_msgs/Int32MultiArray` | BestEffort, KeepLast(1), 50Hz | **唯一权威下行控制入口** (绝对整步) |
| `pus6_motor_driver_node`| `/pus6/joint_feedback/timed` | Pub | `pus6_msgs/TimedJointFeedback`| BestEffort, 50Hz | 带单调接收时间戳的 6 轴遥测流 |
| `pus6_motor_driver_node`| `/pus6/joint_states` | Pub | `sensor_msgs/JointState` | BestEffort, 50Hz | 标准 ROS 2 关节状态输出 |
| `pus6_fk_node` | `/pus6/fk/state` | Pub | `pus6_msgs/FkStateStamped` | BestEffort, 50Hz/200Hz | 完整正解状态诊断与残差消息 |
| `pus6_fk_node` | `/pus6/fk/pose` | Pub | `geometry_msgs/PoseStamped` | BestEffort, 50Hz/200Hz | 动平台实时位姿估计输出 |

---

## 附录 B: J1 ~ J8 核心特性与技术指标演进对比矩阵

| 阶段 | 控制模式 | 目标频率 | 状态反馈机制 | 安全看门狗机制 | 启动与停放契约 | 核心验收结论 |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **J1** | 底层寄存器 | 点对点 | 单轴 0x30 查询 | 无 | 原始整步读写 | RS485 921600 自定义协议调通 |
| **J2** | 多轴只读 | 10 Hz | 6 轴 0x30 串行扫描 | 批次全局锁 | 零写入只读审计 | 0 写入只读审计 PASS |
| **J3** | 独立逆解 | 点对点 | 0x29 伴随反馈 | 绝对硬件限位 [0,2000] | PARK=[0]*6, HOME=[1000]*6 | 台架独立往返就位精度 <= 2 步 |
| **J4** | 连续笛卡尔 | 25 Hz | Piggyback 伴随状态 | Slew Recovery + 5 级门禁 | maintenance_mode 隔离 | 三阶 Jerk 整形与自愈状态机冻结 |
| **J5** | 时间轴/正解 | 200 Hz | 纳秒 TimedJointFeedback | SO(3) 测地线门禁 | 统一单调时间轴 | Fixed-Lag 零外推与 LM 正解冻结 |
| **J6** | 定位模式 (mode=0)| 10 Hz | 0x30 轮询状态 | 梯形限速保护 | 现场总装零位标定 | 连续 10 循环 HOME/PARK PASS |
| **J7** | 伺服模式 (mode=1)| 50 Hz | 纯写 + 1 轴/周期 0x30 | 运行期取消 0x25 读 | 五次 S 曲线平滑过渡 | RMS 误差降低 61.6%，电流降 46.9% |
| **J8** | **50Hz 笛卡尔伺服**| **50.3 Hz** | **Design-B 0x29 伴随主反馈**| **两级看门狗 (Tier1+2)** | **VALID_PARK + 规划种子 [0]*6**| **9 阶段展示全自由度实机准入 PASS** |

---

## 附录 C: 常用 ROS 2 / 驱动调试与硬件实机操作 CLI 指令速查

```bash
# 1. 环境初始化
source /opt/ros/jazzy/setup.bash
source /home/lyapunov/bone_drill/bone_ws/install/setup.bash
cd /home/lyapunov/bone_drill

# 2. 全工作空间 349 项回归测试
pytest src/h30_imu src/pus6_bringup src/pus6_control src/pus6_kinematics src/pus6_motor_driver

# 3. 硬件开机前 READ_ONLY 只读审计与启动状态分类检查
python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j8_c_hardware_readonly_audit.py

# 4. 生产级伺服一键开机 HOME 就位 (24V 开启后)
python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j7_servo_home_park.py   --action home --speed 100 --rate 50 --real-hardware --confirm-motion

# 5. 执行完整的 J8 9 阶段实机视频演示展示轨迹
python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j8_final_video_demonstration.py

# 6. 实验完成：生产级伺服受控 PARK 收拢 (关机前)
python3 /home/lyapunov/bone_drill/bone_ws/scripts/run_j7_servo_home_park.py   --action park --speed 100 --rate 50 --real-hardware --confirm-motion
```
