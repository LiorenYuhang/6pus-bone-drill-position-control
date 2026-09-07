# 6PUS 骨科并联机器人防抖控制与感知框架

> [English](README.md) | [中文]  
> **面向 6-PUS 并联骨科手术机器人的 ROS 2 实时感知与主动防抖控制系统**

---

## 1. 项目简介 (Overview)

**6PUS-Anti-Vibration** 是专为 6-PUS（移动副-十字万向副-球副）Stewart 并联机构研发的开源高精度实时感知与运动控制框架。项目面向骨科手术机器人领域的高精细操作，旨在通过实时姿态估计、加加速度连续轨迹整形与高速伺服通信，实现平台在受扰动条件下的主动抗扰与振动抑制。

核心能力涵盖：
* **并联机构解析逆解与数值正解**：严格推导的解析几何逆运动学（IK）与基于解析雅可比矩阵的 Levenberg-Marquardt 高频数值正解（FK）；
* **主动抗扰与振动抑制管线**：连续 $C^2$ 加加速度有界（Jerk-Limited）轨迹整形、低延迟单总线批次下发与高带宽闭环响应；
* **双链路解耦 ROS 2 架构**：控制下行链路（50Hz 控制指令）与状态观测上行链路（200Hz 遥测解算）在物理与线程级严格解耦；
* **双 H30 IMU 纳秒/微秒时钟感知**：双路 Yesense H30 惯导模块硬件微秒采样时间戳（0x51/0x52 字段）、软件时钟频偏消除、Session-Start 启动陀螺自适应零偏估计以及李群 $\text{SO}(3)$ 相对姿态双边积分；
* **高可靠串口通信与观测链**：单总线半双工 RS485 921600 8N1 驱动、Batch-Atomicity 全局总线锁、Host `CLOCK_MONOTONIC` 统一时间轴、Fixed-Lag 严格零外推时钟对齐重构与单线程独立 FK 观测节点。

---

## 2. 当前阶段状态 (Project Status)

**当前冻结里程碑**：$\mathbf{J9\ IMU\ Integration\ \&\ Measurement\ Qualification\ COMPLETE}$  
*(冻结运行基线：`v1.0-j8-active-servo-hardware-qualified`；收口文档：[`J9_FINAL_SUMMARY.md`](./J9_FINAL_SUMMARY.md)、[`J9_TO_J10_HANDOFF.md`](./J9_TO_J10_HANDOFF.md))*

六轴 50Hz 连续笛卡尔伺服与 Design-B 伴随反馈控制闭环仍作为冻结运行基线保留：
* **J8 准入结论**：`J8 functional hardware qualification: PASS`
* **Design-B 伴随反馈机制**：ACTIVE 运行模式直接复用 50Hz 0x29 目标写入伴随返回的 `StatusFeedback`，将常规 0x30 阻塞轮询彻底移出 50Hz 关键控制路径；
* **两级安全看门狗**：
  * Tier 1: 执行周期内 6/6 轴反馈完整性监测，连续 $N=4$ 周期缺轴触发通信安全保护；
  * Tier 2: 263.324 ms 调度/截止期非实时操作系统安全上限（非反馈超期时间），容忍标准 Linux 调度抖动；
* **集约化 PARK/HOME 契约**：
  * `PARK_CMD = 0 step` ($25.000\text{ mm}$)，`HOME_CMD = 1000 step` ($50.000\text{ mm}$)；
  * 引入暂定 PARK 测量带 $[-5, +5]\text{ steps}$，多样本稳定性校验通过后判定 `VALID_PARK`，规划种子设为 `[0]*6`，指令空间严格 $\ge 0$；
  * 遥测保留原始未篡改编码器读数，严格拒绝混合/未分类负向异常状态；
* **三轮自治硬件准入**：连续 3 轮完整 PARK $\to$ HOME $\to$ 20s 保持 $\to$ PARK 循环 100% PASS（56,754 / 56,754 帧反馈，0 故障，0 超时，0 停机）；
* **最终 9 阶段实机视频演示考核**：137.128s，6898 周期，实测持续刷新率 50.30 Hz，41,388 / 41,388 响应（100.0% 完整率），0 丢帧，0 故障，0 STOP，各轴单向与 6 自由度复合运动全部平滑就位；
* **全工作空间回归测试**：349 项单元与集成测试 **100% PASS**。
* **J9 收口**：IMU 集成与测量资格已完成；J9_FINAL_SUMMARY.md 与 J9_TO_J10_HANDOFF.md 为新的规范入口。

---

## 3. 系统架构 (Architecture)

```text
┌───────────────────────────────────────────────────────────────────────┐
│                       上位规划层 / 笛卡尔目标指令                     │
└───────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼  (/pus6/target_cartesian_pose)
┌───────────────────────────────────────────────────────────────────────┐
│                       运动学与轨迹规划层 (50Hz)                       │
│  • pus6_kinematics_node : 解析逆运动学计算 (Analytical IK)            │
│  • pus6_control_node    : 有状态 Jerk-Limited 轨迹整形器              │
└───────────────────────────────────────────────────────────────────────┘
                                   │
                                   ▼  (/pus6/command_joint_steps, LVW)
┌───────────────────────────────────────────────────────────────────────┐
│                         电机驱动与底层控制层                          │
│  • J7/J8 六轴 50Hz Servo 传输驱动 (Design-B 0x29 目标写入与伴随反馈) │
│  • 两级安全看门狗 (Tier 1: 6/6 反馈完整性; Tier 2: 263.324ms 截止期)  │
│  • safety.py 集约化启动状态分类器 (VALID_PARK / VALID_HOME)           │
│  • pus6_motor_driver_node: 六重安全门禁与全局总线锁机制               │
│  • RS485 半双工单总线硬件 (921600 8N1)                                │
└───────────────────────────────────────────────────────────────────────┘
             │                                              │
             ▼ (0x29 StatusFeedback 伴随遥测)               ▼ (物理执行器)
┌──────────────────────────────────────┐       ┌────────────────────────┐
│             状态观测层               │       │        硬件实体        │
│  • TimedJointFeedback 发布器         │       │  • 6x 高精度伺服电缸   │
│  • FixedLagJointSynchronizer         │       │  • 基座 Yesense H30 IMU│
│  • Pus6FkNode (独立线程 FK 观测器)   │       │  • 动平台 H30 IMU      │
│  • h30_relative_attitude_node (SO3)  │       │  • 6PUS Stewart 机构   │
└──────────────────────────────────────┘       └────────────────────────┘
```

---

## 4. 代码结构导航 (Repository Layout)

```text
6PUS-Anti-Vibration/
├── agent_docs/                     # 权威技术文档、决策日志与冻结基线
│   ├── decisions/                  # J8 最终准入报告与历史架构决策
│   ├── frozen/                     # 历史冻结清单
│   └── stages/                     # 分阶段调试指南
│
├── bone_ws/                        # ROS 2 核心工作空间
│   ├── src/
│   │   ├── pus6_msgs/              # 自定义 ROS 2 接口定义
│   │   ├── pus6_kinematics/        # 解析逆解、LM 数值正解与 Fixed-Lag 观测器
│   │   ├── pus6_motor_driver/      # 串口驱动、J7/J8 50Hz Servo 驱动与安全层
│   │   ├── pus6_control/           # 轨迹整形器与关节控制器
│   │   ├── pus6_bringup/           # 系统级 Launch 与参数配置
│   │   └── h30_imu/                # C++ H30 惯导驱动与 SO(3) 姿态解算节点
│   └── scripts/                    # 调试与实机准入脚本
│
├── hardware/                       # 硬件选型表、接线定义与机械 3D 模型
└── 6PUS_SERVO_OPERATION_GUIDE.md   # 唯一日常实机操作指南
```
