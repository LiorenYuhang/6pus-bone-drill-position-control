# J4 ROS 2 连续笛卡尔控制管线最终冻结报告 (J4 Final Summary & Freeze Document)

**冻结时间**: 2026-08-18 20:47  
**阶段状态**: $\mathbf{J4\ COMPLETE\ /\ FROZEN}$  
**J4 冻结时回归测试快照**: 145 / 145 Unit Tests Passed（历史计数；当前 workspace 的统一回归计数为 162 tests PASS）

---

## 1. J4 全阶段完成状态汇总 (Stage Matrix)

| 子阶段编号 | 阶段定义与验证内容 | 验证环境 | 核心指标与结果 | 最终判定 |
| :--- | :--- | :---: | :--- | :---: |
| **J4-F0** | 无硬件高频架构、整形滤波与解耦 Mock | Mock | 50Hz/25Hz 生产者-消费者解耦，无锁最新值竞争，145 项单测全部通过 | $\mathbf{PASS}$ |
| **J4-F0-R** | ROS 2 运行时调度与 QoS 加固 | Mock/Node | MultiThreadedExecutor 多线程解耦，BEST_EFFORT KeepLast(1) QoS，前检硬门禁 | $\mathbf{PASS}$ |
| **J4-F1A** | 25 Hz 实机静态目标流与基准时延 | Real HW | 10s 连续 250 周期无丢包，单次 0x29 均值 18.39ms，P95 20.05ms，Max 23.89ms | $\mathbf{PASS}$ |
| **J4-F1B-0** | 实机静态 Arming 与 10 周期 Warm-up | Real HW | 捕获初始实际位置，10 周期预热，首帧 57.87ms，均值 23.48ms，无故障 | $\mathbf{PASS}$ |
| **J4-F1B-1** | Heave-only ($\pm 2.0\text{ mm} @ 0.2\text{ Hz}$) 连续实机运动 | Real HW | 50Hz/25Hz 全链路闭环，幅值保真度 99.4%，相位滞后 187.6ms，电流 85mA | $\mathbf{PASS}$ |
| **J4-F1B-2** | Pitch-only ($\pm 0.020\text{ rad} @ 0.2\text{ Hz}$) 连续实机运动 | Real HW | 单命令权威入口收口，幅值保真度 97.4%，相位滞后 191.0ms，电流 85mA | $\mathbf{PASS}$ |
| **J4-F1C** | Heave + Pitch 双自由度复合实机运动 | Real HW | 15s 复合轨迹，幅值保真度 96.8%~98.9%，轴间偏斜 MAE 10.29 步，电流 102mA | $\mathbf{PASS}$ |
| **J4-F1D** | 3DoF (X+Z+Pitch) 非同相五次平滑注入实机运动 | Real HW | 18s 结构化分段，Delivery-Uncertain 重锚与 Vector Slew Recovery 机制实机闭环验证 | $\mathbf{PASS}$ |
| **J4-F1E** | **Full 6DoF 六个笛卡尔自由度同时连续变化的实机管线验证** | Real HW | 18s 空间全自由度，六轴 MAE 5.03~7.96 步，轴间偏斜 MAE 7.48 步，终端残差 0~2 步 | $\mathbf{PASS}$ |

---

## 2. 正式 ROS 2 控制链路与核心架构

### 2.1 完整数据链路拓扑
```
[Cartesian Reference Stream (PoseStamped)] (50 Hz nominal)
                     │
                     ▼ /pus6/reference_pose
[pus6_kinematics_node (Analytical IK + Temporary J4 Envelope Gate)] (50 Hz)
                     │
                     ▼ /pus6/raw_joint_positions
[pus6_control_node (Stateful Jerk-Limited Reference Shaper)] (50 Hz)
                     │
                     ▼ /pus6/command_joint_steps (Single Authoritative Ingress)
[pus6_motor_driver_node]
    ├── MutuallyExclusiveCallbackGroup (ROS2 Subscriber Thread)
    ├── Single-Slot Latest-Value-Wins Target Buffer (_latest_target)
    ├── MutuallyExclusiveCallbackGroup (UART Worker Thread @ 25 Hz nominal)
    │        │
    │        ▼
    │   [Time-Aware Vector Slew Recovery (Cap <= 20 steps, α-scaled)]
    │        │
    │        ▼
    │   [Safety Preflight Hard Gate (|Δq_i| <= 20 steps, Envelope [200, 1800])]
    │        │
    │        ▼
    │   [SixMotorController.command_positions_with_metrics (0x29 × 6)]
    │        │
    │        ├── Exception? ──► [Delivery-Uncertain Resync: pause_all -> Snapshot Re-anchor]
    │        │
    │        ▼
    └── [/pus6/six_axis_telemetry, /pus6/motor_states, /pus6/driver_diagnostics]
```

### 2.2 运行频率与参数配置
* **笛卡尔参考流与解析逆解频率**：$50\text{ Hz nominal}$（实测平均 $49.33\text{ Hz}$）；
* **三阶状态整形器 (Shaper) 参数**：
  - $\omega_n = 20.0\text{ rad/s}$
  - $v_{\max} = 5.0\text{ mm/s}$ ($200\text{ steps/s}$)
  - $a_{\max} = 50.0\text{ mm/s}^2$
  - $j_{\max} = 500.0\text{ mm/s}^3$
* **UART 驱动层消费频率**：$25\text{ Hz nominal}$（实测平均 $23.61\text{ Hz}$）；
* **通信波特率与底层协议**：$921600\text{ 8N1}$, 单总线半双工 0x29 寄存器写入伴随反馈。

### 2.3 安全门禁与死锁恢复算法
1. **单一权威命令入口 (Single Authoritative Ingress)**：
   - 驱动层仅开放 `/pus6/command_joint_steps` 连续流订阅，彻底消除双入口并发竞态。
2. **Latest-Value-Wins 缓冲与守恒恒等式**：
   - 严格维护单槽缓冲区，实机运行满足：
     $$N_{\text{initial\_residual}} + N_{\text{received}} \equiv N_{\text{consumed}} + N_{\text{overwrite}} + N_{\text{final\_residual}}$$
3. **Time-Aware Vector Slew Recovery**：
   - 当通信延迟或丢包导致累积步数跨度超过时间相关裕量 $\text{Allowance} = \min(20, \max(5, \text{int}(v_{\max}\Delta t)))$ 时，以最后成功下发的目标为命令锚点，对六轴修正矢量按 $\alpha = \text{Allowance} / \max|\Delta q_i|$ 同步缩放；
   - 严格保持 joint-space correction vector 相对比例，绝不进行单轴独立硬截断；
   - 保证物理下发单步最大增量严格 $\le 20\text{ steps}$。
4. **Delivery-Uncertain Resync 状态机**：
   - 发生读超时或写超时导致下发状态不确定时，触发 `pause_all()` $\to$ 读取完整六轴最新位置快照 $\to$ 验证无故障后以实际位置重新建立恢复锚点 $\to$ 经矢量斜率恢复平滑消解滞后目标并重归 NORMAL 运行。
   - 正常运行禁止使用 $q_{\text{actual}}$ 替代命令锚点，确保控制环路稳定性。

---

## 3. 关节空间执行与安全性综合结论

* **关节跟踪误差**：Full 6DoF 全时段六轴 MAE 处于 $5.03 \sim 7.96\text{ steps}$ ($0.125 \sim 0.200\text{ mm}$)，六轴轴间偏斜 MAE 为 $7.48\text{ steps}$ ($0.187\text{ mm}$)，P95 为 $12.00\text{ steps}$ ($0.300\text{ mm}$)；
* **频域动态响应**：幅值保真度达 $97.1\% \sim 99.1\%$，端到端相位滞后高度收敛于 $184.9 \sim 206.5\text{ ms}$；
* **HOME 终端收敛**：运动结束时六轴编码器稳定收敛至 OPERATING_HOME 邻域，终端残差严格处于 $0 \sim 2\text{ steps}$ ($0 \sim 0.050\text{ mm}$，处于单步量化噪声内)；
* **硬件安全监测**：峰值动态电流 $85\text{ mA} \le 400\text{ mA}$，六轴故障码持续全部为 `0x00`，接触力传感器数据平稳无异常突变，未发生机械过载或看门狗停机。

---

## 4. J5 (双 IMU ROS 2 接入与外环控制) 准备接口清单

进入 J5 阶段前，需确认并准备的模块与数据接口：

1. **IMU 传感器硬件与驱动接口**：
   - 基座 IMU (Base IMU) 与动平台 IMU (Platform IMU) 串口/I2C/CAN 通信驱动节点；
   - 发布标准 ROS 2 话题：`/pus6/imu_base/data` 与 `/pus6/imu_platform/data` (`sensor_msgs/msg/Imu`)。
2. **位姿解算与滤波节点 (Pose Estimator Node)**：
   - 订阅双 IMU 角速度与线性加速度，融合输出动平台相对基座的姿态四元数/欧拉角估计 (`geometry_msgs/msg/PoseStamped`)；
   - 明确标注为“基于双 IMU 的姿态状态估计 (IMU-Derived State Estimate)”。
3. **外环闭环控制接口 (Outer-Loop Controller Ingress)**：
   - 接入现有 `JointTrajectoryControllerNode` 中的 `OuterLoopControllerHook`；
   - 支持在外环将笛卡尔参考目标与双 IMU 反馈姿态求差，生成修正项叠加至 `pus6_control_node`。
4. **QoS 与时序同步配置**：
   - 双 IMU 采样时间戳对齐与缓存策略（`message_filters.ApproximateTimeSynchronizer`）；
   - 保证 IMU 数据包到达延迟处于控制管线容忍范围内。

## 5. 后续权威状态与边界说明

本报告冻结的是 J4 控制算法与安全策略。后续 UART transport 已应用六轴 batch-atomicity hotfix；因此不能把本报告理解为“J4 之后完全未修改”。该 hotfix 不改变 J4 控制接口、速度语义或安全门禁。

J5-A、J5-B1、J5-B3、J5-C1、J5-C2-SW、J5-C2-RT-SW 当前均为 **PASS / FROZEN**；J5-B2 为 **DEFERRED**，J5-B4 为 **PENDING**。FK 200 Hz 验收使用独立 `SingleThreadedExecutor`，不能把 J4 驱动节点历史上的 `MultiThreadedExecutor` 描述外推为 FK 实时架构结论。

J3/J4 的历史实机结果主要验证执行器、驱动和控制链路。第 4 节中 `ApproximateTimeSynchronizer` 等内容是 J5 之前的接口候选，不是当前 FK 数据流；后续已采用 J5-B1 软件时间同步、J5-C1 Timed telemetry 和 J5-C2 Fixed-Lag。当前完整 6PUS 机构尚未总装，两个 IMU 仍在同一块测试板上；真实机械 FK 精度、最终 IMU 外参、lever arm、融合和 ESKF 均等待后续装配门槛。
