# J4: ROS2 连续笛卡尔控制管线架构与验证方案

**版本**: v1.0 (Architecture Design & Mock Validation Plan)  
**创建时间**: 2026-08-18  
**定位**: 6PUS 机器人正式 ROS2 连续实时控制管线（从 J3 独立单次脚本升级为标准化 ROS2 节点流）  
**权威依据**:
- 机构几何与逆解模型：`/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics`
- 电机驱动与单总线时序：`/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver`
- 架构总览：`/home/lyapunov/bone_drill/agent_docs/architecture/BONE_WS_ROS2_ARCHITECTURE_PLAN.md`
- J3 验证总结：`/home/lyapunov/bone_drill/agent_docs/stages/J3/J3_IK_REAL_HARDWARE_INTEGRATION_PLAN.md`
- 实机总入口指南：`/home/lyapunov/bone_drill/6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`

> **后续状态覆盖说明**：本文件是 2026-08-18 的架构方案和当时的验证计划，下面的 `J4-F0 Status` 为历史快照。当前 J4 控制算法与安全策略为 **PASS / FROZEN**，UART transport 后续已应用 batch-atomicity hotfix；完整机构仍未装配，当前实机操作边界以 `CURRENT_STAGE.md` 和根目录操作指南为准。

> **原始计划阶段状态 (J4-F0 Status，历史快照)**：
> - **Software Pipeline & Mock Validation**: **PASS** (139/139 pytest PASS; Latest-Value-Wins 非阻塞单槽位解耦; 三阶 $(s+\omega_n)^3$ 连续状态滤波整形器与 $C^2$ 导数保真; 0x29 回包伴随状态解析; Stale-Target Watchdog 超时防重入与零写保护)。
> - **50 Hz Real-Hardware Sustained Streaming**: **NOT YET VALIDATED**。
> - **物理时序关键事实**：J2/J3 实测六轴批次写耗时 Mean 为 17.85 ms，但 Max 达到 **19.85 ms**，相对 20 ms (50 Hz) 调度周期的 worst-case 裕度仅约 **0.15 ms (~0.75%)**。绝不能将 Mock 结果等同于实机稳定，后续实机方案需阶梯式验证 (25Hz / 33Hz / 50Hz) 并严密监控 deadline miss 与总线抖动。

---

## 1. 目标与设计原则

### 1.1 核心目标
将 J3 阶段已通过实机验证的闭式逆解、五次多项式平滑轨迹、双位姿坐标体系与单总线驱动逻辑，封装并重构成标准的 **ROS2 连续控制管线**：

$$\mathbf{Cartesian \ Reference \ Pose/Stream} \longrightarrow \mathbf{pus6\_kinematics} \longrightarrow \mathbf{pus6\_control} \longrightarrow \mathbf{pus6\_motor\_driver} \longrightarrow \mathbf{Feedback \ Stream}$$

### 1.2 核心设计原则
1. **控制层级严格分明**：
   * **当前 J4 阶段**：`Cartesian Reference -> Analytical IK -> Joint-Space Closed-Loop Execution`（关节空间闭环伺服执行）；
   * **后续预留接口**：外环预留直接挂载 PID / 自抗扰控制 (ADRC) / 强化学习 (SAC) 以及动定平台双 H30 IMU 姿态观测融合接口（当前 J4 阶段不实现复杂算法，仅预留标准化 Topic/Hook 契约）。
2. **双位姿与相对坐标统一规范**：
   * 笛卡尔空间参考输入 $x_{\text{cmd}} = [x, y, z, \alpha, \beta, \gamma]$ 统一定义在 `OPERATING_HOME` 几何中心点（标称高度 $z_{\text{nom}} = 132.612268\text{ mm}$）；
   * 电机物理空间统一保持 $q_{\text{relative}} = q_{\text{raw}} - 1000\text{ steps}$；
   * 严禁将 `STORAGE_PARK=[0]*6` 用作控制基准。
3. **确定性时序与时间戳 (Timestamping)**：
   * 所有指令流与状态反馈流均严格携带 `builtin_interfaces/Time` (`header.stamp`) 与单调序列号 (`seq`)；
   * 具备管线全链路通信延迟、计算耗时与跟踪抖动监控。
4. **安全保护与工作空间硬限门禁**：
   * 笛卡尔工作空间边界门禁：平移限制 $|X|, |Y| \le 15\text{ mm}$, $|Z - z_{\text{nom}}| \le 20\text{ mm}$；姿态角限制 $|\alpha|, |\beta|, |\gamma| \le 0.10\text{ rad}$ ($\approx 5.73^\circ$)；
   * 关节行程软限：$q_{\text{raw}} \in [0, 2000]\text{ step}$ ($q_{\text{relative}} \in [-1000, +1000]\text{ step}$ 即 $\pm 25.0\text{ mm}$)；
   * 越界指令在 `pus6_kinematics` 节点入口直接拦截报警并饱和限幅，严禁将异常值投递给底层驱动。

---

## 2. ROS2 节点架构与消息流拓扑

```
+---------------------------------------------------------------------------------------------------------+
|                                        ROS 2 High-Level Command Layer                                   |
|   (GUI / Planner / Cartesian Trajectory Generator / Teleop / Outer-Loop Controller Hook)                |
+---------------------------------------------------------------------------------------------------------+
                                                     │
                                                     ▼ [geometry_msgs/PoseStamped] or [Float64MultiArray]
                                                       /pus6/target_cartesian_pose
                                                       /pus6/target_cartesian_stream
+---------------------------------------------------------------------------------------------------------+
| 1. pus6_kinematics_node (逆运动学与工作空间校验节点)                                                        |
|    - 订阅笛卡尔目标位姿 (标称中心坐标系)                                                                   |
|    - 校验 Cartesian 工作空间合法性                                                                        |
|    - Analytical6PusKinematics 闭式逆解求解 (耗时 < 0.05 ms)                                               |
|    - 校验 6 轴关节行程合法性                                                                              |
+---------------------------------------------------------------------------------------------------------+
                                                     │
                                                     ▼ [std_msgs/Float64MultiArray] & [sensor_msgs/JointState]
                                                       /pus6/desired_joint_positions (SI meters, rel to Home)
                                                       /pus6/desired_joint_steps (Device steps, rel to Home)
+---------------------------------------------------------------------------------------------------------+
| 2. pus6_control_node (连续轨迹插补与关节控制节点)                                                           |
|    - 订阅期望关节目标与当前关节反馈                                                                        |
|    - 五次多项式 S 曲线平滑过渡 / 二阶连续低通滤波插补器                                                      |
|    - 关节速度与加速度饱和限幅 (默认 vmax = 200 step/s = 5.0 mm/s)                                         |
|    - 预留外环控制器扩展接口 (PID / ADRC / SAC Outer-Loop Hooks)                                            |
+---------------------------------------------------------------------------------------------------------+
                                                     │
                                                     ▼ [std_msgs/Float64MultiArray] or [std_msgs/Int32MultiArray]
                                                       /pus6/command_joint_positions (SI meters)
                                                       /pus6/command_joint_steps (Device raw steps)
+---------------------------------------------------------------------------------------------------------+
| 3. pus6_motor_driver_node (六轴单总线多机驱动节点)                                                         |
|    - 921600 波特率 UART 顺序写帧调度 (0x29 绝对位置目标下发)                                               |
|    - 驱动层 SafetyValidator 状态校验 (0x28 限速配置、过流监控、故障码轮询)                                   |
|    - 状态采集发布: /pus6/joint_states (SI m), /pus6/joint_steps_raw, /pus6/joint_currents_raw            |
+---------------------------------------------------------------------------------------------------------+
```

---

## 3. Topic、Service 与接口详细规范

### 3.1 Topics 列表

| Topic 名称 | 消息类型 | 发布节点 | 订阅节点 | 频率 (Hz) | 说明 |
| :--- | :--- | :--- | :--- | :---: | :--- |
| `/pus6/target_cartesian_pose` | `geometry_msgs/PoseStamped` | 上层规划/测试器 | `pus6_kinematics_node` | 异步 / 10 | 笛卡尔目标位姿 (带时间戳) |
| `/pus6/target_cartesian_array` | `std_msgs/Float64MultiArray` | 上层规划/测试器 | `pus6_kinematics_node` | 异步 / 10 | $[x, y, z, \alpha, \beta, \gamma]$ 简化接口 |
| `/pus6/desired_joint_positions` | `std_msgs/Float64MultiArray` | `pus6_kinematics_node` | `pus6_control_node` | 10 ~ 20 | 逆解输出的期望关节位置 (SI 米) |
| `/pus6/desired_joint_steps` | `std_msgs/Int32MultiArray` | `pus6_kinematics_node` | `pus6_control_node` | 10 ~ 20 | 逆解输出的期望关节步数 (整步) |
| `/pus6/command_joint_positions` | `std_msgs/Float64MultiArray` | `pus6_control_node` | `pus6_motor_driver_node` | 10 ~ 20 | 控制插补后的电机下发位置 (SI 米) |
| `/pus6/command_joint_steps` | `std_msgs/Int32MultiArray` | `pus6_control_node` | `pus6_motor_driver_node` | 10 ~ 20 | 控制插补后的电机下发步数 (整步) |
| `/pus6/joint_states` | `sensor_msgs/JointState` | `pus6_motor_driver_node` | `pus6_control_node` / 监视器 | 10 ~ 50 | 测量关节状态 (SI 米，带时间戳) |
| `/pus6/joint_steps_raw` | `std_msgs/Int32MultiArray` | `pus6_motor_driver_node` | 监视器 / 记录器 | 10 ~ 50 | 测量关节整步位置原始遥测 |
| `/pus6/joint_currents_raw` | `std_msgs/Int32MultiArray` | `pus6_motor_driver_node` | 监视器 / 安全监控 | 10 ~ 50 | 各轴电流遥测 (mA) |
| `/pus6/control_diagnostics` | `std_msgs/Float64MultiArray` | `pus6_control_node` | 记录器 | 10 ~ 20 | 跟踪误差、插补延迟、周期抖动 |

### 3.2 Services 列表

| Service 名称 | 服务类型 | 服务端节点 | 说明 |
| :--- | :--- | :--- | :--- |
| `/pus6/startup_home` | `std_srvs/Trigger` | `pus6_control_node` | 触发平滑 Startup 到 `OPERATING_HOME` |
| `/pus6/controlled_park` | `std_srvs/Trigger` | `pus6_control_node` | 触发受控收拢到 `STORAGE_PARK` |
| `/pus6/pause_all` | `std_srvs/Trigger` | `pus6_motor_driver_node` | 立即发送 0x1A 制动锁止全部六轴 |
| `/pus6/configure_motors` | `std_srvs/Trigger` | `pus6_motor_driver_node` | 重新下发 0x28 运行速度与加减速寄存器 |

---

## 4. 连续控制周期与总线时间预算分析 (Timing Budget)

### 4.1 标称控制周期：$10\text{ Hz}$ ($\Delta t = 100\text{ ms}$)

在 921600 波特率半双工 UART 单总线下，6 电机轮询与下发的物理时间预算实测如下：

```
+-----------------------------------------------------------------------------+
| 100 ms Control Cycle (10 Hz)                                                |
|                                                                             |
| [1] ROS 2 IK & Interpolation Compute:   ~0.15 ms  (0.15%)                   |
| [2] 6-Axis Sequential 0x29 Dispatch:    18.17 ms  (18.2%)                   |
| [3] 6-Axis Sequential 0x30/0x2A Poll:   15.00 ms  (15.0%)                   |
| [4] Bus Idle & Safety Margin:           66.68 ms  (66.7%)                   |
+-----------------------------------------------------------------------------+
```

* **总线有效占空比**：$(18.17 + 15.00) / 100 = 33.17\%$，具备超过 **$66\text{ ms}$ 的充裕安全空闲余量**；
* **可扩展性**：未来若提升至 $20\text{ Hz}$ ($\Delta t = 50\text{ ms}$)，总线占空比为 $66.3\%$，依然稳定可靠。

---

## 5. 代码复用与新增模块清单

### 5.1 100% 直接复用的现有代码
* `Analytical6PusKinematics` ([`pus6_kinematics/analytical_6pus.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py))：经过实机 16 项工况检验的权威逆解数学库；
* `SixMotorController` ([`pus6_motor_driver/six_motor_controller.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py))：多轴 UART 协议栈、状态快照与 Mock 引擎；
* `SafetyValidator` ([`pus6_motor_driver/safety.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py))：步数转换、行程限幅、安全常量；
* `QuinticTrajectoryPlanner` ([`run_j3_cartesian_motion.py`](file:///home/lyapunov/bone_drill/bone_ws/scripts/run_j3_cartesian_motion.py))：五次多项式平滑轨迹规划核心。

### 5.2 需要升级与新增的代码模块
1. **升级 `pus6_kinematics_node.py`**：
   * 增加笛卡尔工作空间边界安全拦截；
   * 增加对 `geometry_msgs/PoseStamped` 标准带时间戳消息的订阅支持；
   * 保持 `OPERATING_HOME` 差分步数与绝对步数的规范化广播。
2. **重构/升级 `pus6_control_node.py` 与 `joint_trajectory_controller.py`**：
   * 升级为 `ContinuousCartesianTrajectoryController`，支持连续时变位姿流与阶跃位姿平滑 S 曲线插补；
   * 严格按 `q_relative = q_raw - 1000` 映射；
   * 增加系统启动初始化 Service（`/pus6/startup_home`）与关机收拢 Service（`/pus6/controlled_park`）；
   * 增加外部控制器 Hook 接口（`OuterLoopControllerHook` 虚基类，便于后续无缝插入 PID/ADRC/SAC 控制器）。
3. **新增笛卡尔连续轨迹生成器 (`pus6_cartesian_trajectory_generator.py`)**：
   * 支持发布连续时变笛卡尔轨迹流（正弦波扫描、圆弧循迹、阶跃序列），用于管线端到端验证。
4. **新增 J4 Mock 自动化集成测试套件 (`test_j4_continuous_pipeline_mock.py`)**：
   * 覆盖单点阶跃平滑、连续正弦流追踪、工作空间越界拦截、时间戳连续性与延迟审计。

---

## 6. 第一轮 Mock / Offline 验证计划

1. **Test 1: 笛卡尔工作空间越界硬拦截测试**：
   * 输入超出安全范围的 Cartesian 坐标（例如 $Z +30\text{ mm}$ 或 Roll $0.2\text{ rad}$）；
   * 验证 `pus6_kinematics_node` 正确拒绝并在控制台输出报警，不向下游发送非法指令。
2. **Test 2: 单点阶跃平滑插补测试**：
   * 下发单次目标位姿（例如 Heave $+10\text{ mm}$ 或 Pitch $+0.05\text{ rad}$）；
   * 验证 `pus6_control_node` 生成连续平滑的 10Hz S 曲线关节命令流，无加速度跳变。
3. **Test 3: 连续时变正弦笛卡尔轨迹流追踪测试**：
   * Trajectory Generator 连续以 10Hz 发布 $Z(t) = z_{\text{nom}} + 5\sin(0.5\pi t)\text{ mm}$；
   * 验证全管线实时求解、插补下发与 Mock 电机反馈流闭环，监控各周期 tracking error 与延迟。
4. **Test 4: 双位姿管理与初始化 Service 测试**：
   * 调用 `/pus6/startup_home` 与 `/pus6/controlled_park`，验证状态机转换与到位精度。
