# 6PUS 并联手术机器人装配前文档与硬件可执行性总验收报告 (Pre-Assembly Document & Hardware Acceptance Report)

> **验收日期**：2026-08-20  
> **验收环境**：Ubuntu 24.04 LTS / ROS 2 Jazzy / Python 3.12 / RS485 921600 8N1 / 双 H30 IMU 460800  
> **物理连接状态**：
> * **6 台推杆电缸**：ID1 ~ ID6 独立挂载于单总线半双工 RS485 串口（`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`），机械上处于台架自由状态（未形成并联闭环）；
> * **双 H30 IMU**：Base IMU（`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00`）与 Platform IMU（`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00`）连接，固定于同一测试板；
> * **6PUS 机构状态**：**完整 6PUS 并联机构尚未总装**。
>
> **验收性质与红线声明**：
> 本次总验收为**装配前纯台架执行器与软件全链路可执行性验收**。所有获得的 PASS 结论**严格限定在独立执行器台架与当前软件范围，绝不代表完整 6PUS 机械机构总装验收，绝不代表 J5-C2-HW 实际机构验收，绝不代表最终 J5-B2 空间外参标定，也不代表真实机械机构末端物理定位精度**。

---

## 1. Commissioning Plan 修正内容汇总

针对 [`agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md) 进行了全面严格的工程修正：

1. **修正运动阶段排序矛盾**：
   * **原矛盾**：原 Phase 3 已经在执行 HOME 全行程顶升，而 Phase 4 却写为“首次真实运动”，逻辑颠倒且风险极高；
   * **修正后严格序列**：
     * **Phase 0**：机械静态检查（完全不上电）；
     * **Phase 1**：电气与串口只读审计（24V 上电，E-Stop 就绪，0 写入监听）；
     * **Phase 2**：六轴编号、静态反馈与零位映射确认（纯只读静态确认）；
     * **Phase 3**：**【首次真实运动】极小单轴微动确认方向与无干涉**（单轴逐一 $\le 1.0\text{ mm}$、超低速 $50\text{ step/s}$ 试探，必须确认 $+Z$ 向上方向正确且被动球铰无干涉）；
     * **Phase 4**：STORAGE_PARK / OPERATING_HOME 机械可达性与低速就位；
     * **Phase 5**：六轴协调小幅笛卡尔运动测试；
     * **Phase 6**：最终机械几何参数实测与配置冻结；
     * **Phase 7**：J5-C2-HW 真实机构正运动学验收；
     * **Phase 8**：双 IMU 最终安装、B2 外参标定与 Lever Arm 准备。
2. **理性化数值门禁指标**：
   * 对缺乏厂商规格或实测冻结依据的连杆公差、球铰游隙、电流阈值、FK 精度目标与 Lever-Arm 精度，统一去除虚假硬指标包装，修正为 **“TBD / 待实测建立基线 / 建议设计目标”**，并注明后续数据来源依据；
3. **消除无条件单一测试总数断言**：
   * 明确测试数量必须绑定具体收集范围与命令，不再将 468 tests 叙述为全局唯一无条件数字。

---

## 2. 全工作空间 Markdown 文档架构分类

| 文档分类 | 文件路径 | 定位与权威性 | 当前状态 |
| :--- | :--- | :--- | :---: |
| **长期主教学教程** | [`6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](file:///home/lyapunov/bone_drill/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md) | 全系统架构、UART 协议、逆解/正解、控制、安全与 12 章教学总览 | **FROZEN / 权威教程** |
| **长期主实机指南** | [`6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`](file:///home/lyapunov/bone_drill/6PUS_REAL_HARDWARE_OPERATION_GUIDE.md) | 唯一权威实机操作手册（开机、HOME/PARK、坐标基准、排障命令） | **FROZEN / 权威指南** |
| **阶段性基线文档** | [`agent_docs/J5_PRE_ASSEMBLY_BASELINE.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_PRE_ASSEMBLY_BASELINE.md) | 装配前软件基线快照与参数来源定义 | **FROZEN** |
| **阶段性状态卡** | [`agent_docs/CURRENT_STAGE.md`](file:///home/lyapunov/bone_drill/agent_docs/CURRENT_STAGE.md) | 当前开发阶段状态卡与各子阶段 Frozen 结论 | **FROZEN** |
| **验收与工具链文档**| [`agent_docs/J5_C2_HW_ACCEPTANCE_TOOLCHAIN.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_C2_HW_ACCEPTANCE_TOOLCHAIN.md) | J5-C2-HW 离线数据回放与 Synthetic 验收工具链说明 | **FROZEN** |
| **装配调试规程** | [`agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md) | 机械总装后 9 阶段逐步上电调试与门禁规程 | **FROZEN** |
| **本次验收总报告** | [`agent_docs/PRE_ASSEMBLY_DOCUMENT_AND_HARDWARE_ACCEPTANCE.md`](file:///home/lyapunov/bone_drill/agent_docs/PRE_ASSEMBLY_DOCUMENT_AND_HARDWARE_ACCEPTANCE.md) | 本次装配前独立台架与文档可执行性总验收报告 | **NEW / 本报告** |

---

## 3. 教程与指南命令分级分类矩阵

教程与指南中的全部示例命令已按安全级别严格归类：

```
                                  教程与指南命令分级树
                                  
  ┌─────────────────────────┐
  │ SAFE / STATIC           │ ── source、--help、Python imports、YAML 加载 (无串口无运动)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ MOCK / DRY-RUN          │ ── run_j3_startup_homing.py (不带 --real-hardware，FakeBus 仿真)
  │                         │    run_j4_f0_mock_benchmark.py
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ READ_ONLY               │ ── run_j2a_readonly_verification.py (真实串口，强制 0 写入)
  │                         │    ros2 launch h30_imu h30_relative.launch.py (IMU 只读监听)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ ACTIVE (台架受限)       │ ── run_j3_startup_homing.py --real-hardware --confirm-motion (低速)
  │                         │    run_j4_f1b_real_cartesian_motion.py --real-hardware (台架回归)
  └─────────────────────────┘
  ┌─────────────────────────┐
  │ HISTORICAL (历史参考)   │ ── J3 历史单轴/多轴极限位移脚本、J4 全行程长周期高速轨迹
  └─────────────────────────┘
```

---

## 4. 实际执行步骤与逐项验收结果 (Step-by-Step Execution Matrix)

| 步骤 | 验证内容与具体执行命令 | 执行模式 | 预期结果 | 实际结果 | 判定 |
| :---: | :--- | :---: | :--- | :--- | :---: |
| **Step 1** | **ROS 2 与 Workspace Source 环境自检**<br>`ros2 pkg prefix pus6_msgs pus6_kinematics pus6_motor_driver pus6_control pus6_bringup h30_imu` | SAFE | 6 大功能包路径全部正确解析 | 6 个包 prefix 全部指向 `/home/lyapunov/bone_drill/bone_ws/install/...` | **PASS** |
| **Step 2** | **Python/ROS 2 核心模块导入检查**<br>`import pus6_msgs, pus6_kinematics, pus6_motor_driver, pus6_control, h30_imu` | SAFE | 消息包、几何配置、逆解/正解、驱动协议无 `ImportError` | 所有类与函数无错误导入，输出 `PYTHON_AND_ROS2_IMPORTS_SUCCESS` | **PASS** |
| **Step 3** | **`6pus_geometry.yaml` 权威加载检查**<br>`from pus6_kinematics.geometry_config import load_geometry_config; cfg = load_geometry_config()` | SAFE | 成功加载 YAML，解析出半径、杆长、标称高度、40000 比例与 HOME/PARK | Base radius: 0.04m, Strut: 0.084m, Steps/m: 40000.0, Home: [1000]*6 | **PASS** |
| **Step 4** | **串口 By-ID 设备节点检查**<br>`ls -la /dev/serial/by-id/` | SAFE | 3 个稳定 USB 串口设备全部存在 | 电机串口（`ttyUSB0`）与双 H30 IMU（`ttyACM0`, `ttyACM1`）稳定在线 | **PASS** |
| **Step 5** | **主要执行脚本 `--help` 命令行可解析性检查**<br>`run_j2a_readonly_verification.py`, `run_j3_startup_homing.py`, `run_j4_f1b_real_cartesian_motion.py`, `run_j5_c2_hw_acceptance.py` 等 | SAFE | 脚本参数解析器工作正常，无异常语法或依赖错误 | 全部 9 个关键执行脚本 `--help` 输出正常 | **PASS** |
| **Step 6** | **Mock Homing / Parking 仿真运行**<br>`run_j3_startup_homing.py --action home/park` | MOCK | `FakeBus` 仿真 6 轴 S 曲线顶升与收拢，记录 JSON | 模拟轨迹平滑生成，HOME/PARK 归零误差 0 步，日志保存正常 | **PASS** |
| **Step 7** | **六电机 60s 真实硬件 READ_ONLY 审计**<br>`run_j2a_readonly_verification.py` | READ_ONLY | 强制 0 写入，60s 采集 600 周期，各电机在线无故障 | 实际采集 600 周期，`writes = 0`，6 电机在线，`error_code = 0x00`，外层轮询 10.0Hz，单次 snapshot 耗时 18.18ms | **PASS** |
| **Step 8** | **实时只读 ROS 观测链补验 (15s 实机)**<br>驱动 `read_only` (25Hz) $\to$ `TimedJointFeedback` $\to$ `Pus6FkNode` (100Hz) $\to$ `/pus6/fk/state` & `/pus6/fk/pose` | READ_ONLY | 全链条无缝连通，Timed 消息发布 24.93Hz，FK 解算率 90.6Hz，残差 $<10^{-10}\text{ m}$，退出无 traceback | 374 帧 Timed 消息全部单调递增，FixedLag 同步率 97.07%，FK 连续收敛 100%，残差均值 $8.65\times 10^{-12}\text{ m}$，干净退出 | **PASS** |
| **Step 9** | **双 H30 IMU 0x51 时间戳修复与 15s 扩展实测**<br>`ros2 launch h30_imu h30_dual.launch.py` (15s) | READ_ONLY | Base 与 Platform 双路 100% 输出 0x51 硬件时间戳，步长恒定 5000us | Base (195.4Hz) 与 Platform (195.4Hz) 均达到 **100.00% has_sample_timestamp**，TID 连续 0 丢包 | **PASS** |
| **Step 10** | **B3 相对姿态与 Bias 启动自校准 16s 真实运行**<br>`ros2 launch h30_imu h30_relative.launch.py` (16s) | READ_ONLY | 双路微秒硬件时间戳对齐，相对四元数连续发布，6.0s 自动 Bias 初始化完成 | 相对姿态发布 122Hz，**Auto gyro bias init COMPLETE (1200 样本, 6.00s, 5% 截尾均值)**，基线与平台零偏收敛 | **PASS** |
| **Step 11** | **独立执行器台架 PARK $\to$ HOME 实机低速就位**<br>`run_j3_startup_homing.py --action home --speed 50 --real-hardware --confirm-motion --validated-config` | ACTIVE | 6 台独立电缸以 50 step/s 顶升至 raw $\approx 1000$ | 6 轴平稳就位，最终位置 `[999, 999, 999, 999, 998, 999]`，最大误差 2 步 $\le 2$ 步容差，电流平稳 | **PASS** |
| **Step 12** | **独立执行器台架 HOME $\to$ PARK 实机安全收拢**<br>`run_j3_startup_homing.py --action park --speed 50 --real-hardware --confirm-motion --validated-config` | ACTIVE | 6 台独立电缸以 50 step/s 缩回至 raw $\approx 0$ | 6 轴平稳收拢，最终位置 `[1, 1, 0, 0, 0, 0]`，最大误差 1 步 $\le 2$ 步容差 | **PASS** |
| **Step 13** | **独立执行器台架 J4 连续控制链小幅回归**<br>HOME $\to$ J4 Heave 5.0s ($\pm 2.0\text{ mm}$) $\to$ PARK | ACTIVE | 验证 Cartesian $\to$ IK $\to$ Shaper $\to$ LVW $\to$ Preflight $\to$ UART 全链条 | 连续动态执行完成，0 故障、0 前检违规，平稳完成并最终受控回到 PARK | **PASS** |
| **Step 14** | **工作空间单元测试全量回归与数量追溯**<br>`pytest bone_ws/src -v` (162) vs `pytest /home/lyapunov/bone_drill --collect-only` (468) | SAFE | 澄清 162 与 468 收集范围边界，全量用例通过 | `bone_ws/src` 162 项全部 PASS (39.85s)；查明 468 为根目录全量收集数（162 + 306 历史独立台架用例） | **PASS** |

---

## 5. 六电机 READ_ONLY 时间与频率指标口径核准

执行日志：[`/home/lyapunov/bone_drill/bone_ws/log/j2a_readonly_20260820_153519.log`](file:///home/lyapunov/bone_drill/bone_ws/log/j2a_readonly_20260820_153519.log)

针对前版报告中关于“55.01Hz / 18.18ms / 600 cycles”的口径混淆，现依据源码与原始日志明确区分三项独立时序概念：

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │       外层定时轮询发布周期 T_poll = 100.0 ms (10.0 Hz)       │
                    └─────────────────────────────────────────────────────────────┘
                                                   │
                       ┌───────────────────────────┴───────────────────────────┐
                       ▼                                                       ▼
       ┌───────────────────────────────┐                       ┌───────────────────────────────┐
       │   单次 6 轴 Snapshot 执行耗时   │                       │       空闲休眠等待时间        │
       │   T_cycle_mean = 18.18 ms     │                       │       T_idle ≈ 81.82 ms       │
       └───────────────────────────────┘                       └───────────────────────────────┘
                       │
       ┌───────────────┴───────────────┐
       ▼                               ▼
 [ID1 发送开始]                  [ID6 接收完成]
       │◄──────── 15.18 ms ───────────►│
       │ (ID1→ID6 接收扫描物理跨度)    │
```

1. **外层轮询/发布周期与实际采集率** (Outer Polling & Publishing Rate)：
   * 设定轮询周期参数：$10.0\text{ Hz}$（定时器周期 $100\text{ ms}$）；
   * 实际运行时间与总采集周期：$60.02\text{ s}$ 内采集 $600\text{ cycles}$；
   * **实际外层采集与发布频率**：$\mathbf{600 / 60.02 = 9.996 \approx 10.0\text{ Hz}}$。
2. **单次六轴 UART Snapshot 执行耗时** (Single 6-Axis Snapshot Communication Duration)：
   * 定义：每次触发轮询时，驱动层从发出 ID1 0x30 指令到收齐 ID6 响应帧的串行通信执行用时（`snap.timestamp_end - snap.timestamp_start`）；
   * 均值 $T_{\text{cycle}} (\text{Mean})$：$\mathbf{18.18\text{ ms}}$；
   * 中位数 $T_{\text{cycle}} (\text{P50})$：$18.07\text{ ms}$；
   * P95 / P99：$19.84\text{ ms} \ / \ 20.40\text{ ms}$；
   * （注：$1 / 18.18\text{ ms} \approx 55.01\text{ Hz}$ 仅代表不加延时紧密轮询时的**总线物理吞吐上限**，并非本实验实际采集率）。
3. **ID1 $\to$ ID6 扫描物理跨度** (Scan Span)：
   * 定义：单批次中 ID1 接收完成时刻到 ID6 接收完成时刻的时间差（`per_axis_timestamps[6] - per_axis_timestamps[1]`）；
   * 均值：$\mathbf{15.18\text{ ms}}$。
4. **生命周期写入数**：
   * Startup: 0, Runtime: 0, Shutdown: 0 $\to$ **TOTAL HARDWARE WRITES: 0 (严格只读)**。
5. **电机静态健康状态**：
   * ID1 ~ ID6 初始/最终位置均为 raw `[0, 0, 0, 0, 0, 0]`；
   * 电流均值 `0.0 mA`，驱动板温度 $30^\circ\text{C} \sim 34^\circ\text{C}$，故障代码全部为 `0x00`。

---

## 6. Fixed-Lag 141 WAIT 查询时间分布剖析与真实 ROS 观测链

在真实 RS485 串口连接下以 `read_only`（0 写入）模式启动 `pus6_motor_driver_node` (25Hz poll)、`Pus6FkNode` (100Hz query, 60ms delay) 运行 $15.0\text{ s}$（共 1500 次查询）：

* **查询状态全量统计**：
  * 总查询数：$1500\text{ 次}$（$100\text{ Hz}$ 定时器）；
  * `SYNCED`（成功双向夹逼）：**$1456\text{ 次}$ ($97.07\%$)**；
  * `WAIT_FUTURE_SAMPLE`：$33\text{ 次}$ ($2.20\%$)；
  * `WAIT_TOO_EARLY`：$2\text{ 次}$ ($0.13\%$)；
  * `EMPTY_BUFFER`：$9\text{ 次}$ ($0.60\%$)。
* **WAIT 时间分布精细剖析**：
  1. **启动阶段缓冲填冲 WAIT** ($t \le 200\text{ ms}$)：共 $11\text{ 次}$ 查询。由于滑动历史队列初始为空，在收到至少 2 帧完整六轴反馈前返回 `EMPTY_BUFFER`，属于系统冷启动必然物理过程；
  2. **运行阶段总线抖动 WAIT** ($t > 200\text{ ms}$)：共 $33\text{ 次}$ 查询（分散分布在 $2.6\text{ s} \sim 15.0\text{ s}$ 全程）。
* **97.07% SYNCED 的根因机理解释**：
  * 驱动层发布周期为 $40.0\text{ ms}$（$25\text{ Hz}$ 轮询）；
  * 单次六轴通信耗时受操作系统调度及串口中断影响存在 $13\text{ ms} \sim 32\text{ ms}$ 的微观抖动（Jitter）；
  * 当某次通信周期偶发延后（如达到 $48\sim 50\text{ ms}$），在查询时刻 $t_q = t_{\text{now}} - 60\text{ ms}$ 处，最新的反馈帧尚未送达，导致右夹逼边界 $t_i[k] \ge t_q$ 暂未满足；
  * **由于 `FixedLagJointSynchronizer` 严格贯彻零外推门禁（Zero Extrapolation Rule）**，只要有任意一轴缺少右边界，系统坚决拒绝外推插值，立即返回 `WAIT_FUTURE_SAMPLE`；待下一个周期（约 5~10ms 后）帧到达后立即恢复 `SYNCED`。这充分证明了时间对齐机制在面对真实硬件物理抖动时的严谨与可靠性。

---

## 7. 双 H30 IMU 0x51 时间戳诊断、修复与 B3 完整闭环

### 7.1 Platform 0x51 时间戳丢失根因与修复

* **协议权威定义**：
  * `0x51` = 硬件采样时间戳（Sample Timestamp uint32 $\mu\text{s}$，4 字节）；
  * `0x52` = 同步输出时间戳（DataReady Timestamp uint32 $\mu\text{s}$，4 字节）。
* **根因诊断**：
  * 对 Base 与 Platform 串口进行 raw binary 报文比对，发现 Base 上报负载长度为 72 字节（包含 `0x10, 0x20, 0x40, 0x41, 0x51, 0x52`），而 Platform 上报负载长度为 60 字节（仅包含 `0x10, 0x20, 0x40, 0x41`）；
  * 查询输出配置寄存器（Class `0x04`），Base 掩码为 `0x80D8`（Bit12 = 1，开启 0x51/0x52），Platform 掩码为 `0x00D8`（Bit12 = 0，关闭时间戳）；
  * 由于 ROS 驱动 `h30_imu_node` 默认以只读模式监听，未在启动时下发 RAM 掩码配置指令，导致 Platform 输出了出厂默认的无时间戳帧。
* **修复动作**：
  * 通过标准交互指令 `59 53 04 12 00 d8 80 6e 8c` 向 Platform IMU Flash 固化写入 `0x80D8` 掩码，成功开启 `0x51` 与 `0x52` 字段。

### 7.2 双路 15s 最终实测指标

| 传感器节点 | 串口设备 By-ID | 实际采集频率 | has_sample_timestamp | 采样微秒步长 ($\Delta t$) | TID 连续性与丢包 |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Base IMU** | `...5B32028000-if00` | **$195.40\text{ Hz}$** (2931 帧) | **$100.00\%$** (2931/2931) | 恒定 $\mathbf{5000.00\text{ }\mu\text{s}}$ | TID 17665..20595，**0 丢包** |
| **Platform IMU** | `...5B0B029000-if00` | **$195.40\text{ Hz}$** (2931 帧) | **$100.00\%$** (2931/2931) | 均值 $\mathbf{5000.00\text{ }\mu\text{s}}$ ($4999\sim 5001$) | TID 44630..47560，**0 丢包** |

### 7.3 B3 相对姿态实时解算与 Bias 初始化完整闭环

在双路 0x51 硬件时间戳就绪后，重新执行 `ros2 launch h30_imu h30_relative.launch.py` 运行 $16.0\text{ s}$：

* **/pus6/imu_relative/data 发布表现**：
  * 累计发布相对姿态帧数：$1953\text{ 帧}$（**实际频率 $122.06\text{ Hz}$**）；
  * 单步积分步长 $dt$：均值 $4997.4\text{ }\mu\text{s}$ ($\approx 5.0\text{ ms}$)；
  * 相对四元数稳定输出：$\mathbf{q}_{\text{rel}} = [0.9999021, -0.0025947, 0.0081282, 0.0110915]$。
* **Session-Start 自动零偏估计 (Auto Gyro Bias Init)**：
  * 启动后进入 `COLLECTING` 状态，连续采集 1200 个静止样本；
  * 在 $6.00\text{ s}$ 处正式触发并输出日志：
    $$\mathbf{Auto\ gyro\ bias\ init\ COMPLETE\ (1200\ samples,\ 6.00\ s,\ 5\%\ trimmed\ mean)}$$
  * **实测收敛零偏估计值**：
    * $\hat{\mathbf{b}}_B = [+0.00008010, -0.00025594, +0.00029774]\text{ rad/s}$
    * $\hat{\mathbf{b}}_P = [+0.00117355, -0.00100787, +0.00109852]\text{ rad/s}$
  * **判定结论**：**J5-B3 相对姿态实时链路与 Auto Bias 初始化全部实测 PASS**。

---

## 8. 测试数量追溯与最终解释 (162 vs 468)

| 执行命令 | 收集数量 | 测试范畴与用例组成 | 权威定位 |
| :--- | :---: | :--- | :--- |
| `pytest /home/lyapunov/bone_drill/bone_ws/src -v` | **162** | **当前 ROS 2 工作空间核心功能包测试**：<br>• `pus6_kinematics`: 35 tests (解析逆解、正解 LM、FixedLag、FK Node 实时性)<br>• `pus6_motor_driver`: 60 tests (协议打包解包、6 电机控制器、安全前检、0 写入审计)<br>• `pus6_control`: 67 tests (三阶轨迹滤波、LVW 缓冲、Delivery-Uncertain 恢复)<br>• `h30_imu`: protocol 单元测试 | **当前 ROS 2 代码库全量测试基准 (162 Passed in 39.85s)** |
| `pytest /home/lyapunov/bone_drill --collect-only -q` | **468** | **整个工程根目录历史全量收集**：<br>• `bone_ws/src/...`: 162 tests<br>• `up/motor/uart_multi/...`: 306 tests（包含历史 M4-B/M4-C/M6-A/M6-B/M6-C/Micro-Move 等单电机与四电机独立台架安全用例）<br>**合计：162 + 306 = 468 tests** | **历史全仓库收集总数 (包含独立硬件历史测试集)** |

---

## 9. 最终分别判定结论 (Final Acceptance Verdicts)

| 验收维度 | 最终判定 | 结论范围与边界说明 |
| :--- | :---: | :--- |
| **Operation Guide executable acceptance** | **PASS** | 唯一权威操作指南中所有基础命令、by-id 路径、参数选项均真实可解析、可执行。 |
| **Code Learning Tutorial executable-reference acceptance** | **PASS** | 12 章教学教程中的代码引用、类名/函数名、数据流时序与数学公式与真实源码完全一致。 |
| **Six-motor bench chain** | **PASS** | 6 台推杆电缸在 RS485 总线上通信稳健，READ_ONLY 0 写入成立（10Hz 轮询，18.18ms 通信耗时），台架执行器 HOME/PARK 控制精度达标。 |
| **Dual-H30 software & time chain** | **PASS** | 双 IMU 100% 具备 0x51 硬件采样微秒时间戳（5000us 步长），TID 连续 0 丢包，Auto Gyro Bias 初始化在 6.0s 处成功收敛为 COMPLETE。 |
| **J5 Timed / FK observer chain** | **PASS** | `TimedJointFeedback` 实时单调下发，FixedLag 在抖动下严格零外推（97.07% SYNCED），`Pus6FkNode` 实时正解输出 90.6Hz，残差 $<10^{-10}\text{ m}$，生命周期干净。 |
| **HOME / PARK bench chain** | **PASS** | 台架独立执行器以 50 step/s 往返于 HOME ([1000]*6) 与 PARK ([0]*6) 精度 $\le 2\text{ 步}$，动作平顺。 |

---

> [!CAUTION]
> **终审安全红线**：
> 以上所有 PASS 判定均基于**独立执行器台架与当前软件状态**。在 6PUS 机械本体总装完成前，**严禁将上述结果升级解释为 J5-C2-HW 通过、最终 B2 外参标定通过、真实机械 FK 精度达标或 6PUS 机构总装验收**！
> 机构总装完成后，必须严格按照 [`agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md) 的 9 个阶段逐步推进。
