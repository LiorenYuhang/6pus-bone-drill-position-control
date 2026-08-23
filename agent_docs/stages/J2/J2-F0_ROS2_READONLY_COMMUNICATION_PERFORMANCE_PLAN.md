# Stage J2-F0 — ROS 2 + UART 六机 READ_ONLY 实时通信能力表征方案 (Rev 2.1.1)

> 项目根目录：`/home/lyapunov/bone_drill`  
> ROS 2 工作空间：`/home/lyapunov/bone_drill/bone_ws`  
> 正式驱动包：`pus6_motor_driver`  
> 当前阶段：`Stage J2-F0`  
> 阶段状态：`PLANNING / NOT EXECUTED`  
> 执行模式：**`hardware_mode = read_only`**（严格禁止任何硬件写入、禁止电机运动、禁止 J2-B）

---

## 1. 阶段目标与核心问题

在 Stage J2-A 中，正式 ROS 2 驱动在真实硬件上实现了 60s 持续运行、全生命周期 0 硬件写入，并测得单次 6 轴纯 UART 扫描服务时间为 $\bar{T}_{\text{cycle}} = 17.86\text{ ms}$（对应理论背靠背总线容量 $55.99\text{ Hz}$），但实际持续采集速率与 ROS 2 发布速率稳定在 $\approx 27.87\text{ Hz}$。

Stage J2-F0 的核心目标是通过**4 个实验组 / 6 个逻辑子测试 / $N$ 个计时窗口（Timed Windows, $N \ge 6$）**，解耦“端到端 Python 驱动背靠背基准”、“协议帧长影响”与“Driver-Side ROS 发布开销及跨进程端到端传输时延”，为后续 1~20 Hz 机构防抖与高频控制建立精准的通信带宽基线。

J2-F0 必须精确回答以下 5 个核心问题：
1. **持续速率瓶颈定位**：量化测定 $27.87\text{ Hz}$ 持续速率中，有多少来自于 Driver 端发布与 ROS 2 定时器调度开销，有多少来自于总线 I/O 与测试脚本结构；
2. **六机全状态读取端到端基准**：在无 ROS 2 调度开销的背靠背（Back-to-Back）模式下，ID1~ID6 完整状态（CMD 0x30，单机 TX 8B / RX 20B）的端到端最大稳定频率与时延抖动（Jitter）；
3. **六机仅读位置端到端基准**：在无 ROS 2 开销下，ID1~ID6 仅读位置寄存器（CMD 0x31 Reg 0x2A，单机 TX 9B / RX 10B）的端到端最大稳定频率与耗时降幅；
4. **Driver-Side ROS 发布与端到端传输引入的精确开销**：量化比较无 ROS 2、标准 50Hz 定时器发布及最高可持续发布速率下的时延开销，并测量 Driver 发布到 Auditor 接收的端到端传输时延；
5. **控制闭环通信带宽基线**：为后续 1~20 Hz 防抖算法与闭环轨迹跟踪提供经过高精度单调时钟测量的通信时钟基准与时间预算上限。

---

## 2. 核心原则与顶层安全门禁

J2-F0 依然是一个**严格纯只读**测试阶段，必须遵守以下顶层安全红线：

```text
┌────────────────────────────────────────────────────────────────────────────┐
│ Stage J2-F0 顶层安全门禁原则                                               │
├────────────────────────────────────────────────────────────────────────────┤
│ 1. 模式强制: hardware_mode 必须显式设为 "read_only";                       │
│ 2. 0-Write 红线: 启动、测试全过程、退出阶段 hardware write_count 严格为 0; │
│ 3. 禁止配置: 严禁发送 configure_all、0x25 模式写、0x28 速度写;              │
│ 4. 禁止目标: 严禁发送 0x29 目标位置写、单轴微动或任何运动指令;             │
│ 5. 禁止制动写入: read_only 模式下即使 shutdown 也不发送 pause;             │
│ 6. 禁止阶段越级: J2-F0 结束后绝对不自动执行 J2-B 或 J2-C;                   │
│ 7. 准入与异常退出标准:                                                     │
│    - Pre/Post 0x30 准入: 严格要求 fault == 0x00 且 current == 0mA (非0拒绝准入);│
│    - 运行期实时监测: 任一电机 fault!=0、电流>500mA(紧急急停)或漂移>3 step即停; │
│    - F0-B / F0-D: 窗口内实时监测位置越界 (>3 step) 与 UART 错误, 每个计时窗口 │
│      前后均执行 Pre/Post 0x30 严格健康核验。                               │
└────────────────────────────────────────────────────────────────────────────┘
```

### 2.1 高精度计时与 ROS 时间戳规范
- **强制计时源**：所有基准测试与时序统计必须全量使用 **`time.perf_counter_ns()`**（或 `time.monotonic_ns()`）获取纳秒级单调时钟；
- **ROS Header.stamp 规范**：严禁将单调时钟纳秒直接强行塞入 ROS `Header.stamp`。ROS 标准消息中的 `Header.stamp` 必须保持标准 ROS 时钟源（`self.get_clock().now().to_msg()`）；
- **基准测试专用插桩**：Driver 节点通过独立的基准测试插桩话题 `/pus6/benchmark_stamp`（`std_msgs/msg/Int64MultiArray`）携带 `[sequence_id, t_pub_mono_ns]`；Auditor 节点在订阅回调入口捕获 `t_recv_mono_ns = time.perf_counter_ns()` 并计算端到端传输时延。

### 2.2 理论线时（Wire-Time）与实测数据严格隔离
- **理论总线传输时间（Wire-Time）**：由波特率 $921600\text{ bps}$（$1\text{ byte} = 10\text{ bits}$ 含起始/停止位，约 $10.8507\,\mu\text{s/B}$）与物理帧字节数直接计算得出；
- **端到端基准（Empirical Measurements）**：包含 USB 驱动、内核 tty 缓存、pyserial 系统调用、协议打包/解包以及电机固件内部响应时延；
- 报告中必须将二者分栏呈现，严禁将理论估算作为实测结果。

### 2.3 软件打包实测字节核算 (Verified Byte Accounting)
通过对 `pus6_motor_driver/protocol.py` 进行软件只读打包实测验证，各帧精确结构与长度如下：
- **0x30 全状态读指令 (CMD_RD_STATUS)**：
  - TX 指令帧：`55 AA 03 <ID> 30 00 00 <CS>` $\rightarrow \mathbf{8\text{ B}}$
  - RX 应答帧：`AA 55 0F <ID> 30 00 00 <12B Data> <CS>` $\rightarrow \mathbf{20\text{ B}}$
  - 单轴总计：$28\text{ B}$；六轴总计：$\text{TX}=48\text{ B},\; \text{RX}=120\text{ B},\; \text{Total}=\mathbf{168\text{ B}}$
  - 六轴理论 Wire-Time：$168 \times 10.8507\,\mu\text{s} \approx \mathbf{1.823\text{ ms}}$
- **0x31 仅读位置指令 (CMD_RD_REGISTER, Reg 0x2A, Count 1)**：
  - TX 指令帧：`55 AA 04 <ID> 31 2A 00 01 <CS>` $\rightarrow \mathbf{9\text{ B}}$（TX 增加 1B/机）
  - RX 应答帧：`AA 55 05 <ID> 31 2A 00 <2B Val> <CS>` $\rightarrow \mathbf{10\text{ B}}$（RX 减少 10B/机）
  - 单轴总计：$19\text{ B}$；六轴总计：$\text{TX}=54\text{ B},\; \text{RX}=60\text{ B},\; \text{Total}=\mathbf{114\text{ B}}$
  - 六轴理论 Wire-Time：$114 \times 10.8507\,\mu\text{s} \approx \mathbf{1.237\text{ ms}}$
- **精确差异与术语定义**：
  - **RX 净减少**：$120\text{ B} - 60\text{ B} = \mathbf{60\text{ B / 6-axis scan}}$
  - **TX 增加**：$54\text{ B} - 48\text{ B} = \mathbf{6\text{ B / 6-axis scan}}$
  - **总 TX+RX 净减少**：$168\text{ B} - 114\text{ B} = \mathbf{54\text{ B / 6-axis scan}}$
  - **理论线时缩减量 (Theoretical Wire-Time Reduction)**：$1.823\text{ ms} - 1.237\text{ ms} = \mathbf{0.586\text{ ms / 6-axis scan}}$
  - **实测端到端扫描耗时缩减量 (Empirical End-to-End Scan-Time Reduction)**：$\bar{T}_{\text{cycle}}(\text{F0-A}) - \bar{T}_{\text{cycle}}(\text{F0-B})$

### 2.4 F0-B / F0-D 仅读位置安全门禁与快照机制
- **物理特性**：0x31 仅读位置应答帧（10 字节）仅包含位置 `0x2A`，**不包含电流、受力、温度或故障码 `0x2F`**；
- **对齐顶层安全门禁的快照机制**：
  1. **Pre-Flight 0x30 安全快照**：进入 30s 计时窗口前，执行一次完整的 0x30 全状态扫描，校验 6 机全部 `fault == 0x00` 且 `current == 0`；
  2. **计时窗口纯净性**：在 30s 计时窗口内，严格仅发送 0x31 读位置指令，实时监测位置越界（$\text{span} \le 3\text{ step}$）与 UART 通信错误，**严禁在计时循环内插入 0x30 指令**以保证基准测试纯净性；
  3. **Post-Flight 0x30 安全快照**：30s 计时窗口结束后，立即执行一次完整的 0x30 全状态扫描，校验 6 机最终 `fault == 0x00` 且 `current == 0`。

---

## 3. ROS 2 拓扑环境冻结配置

为确保 F0-C 与 F0-D 实验结果的精确性与可复现性，固定以下 ROS 2 测试拓扑：

```text
┌────────────────────────────────────────────────────────────────────────────┐
│ ROS 2 运行拓扑冻结配置                                                     │
├────────────────────────────────┬───────────────────────────────────────────┤
│ 参数项                         │ 冻结值                                    │
├────────────────────────────────┼───────────────────────────────────────────┤
│ ROS_DISTRO                     │ jazzy                                     │
│ ROS_DOMAIN_ID                  │ 42 (隔离本地局域网 DDS 广播干扰)          │
│ RMW_IMPLEMENTATION             │ rmw_fastrtps_cpp (默认)                   │
│ Driver Node Executor           │ SingleThreadedExecutor                    │
│ Process Model                  │ 独立双进程 (Driver Node 与 Auditor Node  │
│                                │ 分别在独立的 OS 进程中运行，避免执行器争用)│
│ Auditor Subscriber QoS         │ sensor_data (Best Effort, Volatile, Q=10) │
│ Subscriber Process Count       │ 严格 1 个独立 Auditor 进程                │
└────────────────────────────────┴───────────────────────────────────────────┘
```

---

## 4. 4 个实验组 / 6 个逻辑子测试 / $N$ 个计时窗口结构

J2-F0 规划为 **4 个实验组**，包含 **6 个逻辑子测试**，共涵盖 **$N$ 个独立的 30.0s 计时窗口（Timed Windows, $N \ge 6$）**：

```text
                              ┌──────────────────────────────────────────────┐
                              │ Stage J2-F0 结构 (4 组 / 6 子测试 / N 窗口)   │
                              └──────────────────────┬───────────────────────┘
                                                     │
                      ┌──────────────────────────────┴──────────────────────────────┐
                      ▼                                                             ▼
         【无 ROS 2 发布开销组】                                       【ROS 2 发布完整链路组】
    (直接调用 SixMotorController / UartBus)                         (Pus6MotorDriverNode + 独立 Auditor)
                      │                                                             │
         ┌────────────┴────────────┐                                   ┌────────────┴────────────┐
         ▼                         ▼                                   ▼                         ▼
    ┌──────────┐              ┌──────────┐                        ┌──────────┐              ┌──────────┐
    │  Group A │              │  Group B │                        │  Group C │              │  Group D │
    │  (F0-A)  │              │  (F0-B)  │                        │ (F0-C1/  │              │ (F0-D1/  │
    │ 全状态读取│              │ 仅读位置 │                        │  F0-C2)  │              │  F0-D2)  │
    │ (1个窗口)│              │ (1个窗口)│                        │ 全状态发布│              │ 仅位置发布│
    └──────────┘              └──────────┘                        └──────────┘              └──────────┘
```

### 4.1 Group A (F0-A)：Full-State / No ROS Publish (全状态端到端背靠背基准)
- **子测试 1 (F0-A)**：单次 30.0s 紧凑循环，直接调用 `read_state_snapshot()`（0x30 帧，168B 总线通信）；
- **输出指标**：$N_{\text{cycles}}$、$T_{\text{cycle}}$（Min, Mean, P50, P95, P99, Max, Std）、轴间跨度 $t_{\text{ID6}} - t_{\text{ID1}}$、持续速率 $f_{\text{b2b}} = N / 30.0\text{ s}$。

### 4.2 Group B (F0-B)：Position-Only / No ROS Publish (仅读位置端到端背靠背基准)
- **子测试 2 (F0-B)**：单次 30.0s 紧凑循环，调用 `read_positions_snapshot()`（0x31 帧，114B 总线通信）；
- **安全机制**：Pre/Post Flight 0x30 安全快照；
- **输出指标**：$N_{\text{cycles}}$、$T_{\text{cycle}}$ 分布、持续速率 $f_{\text{b2b}}$、实测端到端扫描耗时缩减量（对比 0.586ms 理论线时缩减量）。

### 4.3 Group C：Full-State / ROS Publish (标准 ROS 2 全状态驱动管线)
包含 2 个逻辑子测试：
- **子测试 3 (F0-C1，1 个计时窗口，30.0s)**：
  - 配置 `poll_rate_hz=50.0`（$20.0\text{ ms}$ 定时器）；
  - 测定定时器触发稳定性、发布时延抖动与独立 Auditor 接收准时率；
- **子测试 4 (F0-C2，包含 $K$ 个扫频计时窗口，各 30.0s)**：
  - **基于 Timer 的自适应扫频测试**：以 F0-A 实测的 $f_{\text{b2b}}$ 为上限基准，设置目标频率阶梯（如 $30\text{ Hz} \rightarrow 35\text{ Hz} \rightarrow 40\text{ Hz} \rightarrow 45\text{ Hz} \rightarrow 50\text{ Hz} \rightarrow f_{\text{b2b}}$）；
  - 严禁使用无定时器的紧凑循环直调作为权威 ROS 速率结果；
  - 逐点测定定时器准时触发率、`deadline_miss_count`、`max_consecutive_deadline_miss`、`Auditor Delivery Ratio` 与 Driver-Side ROS 发布开销。

### 4.4 Group D：Position-Only / ROS Publish (精简 ROS 2 驱动管线)
包含 2 个逻辑子测试：
- **子测试 5 (F0-D1，1 个计时窗口，30.0s)**：
  - 配置 `poll_rate_hz=50.0`，驱动仅读取 0x31 并仅发布 `/pus6/joint_states`；
  - Pre/Post Flight 0x30 安全快照；验证 50Hz 下的定时器稳定性与端到端延迟；
- **子测试 6 (F0-D2，包含 $M$ 个扫频计时窗口，各 30.0s)**：
  - 以 F0-B 实测的 $f_{\text{b2b}}$ 为上限基准，执行阶梯定时器扫频；
  - 测定精简管线下所能达到的**最大可持续 ROS 速率（Maximum Sustainable ROS Rate）**。

---

## 5. 开销度量与端到端传输时延定义

### 5.1 Driver 端 ROS 发布开销 (Driver-Side ROS Publishing Overhead)
为避免在未隔离细分微观组件时做出不实断言，将驱动侧由于引入 ROS 2 定时器回调与消息序列化所增加的单循环耗时严格定义为：
$$\Delta T_{\text{pub\_full}} = \bar{T}_{\text{cycle}}(\text{F0-C2}) - \bar{T}_{\text{cycle}}(\text{F0-A})$$
$$\Delta T_{\text{pub\_pos}} = \bar{T}_{\text{cycle}}(\text{F0-D2}) - \bar{T}_{\text{cycle}}(\text{F0-B})$$

### 5.2 ROS 端到端传输时延 (ROS End-to-End Latency)
在 F0-C 与 F0-D 中，通过插桩话题 `/pus6/benchmark_stamp` 测量消息从驱动发布到审计节点接收的跨进程传输时延：
- Driver 发布前在插桩消息中记录时间戳 $t_{\text{pub}} = \text{time.perf_counter_ns()}$；
- 独立 Auditor 在订阅回调入口记录接收时间戳 $t_{\text{recv}} = \text{time.perf_counter_ns()}$；
- **端到端传输时延**：
  $$L_{\text{e2e}} = \frac{t_{\text{recv}} - t_{\text{pub}}}{10^6} \quad (\text{单位: ms})$$
- 统计 $L_{\text{e2e}}$ 的 Min, Mean, P50, P95, P99, Max。

---

## 6. 最大可持续 ROS 速率 (Maximum Sustainable ROS Rate) 定义与判定标准

在 F0-C2 与 F0-D2 扫频测试中，某一扫频目标频率 $f_{\text{target}}$ 被判定为 **“可持续速率（Sustainable ROS Rate）”**，必须严格同时满足以下条件：

```text
┌────────────────────────────────────────────────────────────────────────────┐
│ 最大可持续 ROS 速率 (Maximum Sustainable ROS Rate) 判定标准                 │
├────────────────────────────────────────────────────────────────────────────┤
│ 1. 定时器准时率: 实际发布周期误差 |T_actual - T_target| < 2.0ms 的比例     │
│    必须 >= 98.0%;                                                          │
│ 2. 审计交付率: Auditor Delivery Ratio = (Recv_Msgs / Pub_Msgs) >= 99.0%;   │
│ 3. 截止期丢失控制: deadline_miss_count 比例 <= 2.0%,                       │
│    max_consecutive_deadline_miss <= 2;                                     │
│ 4. 单轮耗时边界: T_cycle 的 P95 耗时必须严格小于该频率的目标周期 T_target; │
│ 5. 0-Write 红线: 运行全过程 hardware write_count 恒为 0;                   │
│ 6. 通信健康: 连续 30s 扫频点内 UART 读超时与校验错误率 < 0.1%。            │
└────────────────────────────────────────────────────────────────────────────┘
```

满足上述全部条件的最大 $f_{\text{target}}$ 即被记录为该模式下的 **`Maximum Sustainable ROS Rate`**。

---

## 7. J2-F0 PASS / STOP 验收标准

### 7.1 STOP 条件（触发即立刻终止测试）
1. 任何组中检测到 `hardware_write_count > 0` 或拦截到总线 `CMD_WR_REGISTER (0x32)`；
2. Pre/Post 快照检测到 `fault != 0x00` 或静态电流 `current > 0 mA`（严格 0mA 基线准入）；
3. 运行期全状态监测中任一电机电流 $> 500\text{ mA}$（紧急异常电流急停）；
4. 任一电机在任一 30s 子运行中发生单向位置持续漂移 $> 3\text{ step}$；
5. UART 发生致命总线挂死或连续 5 次读取超时。

### 7.2 PASS 条件（4 组 / 6 子测试全部完成且满足下列要求）
1. 全部子运行全生命周期硬件写入次数**严格为 0**；
2. 4 组 / 6 个子测试均完整完成计划的 $30.0\text{ s}$ 采样窗口；
3. 获得基于 `time.perf_counter_ns()` 测量的 $T_{\text{cycle}}$、持续速率、`deadline_miss_count`、`max_consecutive_deadline_miss`、`Auditor Delivery Ratio` 及端到端时延 $L_{\text{e2e}}$；
4. 获得精确的 0x30 全状态 vs 0x31 仅读位置耗时对比；
5. 明确给出 F0-C2 与 F0-D2 的 `Maximum Sustainable ROS Rate` 及其与 F0-A/B 背靠背基准的定量关系；
6. 6 台电机静态位置众数与基线保持一致，波动 $\le 1\text{ step}$。

---

## 8. 架构与版本规范说明

- **J2-A SHA 定位**：Stage J2-A 生成的代码 SHA 清单为该阶段**真实硬件运行的权威快照（Authoritative Run Snapshot）**，用于版本溯源和数据复现，**并非禁止后续驱动代码改进的永久冻结**；
- **J2-F0 实施约束**：本方案仅作为设计规范，未经人工明确授权不得连接物理串口、不得执行任何实机测试。
