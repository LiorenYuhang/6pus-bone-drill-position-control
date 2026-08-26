# J7-C2.1 六电机 UART 时间预算审查

日期：2026-08-25  
范围：`bone_drill` 当前 ROS2 驱动、J6 frozen HOME/PARK standalone runner、冻结的 J4/J5 实机时序证据。  
方法：仅代码与既有记录审查；未打开串口、未查询 ROS2 运行时、未发送命令、未连接/操作硬件。`hardware_write_count = 0`。

## 1. 结论摘要

1. 标准 ROS2 bringup 的代码配置是上层生产者 **50 Hz**、UART worker **50 Hz**、闲时 feedback poll **10 Hz**。这三个值是**标称定时器频率**，不是已保证的实际 `0x29` 下发频率。
2. 现有 `command_positions_with_metrics()` 对 ID1 到 ID6 逐轴执行 `0x29` **写入 + 同步等待状态响应**；每个完成的命令批次同时得到六轴 feedback。
3. 冻结实机 J4-F1A 对该六轴命令路径测得：均值 **18.39 ms**、P95 **20.05 ms**、最大 **23.89 ms**。50 Hz 的周期只有 **20.00 ms**，所以当前架构在均值上接近可运行，但 **P95 和最大值已超过 deadline**；不能宣称满足硬 50 Hz 六轴连续流。
4. J6 的 `10 Hz` 是 standalone HOME/PARK 的轨迹采样/外层控制循环频率。它不是单轴 UART 频率，也不是总线可以达到的最大频率；`0x29` 只在量化后的轨迹点改变时发送，因此实际命令频率不超过 10 Hz。

## 2. 当前命令周期

### 2.1 ROS2 目标生产与消费链路

```text
pus6_control_node
  control_rate_hz = 50 Hz
  create_timer(20 ms)
  仅在已收到 /pus6/joint_states 后，每 tick 发布 steps 目标
        |
        | /pus6/command_joint_steps, BEST_EFFORT + KEEP_LAST(1)
        v
Pus6MotorDriverNode subscriber
  _enqueue_latest_target(): 单槽 Latest-Value-Wins
        |
        v
worker timer: write_rate_hz = 50 Hz, 20 ms
  仅消费一个尚未 consumed 的最新目标
        |
        v
command_positions_with_metrics()
  ID1 -> ID6: 0x29 write + wait response
```

标准 bringup 的 `system_params.yaml` 为 driver 写 timer 配置 `50 Hz`，为 control node 配置 `50 Hz`。[system_params.yaml](/home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/config/system_params.yaml:8) [system_params.yaml](/home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/config/system_params.yaml:24)

control node 的 20 ms timer 只有在已有反馈 `_current_positions` 后才发布；每次成功循环会发布 `command_joint_steps`。[control_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py:154) [control_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py:174)

driver 的 subscriber 不做 UART I/O，只覆盖单槽目标。worker 每次最多取一个未消费目标；无新目标时直接返回，且 watchdog 超时后不重发旧目标。[motor_driver_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:350) [motor_driver_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:547)

### 2.2 实际 `command_positions()` 调用频率

| 层级 | 频率或上限 | 代码语义 |
|---|---:|---|
| 上层 `pus6_control_node` 目标生成 | 标称 50 Hz | 有 feedback 才发布；其 ROS2 timer 仍受调度影响 |
| DDS/driver 单槽更新 | 至多随收到的消息 | `KEEP_LAST(1)` 和单槽覆盖，消息不会排队 |
| driver UART worker | 标称至多 50 Hz | 20 ms timer；一周期至多消费一个目标 |
| `command_positions_with_metrics()` | 至多每个 worker 周期一次 | 只有存在新目标且 watchdog/前检通过才调用 |
| 稳定但停止发布的目标 | 0 Hz | 当前语义不重发 consumed 目标；超时后停止写流 |

因此，**50 Hz 是尝试消费频率上限，而不是持续、无遗漏的 `0x29` 更新保证。**当生产者和 worker 都是 50 Hz 时，二者相位、ROS 调度或批次超时会造成“本周期没有新目标”或“最新目标覆盖前一个目标”。历史 J4 实机流曾刻意使用 50 Hz 上游、25 Hz UART worker，最终实测消费平均仅 23.61 Hz；这说明 Latest-Value-Wins 的设计会主动舍弃中间点，而不是积压它们。[J4_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/frozen/J4_FINAL_SUMMARY.md:54) [J4_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/frozen/J4_FINAL_SUMMARY.md:65)

### 2.3 第一帧额外开销

首次 `command_positions_with_metrics()` 在 `_limits` 尚为空时，会先顺序读取每轴 `0x24` 和 `0x23`，共 **12 个额外读事务**；该耗时发生在六轴 `0x29` 批次计时开始前，但占用同一 UART，可能使首个 worker 周期明显超时。[six_motor_controller.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:283) [six_motor_controller.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:497)

这与冻结 J4-F1B-0 的“首帧 57.87 ms、后续均值 23.48 ms”现象方向一致，但该报告未将首帧拆分到 `read_limits()`，所以不能把它定量归因于这一项。[J4_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/frozen/J4_FINAL_SUMMARY.md:16)

## 3. Feedback 轮询与 UART 竞争

### 3.1 两种 feedback 来源

| 运行状态 | feedback 来源 | 名义频率 | 与命令总线关系 |
|---|---|---:|---|
| 有效 `0x29` 命令批次 | 每轴 write response 中的 `StatusFeedback` | 等于实际命令批次频率 | 已包含在命令事务内，不另发读请求 |
| 闲置或 READ_ONLY | `poll_timer` 的六轴 `read_state_snapshot()` | 标准 bringup 为 10 Hz | 六轴顺序 `0x30` 读状态 |

`Motor.write_register()` 发送后调用 `UartBus.request(..., retries=0)` 并解析 `StatusFeedback`；worker 直接把六个写响应转换为 snapshot 和 ROS feedback，不再额外轮询。[motor.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor.py:70) [motor_driver_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:578) [motor_driver_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:598)

### 3.2 状态读取线程和竞争控制

driver 使用 `MultiThreadedExecutor(num_threads=3)`，subscriber、worker 和 poll timer 分属不同 callback group。[motor_driver_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:890)

但物理 UART 仍只有一个：`UartBus` 使用 `RLock`，而六轴状态读取和六轴写批次都在持锁区顺序完成，所以不会并行发帧。[uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:159) [six_motor_controller.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:345) [six_motor_controller.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:522)

在 ACTIVE 连续目标流中，poll 回调发现最近 500 ms 内收到目标或写入成功后会直接返回，避免稳态 feedback poll 占用 UART；因此**正常连续运动时，独立 poll 不是主瓶颈**。[motor_driver_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:685)

仍存在一个代码可见的边缘竞争窗口：poll 的“是否活跃”判断发生在申请 UART 锁之前。如果 poll 恰好先进入且判断为 idle，它可以获得锁并完成一个六轴读；随后到来的 worker 必须等待该读批次结束。该风险影响开始/恢复边界，不是连续活动流中的每周期固定开销。

配置来源需要区分：node 源码默认 `poll_rate_hz=10`，标准 `pus6_system.launch.py` 默认加载的 `system_params.yaml` 也为 10 Hz；独立的 `pus6_motor_driver/config/motors.yaml` 则写有 50 Hz。后者不是标准 system launch 的默认参数文件，因此本报告采用标准 bringup 的 10 Hz，同时不把它表述为未读取的现场运行时值。

## 4. UART 时间预算

### 4.1 协议线速下限（不是实测总耗时）

串口配置为 921600 baud、8N1，即每字节 10 bit。[uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:83) [uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:130)

`0x29` 写请求包含 10 bytes：`55 AA | len | id | 32 | 29 00 | value(2) | checksum`。正常写响应的帧长度字段为 `0x0F`，总长为 `5 + 15 = 20 bytes`。[protocol.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py:119) [protocol.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py:284)

| 项目 | 计算 | 线速时间 |
|---|---:|---:|
| 单轴 `0x29` TX | `10 B × 10 / 921600` | 0.1085 ms |
| 单轴状态 response RX | `20 B × 10 / 921600` | 0.2170 ms |
| 单轴 TX+RX 原始字节线速 | `30 B × 10 / 921600` | 0.3255 ms |
| 六轴 TX+RX 原始字节线速 | `180 B × 10 / 921600` | 1.9531 ms |

这些数字**不包含**USB-UART/RS485 转向、设备处理、`flush()`、6 次 Python 调用/解析、锁等待、操作系统调度和响应等待；不能作为实机周期结论。

### 4.2 驱动代码的额外帧间预算

`UartBus` 默认 `min_frame_interval=1 ms`。每次 `_send()` 在上一帧 `flush()` 后不足 1 ms 时执行 `time.sleep()`；每个写事务随后阻塞读取一个 response。[uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:83) [uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:174) [uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:250)

若假设 response 在该 1 ms 门限内已返回且忽略一切软件/转向成本，六轴批次的代码路径下限约为：

```text
6 × TX(0.1085 ms) + 5 × inter-frame gate(1.000 ms) + 最后一轴 RX(0.2170 ms)
= 5.8681 ms
=> 170.4 Hz
```

这是“帧间隔约束加字节线速”的理想下限，不是可用于设计的实际吞吐量。每轴 response 若迟于门限，迟到时间直接进入批次时间。

### 4.3 历史实机证据：同一六轴 `0x29` 路径

冻结 J4-F1A runner 以 25 Hz 对固定目标调用 `command_positions_with_metrics()`，用 `perf_counter_ns()` 从调用前到返回后测量六轴写入及六个伴随状态响应；10 秒完成 250 个周期。[run_j4_f1a_real_static_streaming.py](/home/lyapunov/bone_drill/bone_ws/scripts/run_j4_f1a_real_static_streaming.py:155) [run_j4_f1a_real_static_streaming.py](/home/lyapunov/bone_drill/bone_ws/scripts/run_j4_f1a_real_static_streaming.py:207)

| 指标 | 冻结 J4-F1A 实机值 | 换算/含义 |
|---|---:|---|
| 六轴 `0x29` 完整批次均值 | 18.39 ms | 均值服务能力约 54.38 Hz |
| 六轴批次 P95 | 20.05 ms | 已比 50 Hz deadline 多 0.05 ms |
| 六轴批次最大值 | 23.89 ms | 对应单批次服务能力约 41.86 Hz |
| 单轴“均值等效值” | 18.39 / 6 = 3.07 ms | 仅为批次均分，不是逐轴实测 |

上述实机统计来自冻结报告，且确实对应 `0x29 × 6` 静态目标流。[J4_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/frozen/J4_FINAL_SUMMARY.md:15) 它比 1.953 ms 原始线速大约 9.4 倍，表明实际预算主要受设备应答、USB/RS485 和宿主调度，而不是串行比特率。

单次 response 的读 deadline 为 50 ms；运动写入显式 `retries=0`，即一次响应缺失会在该事务异常后进入暂停/恢复路径，不会通过重发目标修复。[uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:87) [motor.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor.py:79)

## 5. J6 的 10 Hz 究竟是什么

J6 Phase 4 wrapper 的 `control_rate_hz` 默认值是 10 Hz；委托的 J3 runner据此计算 `dt=1/control_rate_hz`，生成 quintic 插值点。[run_j6_phase4_home_park.py](/home/lyapunov/bone_drill/bone_ws/scripts/run_j6_phase4_home_park.py:65) [run_j3_startup_homing.py](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:368)

| 问题 | J6 frozen standalone 的答案 |
|---|---|
| trajectory 频率 | **10 Hz**：每轮生成一个 quintic 位置样本 |
| command 调用频率 | **不超过 10 Hz**：仅当整型六轴 `step_cmd` 相对上次改变时才调用 `command_positions()` |
| UART 帧频率 | 不等于 10 Hz：每一次命令调用包含 ID1..ID6 的 6 个 `0x29` 请求和 6 个 response；若量化点不变则本轮为 0 个 `0x29` |
| feedback 频率 | 目标为 **10 Hz**：每一轮无条件执行一次六轴 `read_state_snapshot()`；这是额外 6 个读事务 |

J3 runner 的循环先写变化的轨迹点，再读取六轴状态，最后用“本轮剩余时间”sleep；所以在任何一轮通信总时长超过 100 ms 时，实际循环频率会自动低于 10 Hz，而不是挤压并发 UART 事务。[run_j3_startup_homing.py](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:398) [run_j3_startup_homing.py](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:418) [run_j3_startup_homing.py](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:499)

因此，J6 的“10 Hz”不能被解释为 ROS2 worker 的 50 Hz，也不能被解释为单轴 10 Hz UART。它是独立 commissioning 工具的慢速轨迹/监控循环。冻结 J6 文档也明确该 runner 不代表 ROS2 50 Hz 链路性能。[J6_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/j6/J6_FINAL_SUMMARY.md:47)

## 6. 50 Hz 六轴 command stream 是否满足

### 判断

**当前代码具备 50 Hz 的参数入口和名义调度，但不满足“有 20 ms 硬 deadline 保证的 50 Hz 六轴连续 `0x29` command stream”。**

理由：

- 50 Hz timer 仅给 worker 20 ms 机会；每次实际命令仍是 6 次同步 request/response。
- 冻结实机均值 18.39 ms 尚有 1.61 ms 表面余量，但 P95 20.05 ms 和最大 23.89 ms 已越过 20 ms。
- 当前 worker 也会在 UART 事务前后执行锁、前检、snapshot 构造和 ROS 发布；J4-F1A 的 18.39 ms 是直接 controller 调用度量，不能拿来承诺完整 ROS2 worker 的更低时间。
- 生产者和消费者都为 50 Hz 时没有缓冲余量；Latest-Value-Wins 会丢弃来不及消费的中间点，以保持不积压。

### 瓶颈排序（仅基于代码和冻结实测）

1. **高置信度：六轴顺序阻塞 response。**每个 `0x29` 都经 `Motor.write_register()` 等待和解析状态 response；总线在完整六轴批次期间被锁定。
2. **高置信度：真实事务服务时间已贴近/超过 20 ms。**18.39/20.05/23.89 ms 的同路径冻结实机数据直接说明 50 Hz 没有统计裕量。
3. **高置信度：Latest-Value-Wins 的消费语义。**它保护不排队，但在 worker 落后、同频错相或 J4 的 50→25 配置时，必然跳过部分轨迹点。
4. **中等置信度：`min_frame_interval=1 ms` 与 sleep 调度。**它给每两轴发送引入固定门限；相对实际批次时长不是唯一主因，但它确实增加时延且 `time.sleep()` 没有实时保证。
5. **中等置信度：边缘 feedback poll 竞争。**活动流会跳过 poll，稳态通常不竞争；但 idle→active/恢复竞态可让一次六轴读先占用锁。
6. **中等置信度：首帧限位读取。**首次命令可能附带 12 个读事务，特别不适合纳入持续流的 20 ms 首周期预算。

## 7. 只读后续调查建议

1. 在任何 Servo 硬件实验之前，先定义 50 Hz 的验收统计口径：worker start-to-start、ID1 TX 到 ID6 response、deadline miss、目标年龄、覆盖数和 idle→active 首帧时间，分别统计 P50/P95/P99/max。
2. 复核冻结 J4-F1A 的原始逐周期记录，若仍可恢复，提取 ID1..ID6 的 per-axis 时间，而不是把 18.39 ms 批次均分为单轴性能。
3. 把“具有 response 的当前位置/状态反馈频率”与“独立 `0x30` poll 频率”分开记录；持续流期间后者应持续为零或明确记录被跳过。
4. 对 J7-C2 后续设计保留独立 write-only Servo 调度与低频反馈窗口的方案；本报告不授权修改、mode 切换或任何硬件测试。

## 8. 审查边界

本文件没有读取当前已连接设备的寄存器、USB 延迟参数、ROS 参数服务器或实时日志。因此“当前运行中的实际频率/实际 mode”不在本报告的证据范围内；结论针对仓库当前代码路径和冻结实机历史数据，不替代新的硬件测量。
