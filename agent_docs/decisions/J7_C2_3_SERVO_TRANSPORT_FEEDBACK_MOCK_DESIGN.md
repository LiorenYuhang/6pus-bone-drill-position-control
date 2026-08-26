# J7-C2.3 Servo Transport / RX / Feedback Mock Design

日期：2026-08-25  
范围：J7 Servo 六轴 write-only transport、RX 残留处理和 feedback 调度的实现前 Mock 方案。  
边界：只新增本设计资产；不修改 `protocol.py`、`uart_bus.py`、`motor.py`、ROS2 节点、配置文件或 J6 frozen baseline。未打开串口、未连接硬件、未发送运动命令。`hardware_write_count = 0`。

## 1. 目标与验证边界

J7-C2.3 不是硬件实验，也不是生产 transport 实现。目标是先在 Mock 中回答三个问题：

1. Servo `0x29` 写入即使仍产生 response，RX 残留是否可被计数、排空和正确隔离？
2. feedback 读操作遇到 stale `0x32` response、错误 ID、延迟帧或损坏帧时，是否会拒绝误匹配？
3. 在同一单总线调度器下，方案 A/B/C 的 feedback 工作是否会造成 Servo 50 Hz deadline miss、TX jitter 或 feedback age 超限？

本阶段只定义 Mock 组件、接口、场景、审计字段和通过门槛；不把任何 Mock 结果表述为真实 UART 或电机性能。

## 2. RX 残留模型

### 2.1 `J7ResidualRxBus` 模型

新增的隔离 Mock（实现时放在 J7 专用测试命名空间）应维护以下状态：

```text
rx_buffer: deque[RxFrame]
rx_capacity_bytes
device_response_policy
clock: VirtualClock
```

每次 `write_only_0x29(axis, target, cycle_id)`：

1. 记录 TX start/end、轴 ID、目标、cycle_id；
2. 根据 `device_response_policy` 生成一个可选的 `0x32` response（默认 20 bytes）；
3. 将 response 以 `ready_at_ns = tx_end + response_latency_ns` 放入 RX 队列；
4. **不执行 read、不等待 response、不重试运动帧**；
5. 若队列超过容量，记录 `rx_overflow_count` 和最早丢失帧，不静默清零。

设备响应策略至少覆盖：

| 策略 | 含义 | 用途 |
|---|---|---|
| `never` | Servo `0x29` 不产生 response | 验证最理想 write-only 路径 |
| `always` | 每个 `0x29` 产生标准写 response | RX 残留主场景 |
| `delayed` | response 延迟跨越下一个 Servo 周期 | 验证周期边界残留 |
| `burst` | 多帧集中到达 | 验证排空和容量上限 |
| `malformed` | 错 checksum/长度/ID | 验证 feedback 错误处理 |

### 2.2 RX 状态与审计字段

每个 Servo 周期和 feedback 操作都必须记录：

| 字段 | 说明 |
|---|---|
| `rx_buffer_bytes_start/end` | 操作开始/结束时队列字节数 |
| `rx_residual_frame_count` | 尚未处理的历史 response 帧数 |
| `rx_stale_discard_count` | 明确判定为 stale 并丢弃的帧数 |
| `rx_matched_count` | 与当前 feedback 请求 ID/命令/寄存器匹配的帧数 |
| `rx_unmatched_count` | 解析成功但不匹配当前请求的帧数 |
| `rx_malformed_count` | 长度/checksum/帧头错误数 |
| `rx_overflow_count` | 队列容量溢出次数 |
| `rx_discard_bytes` | 丢弃的原始字节数 |
| `rx_oldest_frame_age_ns` | 队列中最老帧的等待时间 |

审计不能只记录“读到了一个 frame”；必须能区分 response 被匹配、被识别为 stale、被丢弃或因 overflow 丢失。

## 3. Servo Transport Adapter 设计

### 3.1 隔离接口

不改变现有 `UartBus`/`Motor` API，J7 设计一个独立 adapter。接口语义如下：

```text
configure_servo(axis_ids, mode=1, speed=...)
  -> response + readback；失败不 arm

write_only_position(axis_id, target, cycle_id)
  -> TxRecord；不 read、不 retry

begin_feedback(kind, axis_id | axis_ids)
  -> FeedbackRequest；由同一 UART scheduler 串行执行

drain_rx(policy)
  -> RxDrainRecord；记录 stale/malformed/overflow

pause_all(reason)
  -> PauseRecord；安全事件只执行一次，不重发运动帧
```

`TxRecord` 至少包含：`cycle_id`、`axis_id`、`register=0x29`、`target`、`tx_start_ns`、`tx_end_ns`、`write_ok`、`response_wait=False`、`retry_count=0`。  
`FeedbackRecord` 至少包含：请求类型（full/status/position-only）、axis、request start/end、匹配 response 的 ID/command/register、feedback timestamp、age 和错误分类。

### 3.2 单一 writer 状态机

```text
ARMED
  -> SERVO_TX_BATCH
       -> optional RX drain boundary
       -> FEEDBACK_WINDOW (if budget permits)
       -> next 20 ms deadline

SERVO_TX error / RX overflow / safety feedback stale
  -> DELIVERY_UNCERTAIN
  -> stop future Servo TX
  -> one pause_all
  -> STOPPED
```

禁止两个线程分别执行 Servo TX 和 feedback read。所有 TX、RX drain、feedback read 都必须经过同一个 Mock scheduler，以便检验真实单总线的互斥关系。

### 3.3 RX 处置策略

设计必须显式选择并记录以下策略之一，不能隐含“write-only 就没有 RX”：

| 策略 | 行为 | 风险/适用性 |
|---|---|---|
| `discard_before_tx` | 每个 TX 前丢弃当前 RX 队列 | 与 `inspire_motor_uart` 参考相似；可能丢失尚未审计的 feedback |
| `drain_after_tx` | TX 后在限定窗口排空并分类所有回包 | 可审计，但会占用 Servo 周期 |
| `defer_to_feedback` | 运动阶段保留队列，feedback 阶段按 ID/command 匹配 | 队列可能增长，必须有容量/overflow 门禁 |
| `reject_residual` | 发现任何未预期残留即停止 Servo | 最保守，适合作为早期 Mock 安全策略 |

Mock 默认推荐 `defer_to_feedback + bounded_queue`，并增加 `discard_before_tx` 对照组。两者都必须证明：stale `0x32` response 永远不会被当作当前 `0x30/0x31` feedback 接受。

## 4. Stale Response 与误匹配验证

### 4.1 必测场景

| 场景 | 注入 | 必须观察 |
|---|---|---|
| S1：每个运动帧都有回包 | 6 个 `0x29`/周期均生成 `0x32` response | RX residual 持续可计数；Servo TX 不等待 response |
| S2：stale response 先到 | feedback 请求前队列含旧 cycle 的 `0x32` | `rx_stale_discard_count` 增加；不得生成 feedback match |
| S3：错误轴 ID | 队列含同命令但另一 ID 的 response | 计入 unmatched/stale；继续寻找正确 ID或按策略停止 |
| S4：延迟跨周期 | response 在下一个 TX 周期后才 ready | 不得污染下一周期的 cycle_id；应被 drain 或标记 residual |
| S5：损坏帧 | checksum/length/header 错误 | 计入 malformed；不得被 parser 当作当前 feedback |
| S6：RX overflow | `always` response + 有限 buffer + feedback 暂停 | overflow 非零时进入 DELIVERY_UNCERTAIN/STOPPED，不得静默继续 |
| S7：反馈超时 | 当前请求没有匹配 response | 记录 feedback timeout；不重试运动帧；按安全策略停止或跳过 feedback |

### 4.2 通过条件

- `stale 0x32` 永远不能被记录为当前 `0x30` 或 `0x31` feedback。
- 错误 ID、错误 command、错误 register、损坏帧均不能转化为有效反馈。
- RX overflow、连续 feedback timeout 或 malformed 超过门槛时，Servo 写流停止，并且只执行一次 pause。
- 正常 write-only 路径的 `response_wait_count=0`、`motion_retry_count=0`。
- 每个 RX 丢弃动作都有原因、cycle、轴 ID（若可解析）和字节数。

## 5. Feedback 调度 Mock

### 5.1 共同参数

三种方案均使用：

```text
Servo TX nominal rate: 50 Hz
Servo period:          20 ms
Servo axis order:      ID1 -> ID6
Full status reference: 18.18 ms mean historical scan (not a new measurement)
```

Mock 不把 `18.18 ms` 硬编码成设备真值；应允许注入 `full_scan_duration_ns`、单轴读耗时、response latency、OS jitter 和 RX drain cost。

### 5.2 方案 A：100 ms full six-axis scan

调度规则：

```text
每 20 ms：先执行六轴 Servo TX
每 100 ms：在一个明确窗口执行一次六轴 full status scan
```

预算预期：`5.651 ms + 18.18 ms = 23.831 ms`。如果 full scan 与 Servo 批次串行且使用历史均值，至少会产生约 `3.831 ms` 的周期挤压；Mock 应预期 `deadline_miss > 0` 或明确延后下一次 Servo TX，而不是把方案 A 标成无条件 PASS。

记录：

- Servo deadline miss 次数/比例；
- full scan start/end、scan age、每轴 feedback age；
- scan 期间 RX residual/stale discard；
- full scan 前后 TX jitter。

### 5.3 方案 B：逐轴轮询

调度规则：

```text
每 20 ms：先执行六轴 Servo TX
随后只读取一个轴：ID1, ID2, ... ID6 round-robin
```

六个周期完成一轮，理想全轴 feedback 更新率约 `50/6 = 8.33 Hz`；单轴读取耗时必须以可注入参数建模，不能从 18.18 ms 整批均值直接除以 6 当作实测。

预期优点：反馈工作摊平，单次占用小于 full scan，更容易保留 Servo 周期。  
预期代价：单轴 feedback age 最大接近 120 ms（不含额外抖动），六轴不是同一时刻采样。

### 5.4 方案 C：位置快速反馈 + 低频安全反馈

建议 Mock 参数化为：

```text
每个 20 ms 周期：Servo TX + 一个轴的 0x31/0x2A position-only read
每 500 ms（2 Hz）：排队一次六轴 full status safety scan
```

位置反馈一轮仍为 6 个周期，轴位置更新率约 8.33 Hz；full status 只负责 fault/current/temperature 等安全字段，不能被误称为高频闭环反馈。若 safety scan 与 Servo deadline 冲突，应延后 scan 或进入安全状态，不能抢占/重试运动帧。

需要分别记录：

- position-only feedback age（每轴）；
- full safety feedback age（每轴）；
- Servo deadline miss 和 TX jitter；
- safety scan deferred_count；
- RX residual/stale/malformed/overflow。

## 6. A/B/C Mock 验证矩阵

| 测试维度 | 方案 A | 方案 B | 方案 C |
|---|---|---|---|
| Servo TX | 50 Hz 六轴 | 50 Hz 六轴 | 50 Hz 六轴 |
| feedback 工作 | 每 100 ms 六轴 full | 每周期一轴 | 每周期一轴位置 + 2 Hz full safety |
| 预期 Servo deadline | 历史均值模型下存在 miss 风险 | 取决于单轴读注入耗时 | 取决于单轴读和 safety defer 策略 |
| 全轴反馈更新率 | 10 Hz | 8.33 Hz | 位置 8.33 Hz；安全 2 Hz |
| 主要 freshness 风险 | full scan 阻塞 TX | 轴间年龄差 | 安全字段低频/延迟 |
| 主要 RX 风险 | scan 前后 residual 堆积 | 每周期 residual 处理 | 两类反馈 response 混杂 |
| 通过判定重点 | 正确暴露 miss，不静默丢周期 | 无误匹配、TX jitter 受控 | 安全反馈不被位置反馈替代 |

## 7. 统一审计输出

每个 Mock run 应输出 JSON/表格等价字段：

```text
servo_cycle_timestamp_ns
expected_cycle_timestamp_ns
cycle_jitter_ns
axis_id
tx_start_ns / tx_end_ns
batch_complete_ns
servo_deadline_miss
target_age_ns
feedback_kind
feedback_request_start/end_ns
feedback_age_ns
feedback_deferred
rx_buffer_bytes_start/end
rx_residual_frame_count
rx_stale_discard_count
rx_matched_count
rx_unmatched_count
rx_malformed_count
rx_overflow_count
rx_discard_bytes
pause_count
hardware_write_count
```

汇总至少计算 P50/P95/P99/max：Servo 周期、TX skew、cycle jitter、feedback age、RX residual 数量；另列 deadline miss、overflow、误匹配和 pause 次数。

## 8. Mock 通过门槛与未解决问题

### 8.1 实现前的通过门槛

1. C1 的 mode=1 配置和六轴 write-only 语义保持不变，J6 frozen 文件零修改。
2. S1-S7 所有 RX 场景可重复注入；stale response 零误匹配。
3. 运动帧 response wait 和 retry 均为零；部分轴 TX 失败会进入 DELIVERY_UNCERTAIN 并只执行一次 pause。
4. A/B/C 的 feedback age、deferred、RX residual 和 Servo deadline miss 可按注入参数重现。
5. `hardware_write_count = 0`，且测试不会创建真实 `serial.Serial` 或读取 `/dev/serial`。

### 8.2 未解决问题

- 设备在 Servo `0x29` 下是否实际回包，当前只有参考代码的间接证据；必须在 Mock 中默认按“会回包”压力测试。
- RX 清空、排空或延迟匹配的真实成本未知；不能把 `discard_before_tx` 的零残留结果外推为硬件事实。
- 单轴 position-only response 的真实时间未知，方案 B/C 的 20 ms 可行性只能在注入模型下评估。
- full safety scan 的频率、过期阈值和是否允许延后，需要在单电机实验前单独冻结，不能由本设计自动决定。
- 本设计不授权 J7-C2 单电机或 J7-C3 六电机硬件实验；硬件前置条件仍按 J7-C2.2 报告执行。

## 9. 阶段结论

J7-C2.3 的必要设计原则是：**write-only 只是不等待运动响应，不等于忽略 RX；feedback 只允许在同一 UART 调度器的明确窗口执行。**先用有回包、延迟、错误帧和有限容量的 RX Mock 验证 stale 防护，再比较 A/B/C 的 deadline 与 freshness，之后才有资格讨论隔离 transport 的实现。当前阶段到此结束，未实施任何生产代码或硬件动作。
