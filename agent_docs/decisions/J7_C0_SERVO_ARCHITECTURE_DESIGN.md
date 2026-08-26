# J7-C0：Servo Mode 隔离实验架构设计

## 1. 设计边界

### 目标

设计一个只属于 J7 的 Servo Mode 实验路径，用于比较：

- J6 frozen baseline：`mode=0`、Positioning Mode、HOME/PARK standalone；
- 当前通用 ROS2 driver：`command_positions()` 的逐轴写入并等待响应；
- J7 candidate：`mode=1`、固定频率位置刷新、运动写入不等待响应。

### 明确不变项

- J6 frozen HOME/PARK 路径不修改、不复用为 Servo 实验入口。
- `protocol.py`、`uart_bus.py`、`motor.py` 视为 UART 基线，不原地修改。
- 不修改现有 ROS2 节点、配置文件、电机参数或安全门禁。
- 本阶段只输出设计，不创建运行程序，不连接硬件，不执行动作。

## 2. 当前驱动可复用模块

| 模块 | 可复用内容 | 不能直接复用的部分 |
|---|---|---|
| `protocol.py` | 帧头、`CMD_WR_REGISTER`、帧打包、校验、响应解析 | 当前没有 Servo 专用语义；不应为了实验修改冻结协议基线 |
| `safety.py` | `CONTROL_MODE_ADDR=0x25`、`TARGET_POSITION_ADDR=0x29`、范围校验、硬件模式、安全异常、单位转换 | 当前只命名 `SPEED_MODE=2`，没有正式 `SERVO_MODE=1` 策略 |
| `SixMotorController` | 连接生命周期、六轴 ID 顺序、限位读取、状态验证、暂停/停止路径、配置读回 | `command_positions_with_metrics()` 对每轴写入都等待响应，不适合作为 Servo 运动发送函数 |
| `UartBus` | 单总线锁、串口打开参数、帧间隔、读帧、传输统计 | `request()` 将发送和响应读取绑定；不能直接得到“写入后立即返回”的运动 API |
| `Motor` | 单轴读状态、读寄存器、普通配置写入及响应解析 | `write_register()` 默认等待响应；不应改写冻结文件来适配 J7 |
| `Pus6MotorDriverNode` | Latest-Value-Wins 单槽、目标 watchdog、目标范围/增量预检、诊断发布、回馈发布 | 当前 worker 只消费新目标；没有 Servo 所需的“新目标未到时，在 watchdog 窗口内周期性刷新上一目标”策略 |
| `FakeBus` | 无硬件状态模型、写入审计、故障注入、模式/速度/目标寄存器镜像 | 当前模型把目标写入直接反映为位置，不能单独证明真实 Servo 插补行为 |
| 现有测试工具 | 写入分类、零写入断言、目标向量审计、Mock 生命周期 | 需要新增 J7 专属测试语义，不能修改 J6 frozen 测试来承载 Servo |

当前响应型运动路径由 [six_motor_controller.py:497](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:497) 和 [six_motor_controller.py:523](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:523) 实现；当前 worker 由 [motor_driver_node.py:513](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:513) 调用。

## 3. Candidate Servo 架构

### 3.1 隔离组件

```text
J7 Servo target source
  -> Latest-Value-Wins target slot
  -> Servo watchdog / anchor gate
  -> 50 Hz Servo scheduler
       (single UART writer)
  -> Servo transport adapter
       mode configuration: response + readback
       motion target: write-only 0x29
  -> six motors in ID order

Independent feedback scheduler
  -> read-only status / position requests
  -> feedback validation and telemetry
  -> no target generation
```

建议将上述组件放在新的 J7 实验命名空间或新增隔离模块中。现有 J6/J4 ROS2 节点只作为库和参考，不作为 Servo 模式的默认入口。

### 3.2 mode=1 配置入口

配置阶段与运动阶段必须分离：

```text
J7 explicit Servo configuration
  -> write 0x25..0x28 with mode=1
  -> wait for configuration responses
  -> read back 0x25 and 0x28 for every participating motor
  -> only after all axes pass, arm Servo stream
```

要求：

- `mode=1` 必须是 J7 显式参数，而不是改变 `configure_all()` 默认值。
- 配置失败时不得进入运动流。
- 不得自动修改或覆盖 J6 的 `POSITIONING_MODE=0`。
- 配置写入允许使用响应型路径，因为配置不是 50 Hz 运动路径。

### 3.3 连续位置发送接口

Servo 运动接口应满足：

- 输入为完整六轴目标向量，或在 C2 阶段限制为单轴向量；
- 目标单位仍为 raw steps；
- 每个 Servo 周期发送一次 `0x29`；
- 运动写入不读取响应、不自动重试；
- 记录每轴实际 TX 时间、序号、目标值和写入结果；
- 上游目标未更新但仍在 watchdog 新鲜窗口内时，可重复发送最近目标；
- 超过 watchdog 窗口后立即停止运动写入，不得无限重复陈旧目标。

这与当前 worker 的语义不同：当前 worker 只有发现未消费的新目标时才调用运动命令，[motor_driver_node.py:547](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:547)。Servo 模式需要“刷新频率”和“目标更新频率”分离，但仍必须受 watchdog 控制。

### 3.4 UART 调度策略

推荐的调度优先级：

1. Servo 运动写入：固定 50 Hz，单一 UART 写者，ID1..ID6 顺序发送。
2. 安全事件：停止 Servo 写流，并按既有规则对各轴执行一次暂停路径。
3. 反馈读取：仅在运动周期预算允许的反馈窗口执行，不得与运动写入并发使用 UART。

必须记录以下时间：

- Servo 周期开始时间；
- 每轴 TX 开始和完成时间；
- 六轴批次完成时间；
- 周期抖动和 deadline miss；
- 目标年龄、覆盖次数、超时次数；
- 运动写入错误和暂停确认结果。

不建议在当前 `UartBus.request()` 外部偷偷丢弃响应，因为这会破坏事务边界。由于 `protocol.py`、`uart_bus.py`、`motor.py` 是冻结基线，J7 应设计独立的 Servo transport adapter 或新的实验模块，避免原地改变现有请求/响应语义。

### 3.5 Feedback 处理策略

Servo 运动写入没有逐帧响应，因此反馈必须成为独立低频流：

```text
Servo write stream: 50 Hz, no response
Feedback stream:    lower-rate read-only polling
Safety layer:       validates fault/current/position freshness
```

反馈策略要求：

- 配置阶段必须有 0x25/0x28 读回；
- 运动阶段定期读取状态或实际位置，但不得与 Servo TX 并发；
- 反馈不能直接重新生成目标，也不能隐式形成 PID；
- 反馈超时、fault、overcurrent、越界或写入失败时，停止 Servo 流并执行一次暂停/停止确认；
- 反馈只用于安全、对齐和实验记录。

## 4. Current 与 Candidate 对比

### Current

```text
command_positions()
  -> validate full target vector
  -> ID1 write 0x29 + wait response
  -> ID2 write 0x29 + wait response
  -> ...
  -> ID6 write 0x29 + wait response
  -> response status becomes telemetry snapshot
```

### Candidate Servo

```text
50 Hz target stream
  -> validate / watchdog / anchor
  -> ID1 write 0x29, no response wait
  -> ID2 write 0x29, no response wait
  -> ...
  -> ID6 write 0x29, no response wait
  -> independent feedback window
```

| 项目 | Current | Candidate Servo |
|---|---|---|
| 模式 | J6 standalone 为 mode=0；通用默认为 mode=2 | 显式 mode=1 |
| 运动发送 | 写入后读取响应 | 写入后立即返回 |
| 目标刷新 | 只发送新消费目标 | 50 Hz 周期刷新最近新鲜目标 |
| 反馈 | 每轴运动响应直接带回馈 | 独立低频反馈 |
| 优点 | 每次写入有状态确认，交付可观测 | 减少阻塞和响应等待，降低运动路径延迟 |
| 主要风险 | 六轴串行事务长，周期可能超时 | 无法逐帧确认交付，存在丢帧/部分轴更新不确定性 |
| 轴间同步 | 受每轴响应时间影响 | 仍按 ID 顺序有 TX skew，不是硬件同时更新 |
| 失败处理 | 当前异常路径有暂停和重同步 | 必须在无响应条件下显式记录“交付不确定”，禁止重试运动帧 |

### 收益

1. 去除六轴运动写入中的逐帧响应等待。
2. 让 50 Hz 位置刷新更接近 Servo 模式的设计要求。
3. 将运动周期与反馈周期解耦，便于测量实际 TX 抖动。
4. 保留上游轨迹、目标限位、watchdog 和安全审计能力。

### 风险

1. **交付不可确认**：写入成功返回不代表电机已接收或执行；不能把 TX 计数当作运动成功。
2. **部分轴更新**：六轴写入中途失败时，轴间目标可能不一致；必须停止并记录批次状态。
3. **反馈饥饿**：运动写入占满总线时，反馈可能过期；必须有明确的反馈预算。
4. **陈旧目标重复**：Servo 要求周期刷新，但 watchdog 必须优先，不能无限重发最后目标。
5. **配置残留**：mode=1 配置失败或部分成功时，不能直接开始运动；必须逐轴读回。
6. **冻结资产污染**：把 Servo 逻辑塞入现有 J6/J4 路径会改变 frozen baseline 的可追溯性。

## 5. 最小实验路线

### J7-C1：Mock 验证

目标：验证软件调度和安全语义，不宣称电机物理性能。

设计门槛：

- 仅使用 Mock/FakeBus 或新的 J7 test double；真实串口路径不可打开；
- 配置阶段确认六轴 `0x25=1`、`0x28` 读回一致；
- 运动阶段每个 Servo 周期产生六个 `0x29` 写入记录；
- 运动写入不产生响应读取，不发生运动帧自动重试；
- 目标未更新时只在 watchdog 新鲜窗口内重复发送；超时后写入计数停止；
- 模拟写入错误时验证一次暂停路径、停止状态和 graceful exit；
- 验证单轴和六轴目标向量长度、范围、重连锚点和 inter-frame delta 门禁；
- 输出 TX 周期、抖动、覆盖次数、超时、暂停和写入审计。

C1 通过条件：所有安全和调度断言通过，且 J6 frozen 测试、文件和配置均未改变。

### J7-C2：单电机硬件设计

仅在 C1 通过并获得独立硬件授权后进行。设计边界：

- 一次只选一个电机 ID；其他轴保持只读观察；
- 上电后先只读确认 ID、位置、fault、current、0x25、0x28、0x29；
- 只对该轴执行 mode=1 配置和逐项读回；
- 使用受限的小范围目标流，先验证固定 50 Hz TX 和反馈时间轴；
- 任何通信错误、fault、过流、方向异常或停止未确认，立即停止实验；
- 对比记录：目标位置、TX 时间、实际位置、current、fault、反馈年龄；
- 不调用 J6 HOME/PARK runner，不改变 J6 mode=0 或 speed=50 文档基线。

C2 通过条件：单轴 Servo 配置可重复读回，TX 周期可解释，watchdog/暂停可验证，且没有安全异常。

### J7-C3：六电机硬件设计

仅在 C2 通过后进行。设计边界：

- 六轴逐一完成 mode=1、0x28、0x29 配置读回，再整体 arm；
- 初期只使用小幅、低风险、可返回 HOME 的同步目标流；
- Servo 写入保持单一 UART writer 和固定 ID 顺序；
- 反馈读取使用明确的低频窗口，不与 Servo TX 并发；
- 记录每轴 TX skew、批次完成时间、目标年龄、实际位置和故障状态；
- 不将 C3 结果覆盖 J6 frozen HOME/PARK 证据；
- C3 只验证 Servo 架构行为，不直接宣称完整 6PUS 机构振动性能改善。

C3 通过条件：六轴 mode=1 配置完整读回，目标流与反馈流稳定，异常停止路径可复现，且结果与 J6 mode=0 基线可独立比较。

## 6. J7-C0 设计结论

1. 可复用现有的协议打包、安全验证、ID/限位管理、Latest-Value-Wins、watchdog、反馈校验、Mock/FakeBus 和审计框架。
2. `command_positions()` 不能直接作为 Servo 运动接口，因为它把每轴写入与响应读取绑定在一起。
3. Servo 必须新增隔离的 mode=1 配置入口、无响应运动发送适配层、固定频率 scheduler 和独立反馈 scheduler。
4. 无响应运动写入的核心风险是交付不可确认和部分轴更新；收益是降低 50 Hz 运动路径阻塞与周期抖动。
5. 推荐顺序为 Mock -> 单电机硬件 -> 六电机硬件；任何阶段都不得修改 J6 frozen baseline。

本阶段到此结束，不实施任何架构变更。
