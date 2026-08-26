# J7-C2.2 六轴 Servo UART 时间预算设计审查

日期：2026-08-25  
范围：`bone_drill` 当前 UART/六轴驱动、J7-C1 Mock 结论、`inspire_motor_uart` 单电机 Servo 参考。  
边界：只做设计审查和数学预算；未修改代码、未打开串口、未发送运动命令、未连接/操作硬件。`hardware_write_count = 0`。

## 1. 审查结论

- **理论可行性：有条件可行。**在 921600 baud、10-byte `0x29` 写帧下，六轴 write-only 的原始线速只需约 `0.651 ms`；若沿用 Project A 的 `1 ms` 最小帧间隔，六轴批次理想下限约 `5.651 ms`，低于 50 Hz 的 `20 ms` 周期。
- **实际可行性：尚未证明。**当前六轴实机记录 `18.39 ms / P95 20.05 ms / Max 23.89 ms` 包含每轴响应等待；去掉响应后是否能稳定达到约 `5.65 ms`，仍受 USB-UART 延迟、RS485 收发切换、设备处理和主机调度影响，代码审查不能替代实测。
- **六轴同步不是同时更新。**即使 write-only，ID1 到 ID6 仍按顺序发送；沿用 1 ms 帧间隔时最坏 TX start skew 约 `5.543 ms`。这不是硬件锁存同步，只是同一批次的顺序写入。
- **J7-C3 当前不具备直接硬件实验条件。**C1 只验证了 Mock 调度；尚无隔离的六轴 write-only transport、RX 回包处置策略、feedback 窗口实测、单电机 Servo 硬件门槛或独立硬件授权。因此本报告不授权 C3。

## 2. 协议与计算前提

### 2.1 帧长度和线速

Project A 的串口配置为 `921600 baud, 8N1`，即每个 UART 字节按 10 bit 计。[uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:83)

`0x29` 单轴写命令由 `pack_write_register_cmd(ID, 0x29, [target])` 生成，数据段长度为 5，完整命令帧为 **10 bytes**。[protocol.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py:119)

线速计算：

```text
T_byte = 10 / 921600 s = 10.8507 us
T_axis_TX = 10 × T_byte = 0.1085 ms
T_6axis_TX = 6 × T_axis_TX = 0.6510 ms
```

这是物理字节发送时间，不包含 `flush()`、USB/RS485 转向、设备处理和系统调度。

### 2.2 当前 request-response 路径

当前 `Motor.write_register()` 强制通过 `UartBus.request(..., retries=0)` 发送后等待一个状态响应；六轴 controller 在 ID1..ID6 顺序重复该流程。[motor.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor.py:70) [six_motor_controller.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:522)

每轴正常写响应是 20 bytes，因此原始六轴 TX+RX 字节线速为：

```text
6 × (10 + 20) bytes × 10 / 921600 = 1.9531 ms
```

但冻结 J4-F1A 同路径实机均值为 `18.39 ms`、P95 `20.05 ms`、最大 `23.89 ms`；响应等待和主机/转换器开销约束了实际周期，而非裸线速。[J4_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/frozen/J4_FINAL_SUMMARY.md:15)

## 3. Candidate Servo write-only 预算

### 3.1 纯线速和沿用帧间隔两种下限

`UartBus` 默认 `min_frame_interval=1 ms`，每次发送前若距上一帧不足该间隔会调用 `time.sleep()`。[uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:83) [uart_bus.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/uart_bus.py:174)

| 候选条件 | 六轴批次计算 | 理想批次时间 | 50 Hz 周期余量 |
|---|---:|---:|---:|
| 只计 6 个 TX 字节 | `6 × 0.1085 ms` | `0.6510 ms` | `19.3490 ms` |
| 沿用 1 ms 最小帧间隔 | `6 × 0.1085 + 5 × 1 ms` | `5.6510 ms` | `14.3490 ms` |

这里的 1 ms 间隔按当前实现的语义计算：第一帧立即发送，后续五帧在上一帧发送完成后至少等待 1 ms。实际 adapter 若还需要收发方向切换或明确 RX 丢弃窗口，必须把这些时间加回预算。

### 3.2 ID1-ID6 TX 时间偏差

以批次起点 `t0` 为基准，忽略额外 OS 抖动：

| 轴 | 无人工帧间隔的 TX start | 沿用 1 ms 间隔的 TX start |
|---|---:|---:|
| ID1 | `t0` | `t0` |
| ID2 | `t0 + 0.1085 ms` | `t0 + 1.1085 ms` |
| ID3 | `t0 + 0.2170 ms` | `t0 + 2.2170 ms` |
| ID4 | `t0 + 0.3255 ms` | `t0 + 3.3255 ms` |
| ID5 | `t0 + 0.4340 ms` | `t0 + 4.4340 ms` |
| ID6 | `t0 + 0.5425 ms` | `t0 + 5.5425 ms` |

因此候选的最坏 axis TX start skew 为：

- 仅受 UART 字节序列限制：**`0.5425 ms`**；
- 沿用当前 1 ms 帧间隔：**`5.5425 ms`**。

这只是发送端时间偏差；电机内部何时锁存目标、是否对 0x29 回包、以及执行器内部插补均未由仓库代码证明。

### 3.3 当前与候选对比

| 项目 | Current `0x29 + response` | Candidate Servo `0x29 write-only` |
|---|---|---|
| 单轴主机事务 | 写 10 B，读 20 B，等待解析 | 写 10 B，返回，不等待解析 |
| 六轴裸线字节 | 180 B TX+RX | 60 B TX |
| 裸线时间 | 1.9531 ms（不含等待） | 0.6510 ms |
| 沿用 1 ms 帧间隔 | 还受每轴 response 处理影响 | 理想约 5.6510 ms |
| 历史实机证据 | 18.39 / 20.05 / 23.89 ms | 无六轴实测证据 |
| 每轴交付确认 | 有（response） | 无；必须记录 delivery-uncertain |
| 失败策略 | 当前 controller 异常后 pause/resync | 单批次失败即停止写流并 pause；禁止运动帧重试 |
| 反馈来源 | 写 response 直接构成 snapshot | 独立 feedback 窗口 |

Candidate 的收益是移除六次 response 等待；风险是“TX 完成”不等于“电机已接收/执行”，且六轴中途失败时会产生部分轴更新。

## 4. Write-only 的 RX 回包问题

### 4.1 参考项目的实际语义

`inspire_motor_uart` 定义 `ControlMode.SERVO = 1`，并明确要求控制器以至少 50 Hz 发送目标位置。[inspire protocol.py](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/protocol.py:49)

其 `send_servo_position()` 调用 `write_register(..., read_response=False)`，只执行 `_send_command(frame)`，不读取响应。[inspire motor_controller.py](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/motor_controller.py:388) [inspire motor_controller.py](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/motor_controller.py:296)

但该参考的 `_send_command()` 每次发送前会 `reset_input_buffer()`，随后 `write()` + `flush()`；它没有证明电机不回包，而是通过下一次发送前清空 RX 来丢弃潜在回包。[inspire motor_controller.py](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/motor_controller.py:178)

参考节点是单电机：Servo 线程每 `time.sleep(0.02)` 发送一次，后台状态 timer 每 1 s 读一次。[inspire motor_node.py](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/motor_node.py:245) [inspire motor_node.py](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/motor_node.py:147)

因此可确认“单电机参考路径不等待响应”，但不能直接推出“六电机可以安全忽略所有响应”。六轴设计必须明确以下一种协议语义并在 Mock/后续受控验证中证明：

1. 电机在 Servo `0x29` 下确实不产生响应；或
2. 总线适配器在每个 TX 后有确定的回包丢弃/排空策略，且不会把残留帧误认作 feedback；或
3. 运动帧与响应帧采用独立、可验证的调度边界。

仅仅在 `UartBus.request()` 外层“不调用 read”是不充分的，因为上一帧响应可能留在 RX 缓冲；下一次读操作必须知道如何处理它。

## 5. Feedback 窗口设计

### 5.1 负载预算

当前六轴完整状态扫描历史均值约 `18.18 ms`（READ_ONLY），扫描跨度约 `15.18 ms`；该值属于六轴 `0x30` 读状态路径，不是裸线时间。[J5_FINAL_TEST_REPORT.md](/home/lyapunov/bone_drill/agent_docs/frozen/J5_FINAL_TEST_REPORT.md:54)

若每 20 ms Servo 周期同时执行一次完整六轴 feedback scan，则保守批次预算是：

```text
Servo TX-only 约 5.651 ms + full feedback scan 约 18.18 ms
= 23.831 ms > 20 ms
```

所以**不能把完整六轴 feedback scan 放进每一个 50 Hz 周期**，也不能假设 10 Hz full scan 会自动落在“空闲窗口”而不影响某个 20 ms deadline。

### 5.2 可审查的候选窗口

| 方案 | 设计 | 预算影响 | 评价 |
|---|---|---|---|
| A：低频整批反馈 | Servo TX 50 Hz；每 100 ms 做一次六轴 full scan | 5 次 TX 批次 + 18.18 ms scan 在 100 ms 内平均约 46.4 ms；但 scan 本身会占用近一个 20 ms 槽 | 平均总线负载可行，严格无丢周期不可保证 |
| B：逐轴轮询 | 每个 20 ms 周期保留 Servo TX，再只读一个轴；6 周期完成一轮 | 单轴 response 的真实时间需实测；理论上比整批 scan 更易塞入 20 ms | 更适合保持 50 Hz TX，但反馈全轴更新约 8.33 Hz |
| C：位置-only 反馈 | 用较短的 `0x31/0x2A` 读代替 full status | 减少 RX 字节，但仍有设备处理和锁开销 | 需要确认安全所需 fault/current 是否可由低频 full status 补齐 |
| D：独立反馈线程无锁并发 | Servo 与 feedback 同时访问 UART | 单总线不允许真实并发，最终仍需仲裁 | 不推荐；会制造帧交错/残留响应风险 |

方案 A 的“平均可行”不能等同于“每 20 ms deadline 可行”：如果 full scan 连续占用 18.18 ms，期间最多只能插入很少的 Servo TX，至少会产生一个周期级调度扰动。方案 B 将反馈工作摊平，代价是单个轴的更新频率与全轴一致性下降。

### 5.3 推荐的设计边界（仍未实现）

在不改变 J6 frozen baseline 的前提下，J7 设计应满足：

1. 单一 UART writer；Servo TX 和 feedback read 由同一个调度器串行仲裁。
2. Servo 周期先保留六轴 write-only 预算，再把反馈放入明确剩余窗口；不得用两个线程同时碰串口。
3. feedback 必须区分“配置/安全 full status”和“位置-only 观测”；每类都记录 freshness 和 deadline。
4. 若 feedback 无法在一个周期完成，宁可跳过该次 feedback，也不得隐式跳过 Servo TX 或发送陈旧目标。
5. write-only 运动帧不重试；反馈错误或写入错误触发停止写流和一次 pause 路径。

## 6. 六轴同步和运动影响

### 6.1 轴间 skew 的确定范围

在无硬件锁存的顺序 UART 架构中，ID6 相对 ID1 的目标到达偏差至少包含 TX 序列时间：

- 理想无额外帧间隔：`0.5425 ms`；
- 沿用 1 ms 间隔：`5.5425 ms`；
- 实际值还要加 OS scheduling、USB/RS485 发送完成和设备接收/锁存差异。

当前 request-response 路径的轴间完成时间还会叠加每轴 response 处理；已有批次均值不能反推出每轴偏差，必须从 per-axis timestamp 原始记录测量。

### 6.2 对 6PUS 运动的可计算影响

若上层某轴目标变化速度为 `v` steps/s，则单纯由 TX skew 造成的目标时间差上界可写成：

```text
Δq_skew = v × T_skew
```

以沿用 1 ms 间隔的 `5.5425 ms` 为例：

- `v = 50 steps/s` 时，约 `0.277 steps`；
- `v = 200 steps/s` 时，约 `1.109 steps`。

按工程约定 `1 step = 0.025 mm`，这分别约为 `0.0069 mm` 和 `0.0277 mm` 的目标时间差等效量。它只是几何/时间换算，不是实际平台误差或振动实测。

如果六轴目标变化速度更高，或总线发生额外抖动/部分批次失败，偏差按同一公式线性增加。Servo 电机内部插补可能减小可见阶梯，也可能因目标更新缺失产生滞后；仓库代码没有提供电机内部响应证据，不能在 C2.2 结论中替硬件下判断。

## 7. J7-C3 六电机硬件实验条件判断

### 已满足的设计输入

- J7-C1 Mock 已验证 mode=1 配置、50 Hz 六轴 write-only 调度、无响应等待、无运动帧重试、watchdog 和 pause 语义。
- 协议帧长度、921600 baud 和当前六轴 response 实测预算已知。
- 单电机参考明确了 Servo mode=1 和 `read_response=False` 的软件调用形式。

### 尚未满足的必要条件

1. **没有六轴 write-only transport 实现/Mock**：当前生产 `UartBus`/`Motor` 仍是 request-response 语义，不能直接用于候选架构。
2. **没有 RX 回包处置证明**：参考项目通过清空 RX 缓冲工作，但六轴残留响应、帧边界和反馈窗口尚未验证。
3. **没有 feedback 调度实测**：full scan 的 18.18 ms 历史值与 5.651 ms TX 下限组合后会挤压 20 ms 周期；逐轴轮询或低频整批策略尚未实现和验证。
4. **没有 J7-C2 单电机硬件门槛**：mode=1 写入、连续目标接收、停止和失联行为尚未获得真实设备证据。
5. **没有本阶段独立硬件授权**：当前任务明确禁止连接和发送运动命令，不能把“六轴已连接”当作“Servo 实验已获准”。

### 判断

**J7-C3 当前不具备直接执行条件。**

从时间预算看，write-only 六轴 50 Hz 是“名义上有余量”的候选架构；从工程验证看，必须先完成隔离 transport/feedback Mock、单电机受控验证和明确安全授权，再重新审查 C3。C3 不能以 J6 mode=0 frozen baseline 的证据替代，也不能把理论 `5.651 ms` 当作真实硬件结果。

## 8. 推荐的只读后续审查项

1. 在 Mock 中加入“每个 `0x29` 仍产生未读取 response”的模型，验证 RX 排空、下一次读帧匹配和 stale response 防护。
2. 记录 write-only 候选调度的 `cycle_start`、每轴 TX start/end、batch complete、RX discard、feedback start/end、cycle jitter 和 deadline miss。
3. 用同一 UART 时间轴比较方案 A/B/C 的 Servo TX 丢周期数量和 feedback age；不得只看平均吞吐。
4. 只有在上述软件证据通过后，才讨论 J7-C2 单电机硬件授权；六电机 C3 另行审批。

本阶段到此结束；未实施任何架构变更。
