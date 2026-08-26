# J7-C8 六轴 Servo 实机前代码审查

## 1. 审查结论

当前 J7 C5/C6/C7 代码**尚未具备直接连接真实 `UartBus` 并安全执行首次六轴 Servo 运动的条件**。

已具备：

- C5 六轴 ID1->ID6 write-only `0x29` batch、mode=1 配置/readback、RX residual 接口和停止状态；
- C6 round-robin/full-scan feedback、ID/command 校验、residual/mismatch/malformed/timeout 审计；
- C7 配置门禁、VERIFY、PARK/HOME 目标、watchdog 和一次性 stop 状态机；
- 纯 Mock 回归证据。

尚缺：

- 真实 `UartBus` 到 C5/C6 `SixAxisServoBus` 接口的隔离适配层；
- 真实 write-only 发送和非阻塞 RX 读取的实现与验证；
- 可执行六轴 `0x1A` stop 的真实 pause callback；
- 在上述适配和 stop 门禁完成前，不得打开串口或进行实机运动。

## 2. 真实 UART backend 注入链路

### 当前 runner 的注入方式

`J7ServoExperimentRunner` 的构造函数接收两个已经实例化的对象：

```text
caller
  -> J7SixAxisServoTransport(bus=...)
  -> J7ServoFeedbackScheduler(writer=transport)
  -> J7ServoExperimentRunner(transport, feedback)
```

runner 本身不创建串口、不解析 port/baudrate，也不把 `UartBus` 自动转换为 J7 bus。C5 要求注入对象提供：

- `request(frame, expected_id, expected_command, retries)`；
- `write(frame)`；
- `read_available()`；
- C5 的 `uart_lock` 作为共享仲裁边界。

### 与现有 `UartBus` 的差异

现有 `uart_bus.py:75-105` 的 `UartBus` 提供阻塞 `request()` 和内部 RLock，但没有公开的 write-only `write()` 或非阻塞 `read_available()` 接口。其 `_send()`（`uart_bus.py:181-208`）会在每次 request 前清空 RX，并且 `request_frames()`（`uart_bus.py:250-330`）默认允许 retries。

因此当前不能把 `UartBus` 实例直接传给 C5：

1. C5 的 `send_cycle()` 调用 `bus.write()`，现有 `UartBus` 没有该公开方法；
2. C6 的 residual 管理需要 `read_available()`，现有 `UartBus` 没有该接口；
3. `UartBus` 的 request 前 RX 清空和阻塞 response 语义会破坏 write-only 残留分析；
4. C5/C6 的共享锁是 `writer.uart_lock`，不能只依赖一个未与 writer 绑定的外部锁。

## 3. C5/C6 连接真实 UART 所需最小适配

需要新增隔离的 J7 backend/adapter，不能通过修改冻结的 `protocol.py`、`uart_bus.py` 或 `motor.py` 取得“直接可用”结论。适配层至少应：

1. 使用批准的单一物理 UART、`921600 8N1`、有界读写 timeout；
2. 提供 C5/C6 所需的 `request`、write-only `write`、`read_available` 和同一把共享锁；
3. 配置/readback request 固定 `retries=0`，并验证 ID、command、register；
4. 运动 `0x29` 只写一次，不等待 response，不 retry；
5. 维护 RX parser/inbox，使 residual、stale response 和 feedback response 不被两个独立缓冲区重复消费；
6. 提供实际 pause callback 所需的六轴 `0x1A` 停止事务和结果记录；
7. 在 Mock 中先验证短写、timeout、stale response、错误 ID/command、malformed frame 和 stop 失败。

适配层必须明确串口所有权：配置、Servo TX、feedback、RX 清理和 stop 都只能由同一 writer/arbiter 访问。不能同时运行 ROS2 J6 节点、普通 `UartBus` client 或其他串口程序。

## 4. 关键路径审查

### 4.1 mode=1 配置

C5 `configure_servo()`（`j7_six_axis_servo_transport.py:210-264`）按 ID1-ID6：

```text
0x32 写 0x25..0x28: [mode=1, 0, 0, speed]
  -> response（retries=0）
  -> 读 0x25
  -> 读 0x28
  -> 核对 mode=1、speed
```

C7 runner 的 `CONFIG_SERVO` 只在六轴配置和 readback 全部通过后继续；配置异常进入 STOP，不发送 `0x29`。

**实机门禁**：适配层必须证明这些 request 没有被现有 `UartBus` 的默认 retry 或 RX 清空语义改变，并保存每轴 response/readback/fault 证据。

### 4.2 readback 与 VERIFY

C6 `_read_axis_locked()`（`j7_servo_feedback_scheduler.py:295-354`）对每轴执行：

```text
0x30 status -> position/current/fault
0x31 read 0x25 -> mode
```

C7 在 `CHECK` 做全轴健康扫描，在 `VERIFY` 再次要求六轴 `mode=1`、fault=0、无 timeout。该路径的软件语义完整，但真实 backend 尚未提供。

### 4.3 `0x29` write-only

C5 `send_cycle()`（`j7_six_axis_servo_transport.py:266-364`）具备：

- 固定 ID1->ID6 顺序；
- 六轴 batch audit、TX start/end、skew、batch complete、deadline；
- `response_wait=False`、`retry_count=0`；
- cycle 前 RX residual 策略处理。

**实机缺口**：实际 `bus.write()` 尚未由现有 `UartBus` 提供，且尚未证明执行器是否会在 write-only `0x29` 后产生 response。两者必须在适配层和 Mock 中先验证。

### 4.4 feedback 读取

C6 使用非阻塞 lock acquire；锁忙或 Servo deadline 临近时 defer，不执行 I/O。正常时按 round-robin 读取一轴，或按请求执行 full six-axis scan。

**实机缺口**：现有 `UartBus.request()` 是阻塞 response transaction，且默认 `retries=2`（除非调用者显式覆盖）。适配层必须确保 C6 的 `retries=0` 被实际执行，并把 response timeout 限制在批准的 feedback budget 内，否则 feedback 可能占用 Servo 周期。

### 4.5 RX residual

C5 维护 parser、residual frame/byte 上限和 `DISCARD_BEFORE_TX`、`DEFER_TO_FEEDBACK`、`REJECT_RESIDUAL` 策略；C6 在反馈前通过 writer residual 接口清理残留，并验证当前 response 的 ID/command。

**实机风险**：如果适配层继续调用 `UartBus._send()` 的“发送前清空 RX”，会丢失 write-only residual 证据；如果 C5 和适配层各自维护 RX 缓冲区，则可能重复消费或误匹配。必须采用单一 RX owner，并记录：

- stale response 数量；
- residual frame/byte 数量；
- 错误 ID/command；
- malformed/checksum/overflow；
- feedback response mismatch。

## 5. stop 机制审查

### 当前行为

- C5 `transport.stop(reason)` 只将 transport 置为单向 stopped，并调用可选 `pause_callback` 一次；
- C6 异常时调用 writer stop；
- C7 runner 的 `stop()` 只调用 transport stop 一次，重复 stop 不重复调用 callback；
- 当前默认构造没有真实 pause callback，也没有 C5 内置的 `0x1A` 六轴写入。

因此当前 stop 机制是**状态和回调框架**，不是已完成的真实电机停止命令。

### 是否需要真实 stop command

需要。第一次真实运动前，pause callback 必须绑定到经过 Mock 验证的六轴停止事务，至少满足：

1. 由同一 UART writer 按 ID1-ID6 发送批准的 `0x1A`；
2. 停止路径不 retry 运动帧，也不继续发送陈旧目标；
3. 记录每轴 pause TX、response/timeout、失败轴和最终停止结果；
4. callback 只执行一次，异常时转入人工急停/断电流程；
5. watchdog、写失败、feedback timeout/mismatch、fault/current 越限都能到达该路径。

在真实 pause callback 尚未实现和 Mock 验证前，C7 runner 的 STOP 不能作为实机安全停止保证。

### 第一次运动前最低安全要求

- 独立 J7 backend 已通过无硬件 Mock 回归；
- 真实 pause/急停路径已由现场负责人确认，且至少一次静态 stop dry-run 不发送运动目标；
- E-stop 可达、限流电源、机械防护和双人观察就位；
- 当前没有 ROS2 J6 runner、普通 `UartBus` client 或其他串口占用；
- 六轴 ID/物理映射、位置范围、fault/current/mode 读取正常；
- watchdog、feedback age、RX residual 和通信 timeout 阈值已写入实验记录；
- 明确任何一轴异常都立即停止全轴，不通过重试或跳过轴继续运行。

## 6. 第一次真实实验流程设计

以下是执行前方案，不是本阶段执行记录。

### Phase 0：静态检查（只读）

**进入条件**：硬件授权、两名观察者、E-stop、限流电源和机械包络检查完成；仅 J7 backend 占用 UART。

**动作**：

- 只读确认 ID1-ID6 与支链映射；
- 读取并记录 mode、fault、current、position、temperature（若可读）；
- 确认 RX 初始为空或残留已分类；
- 确认没有目标流、没有陈旧目标、watchdog 尚未触发；
- 确认实际位置处于批准的静止范围，不能假定上电位置就是 PARK。

**禁止**：不发送 `0x29`，不执行 PARK/HOME，不调用 J6 runner。

**通过条件**：六轴 ID 唯一、反馈完整、fault=0、current 在批准阈值内、mode 状态可解释、无通信/残留异常。

### Phase 1：mode=1 配置

**进入条件**：Phase 0 PASS，pause callback 和人工急停路径已确认。

**动作**：

- 按 ID1-ID6 写 `0x25=1` 和批准 speed；
- 每轴等待并校验配置 response；
- 读取 `0x25`/`0x28` readback；
- 读取 fault/current/position，记录配置前后状态；
- 六轴全部通过后才进入 VERIFY/ARMED。

**失败条件**：任一轴写失败、readback 不一致、fault/current 异常、RX mismatch 或 timeout。失败后立即停止配置流程，按真实 pause/E-stop 流程收尾，禁止任何 `0x29`。

### Phase 2：小范围 Servo 运动

**进入条件**：Phase 1 全轴 `mode=1`、VERIFY PASS、runner ARMED，实际位置与小范围起点一致，且机械方向/间隙已确认。

**动作**：

- 使用六轴同目标、小范围段，例如在现场确认起点后的 `500 -> 550` steps；不得把 500 当作未经反馈确认的当前位置；
- 以 50 Hz write-only `0x29` stream 发送同一目标向量；
- C6 按 round-robin 反馈，按批准周期安排 full safety scan；
- 实时记录 TX skew、cycle jitter、feedback age、position/current/fault/mode、residual 和 watchdog。

**立即停止**：任一 fault/current/位置差异常、feedback timeout/mismatch、RX overflow、deadline 连续 miss、target watchdog、异常噪声/卡滞或观察者急停。

**通过条件**：六轴正确响应并在批准位置/同步/电流窗口内稳定停止，且全套日志完整。

### Phase 3：PARK -> HOME

**进入条件**：Phase 2 PASS；现场再次确认无机械干涉、无负载异常和 E-stop 可达。

**动作**：

1. 只读确认六轴实际位置已经是 PARK `[0]*6` 或批准的安全起点；
2. 发送 PARK 同目标保持窗口并确认稳定；
3. 发送 HOME `[1000]*6` 的 50 Hz write-only stream；
4. 到达批准 HOME 位置窗口后保持并采集，再停止写流。

PARK/HOME 的 raw 值必须以实际反馈为准，不能仅依赖软件目标。任一轴未到达、反馈缺失或安全条件触发时，立即执行真实 stop/E-stop，不继续扩大行程。

## 7. C8 放行结论

当前结论：**NOT READY FOR REAL MOTION**。

在以下事项完成并通过新的 Mock/代码审查前，不得进入真实六电机 Servo：

1. 隔离真实 UART backend/adapter；
2. 单一 RX owner 和 residual 归属策略；
3. 真实六轴 pause/stop command 及失败收尾；
4. bounded feedback timeout 与 50 Hz deadline 复核；
5. 实机前 Phase 0 静态检查和现场安全门禁确认。

本报告不授权打开串口、连接硬件或发送动作命令，也不改变 J6 frozen baseline。
