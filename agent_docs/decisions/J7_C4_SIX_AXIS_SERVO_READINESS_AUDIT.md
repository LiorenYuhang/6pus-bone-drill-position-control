# J7-C4 六轴 Servo 实机前代码就绪审查

## 1. 审查结论

当前代码**尚未达到第一次六轴 Servo 实验的代码就绪状态**。

已存在的是两部分能力：

1. 现有 ROS2/`SixMotorController` 的六轴目标输入、六轴顺序 request-response 写入、通用 mode/speed 配置和暂停路径；
2. J7-C2.4 的隔离 `J7SingleAxisServoTransport`，可对单轴执行 `mode=1` 配置、readback 和 write-only `0x29` Mock/未来单电机适配。

二者尚未连接成六轴 Servo 实验路径。C1 测试中的六轴 scheduler 是测试文件内的 Mock 实现，不是可用于真实六轴硬件的 transport 或入口。

## 2. 六轴 Servo 目标流入口

### 已具备

- `motor_driver_node.py:243-265` 已有六轴目标订阅入口：`/pus6/command_joint_steps` 或 `/pus6/command_joint_positions`。
- `motor_driver_node.py:350-382` 已有单槽 Latest-Value-Wins、覆盖计数和目标时间戳。
- `motor_driver_node.py:302-307` 默认创建 50 Hz worker timer。
- `motor_driver_node.py:513-645` 已有六轴 worker、目标年龄、deadline 统计和 watchdog 检查。

### 缺少

- 这些入口最终调用的是 `SixMotorController.command_positions_with_metrics()`，不是 J7 write-only transport；每个 `0x29` 仍通过 `Motor.write_register()` 等待 response，并把响应当作状态反馈。
- 没有标识为 J7 Servo 的独立目标流入口，也没有把一个 cycle 明确定义为 ID1-ID6 的 write-only batch。
- 没有六轴 Servo 实验启动/arm 门禁入口，无法在目标流启动前强制完成六轴 `mode=1`、speed 和 readback 全部通过。

### 最小需要实现内容

- 新增隔离的 J7 六轴 target-stream/scheduler 入口，接收长度为 6 的 raw step 目标并采用 latest-value-wins。
- 将每个周期绑定 `cycle_id`、目标时间戳、目标年龄、batch 完成时间和 deadline 结果。
- 在进入运动流前强制执行六轴配置 readback gate；配置未全部通过时禁止任何 `0x29`。
- 保持 J6 runner 和现有 ROS2 节点不变，首个硬件实验应使用独立 J7 实验入口。

## 3. Servo transport：六轴顺序、batch 和单 UART writer

### 已具备

- `j7_servo_transport.py:150-272` 的 `J7SingleAxisServoTransport` 有线程锁、最小帧间隔和 write-only `write_only_position()`。
- `j7_servo_transport.py:272-321` 明确 `response_wait=False`、`retry_count=0`，写失败进入停止状态。
- `six_motor_controller.py:522-541` 的现有路径可按 `MOTOR_IDS=(1..6)` 顺序访问六轴，并用共享 bus lock 串行 request-response。
- `uart_bus.py:75-80,118-163` 的普通 `UartBus` 只拥有一个物理 UART，并用 RLock 串行交易。
- C1 Mock 的 `J7ServoScheduler`（`test_j7_c1_servo_mock.py:195-310`）验证过 ID1-ID6、50 Hz、batch audit 和写失败停止语义。

### 缺少

- J7 transport 类名和实现都明确为 **single-axis**；`j7_servo_transport.py:150-156` 没有六轴对象、ID 顺序循环或 batch API。
- J7 的锁是每个 transport 实例自己的锁；创建六个实例不能证明它们共享同一个 writer/锁，也不能防止反馈线程绕过 writer 写 UART。
- `UartBus` 没有 J7 所需的公开 write-only `write()` 与非阻塞 `read_available()` 接口。现有 `request()` 会发送后等待 response，不能直接作为 J7 write-only backend。
- C1 的六轴实现只在 Mock 测试文件中，不能被当前生产包或硬件 runner 调用。

### 最小需要实现内容

- 新增 `J7SixAxisServoTransport` 或等价隔离模块：持有一个共享 UART writer、固定 ID1->ID6 顺序、一个 cycle 内六次 `0x29` write-only，并输出每轴 TX start/end、skew 和 batch complete。
- 提供专用 backend/adapter，使 J7 writer 能在不改变冻结 `protocol.py`、`uart_bus.py`、`motor.py` 的前提下获得单一 UART 的 `write`、`read_available`、配置 request/response 和共享锁能力。
- 明确 writer 所有权：配置、运动、feedback、pause 均通过同一调度器仲裁，禁止并行直接访问串口。

## 4. mode=1 配置和六轴 readback

### 已具备

- `j7_servo_transport.py:209-268` 的 `configure_servo()` 能对**单个 motor_id** 写 `0x25` 的 mode=1 与 `0x28` speed，并逐项 readback；配置 request 使用 `retries=0`。
- `six_motor_controller.py:437-471` 的 `configure_all(mode, speed)` 能按 ID1-ID6 写 `0x25..0x28`，再逐轴读取 `0x25` 和 `0x28` 验证。直接传入 `mode=1` 在该函数层面是可行的。

### 缺少

- ROS2 的 `configure_motors` 服务在 `motor_driver_node.py:855-870` 调用无参数的 `configure_all()`，实际默认仍是 `mode=2, speed=50`，不是 J7 Servo 配置入口。
- `configure_all(mode=1)` 属于现有 request-response 控制器路径，不会建立 J7 write-only transport、RX residual 策略或六轴 Servo arm 状态。
- 没有 J7 专用的“六轴全部配置成功后才 arm”的持久状态，也没有在运动过程中监视 mode 是否仍为 1。

### 最小需要实现内容

- 在隔离 J7 六轴 transport 中逐轴执行 `0x25=1`、批准 speed 写入和 `0x25/0x28` readback；任一轴失败即禁止 arm。
- 记录每轴配置响应、readback、fault 和位置快照，并提供全轴配置结果给启动门禁。
- 运动期间增加低频 mode/fault 健康检查；发现任一轴不再是 mode=1，立即停止 write stream。

## 5. RX residual 安全策略

### 已具备

- `j7_servo_transport.py:330-386` 有 parser、残留 frame 队列、最大 frame/byte 限制和 malformed/overflow 失败处理。
- `ServoRxPolicy` 支持 `DISCARD_BEFORE_TX`、`DEFER_TO_FEEDBACK`、`REJECT_RESIDUAL`；`j7_servo_transport.py:387-420` 会记录残留审计。
- C2.4 Mock 已验证残留、reject、overflow 和 write failure 时不 retry 运动帧。
- 普通 `UartBus` 在 request 前清空输入 buffer，并对 request-response 做 ID/command 匹配；这只覆盖当前阻塞交易。

### 缺少

- 当前 residual 队列隶属于一个 `J7SingleAxisServoTransport`；没有一个共享六轴 RX demultiplexer 按 motor ID/command 归属残留帧。
- 没有证明六轴连续 `0x29` write 后产生的六轴 response 能与后续 `0x32`/feedback response 安全区分；也没有 batch 级 stale/mismatch 统计。
- 现有 `UartBus` 的清空策略可能直接丢弃 write-only 运动残留，且没有向 J7 audit 暴露 residual 数量、来源 ID 或 malformed 记录。

### 最小需要实现内容

- 新增共享 RX 管理器：在每个运动 cycle 前后 ingest 可用字节，解析并按 ID/command 分类，限制残留数量/字节数。
- 为 stale response、未知 ID、错误 command、malformed frame、overflow 和 feedback mismatch 定义明确的 discard/defer/reject 行为及计数。
- feedback transaction 必须只接受当前期望 ID/command，旧 `0x29` response 不得被当成位置或状态反馈；无法安全归属时进入停止路径。

## 6. Feedback 调度

### 已具备

- `six_motor_controller.py:329-430` 已有六轴完整状态 scan 和位置-only scan，按 ID1-ID6 顺序 request-response。
- `motor_driver_node.py:311-318,685-747` 已有低频 feedback timer；默认 poll rate 为 10 Hz，并在 active target stream 时跳过 standalone poll 以减少 UART 竞争。
- 现有 telemetry 已包含位置、电流、温度、force 和 fault 字段。

### 缺少

- `j7_servo_transport.py:118-126,403-414` 的 `feedback_request()` 明确是 `FeedbackRequestPlaceholder`，`implemented=False`，不执行任何 I/O。
- 没有与 50 Hz write-only cycle 共享 writer 的 feedback scheduler，没有 ID1-ID6 round-robin、100 ms full safety scan、feedback age 或 defer 统计。
- 当前 worker 把 `0x29` response 当作六轴反馈；切换为 write-only 后该来源消失，不能继续使用现有 `SixAxisCommandResult.to_state_snapshot()` 作为反馈。
- 当前 poll 在 active stream 时整体跳过，不能满足 J7 需要的“运动期间仍有可解释的安全反馈窗口”。

### 最小需要实现内容

- 实现一个由单 UART writer 仲裁的反馈调度器：建议每个 Servo cycle 最多轮询一个轴，ID1-ID6 round-robin，并周期性安排 full six-axis safety scan。
- 每个反馈样本记录轴 ID、采样时间、位置/current/fault/mode、feedback age、defer 和 timeout；反馈不可用时不阻塞 50 Hz writer。
- 把反馈快照与 command cycle 分离，禁止把 write-only motion TX 当作反馈证据。

## 7. Watchdog 与 stop 路径

### 已具备

- `motor_driver_node.py:513-565` 有目标流超时检查；超时后停止重复发送陈旧目标，并可调用 `controller.pause_all()`。
- `SixMotorController.command_positions_with_metrics()` 在 request-response 写失败时调用 `pause_all()`；`pause_all()`（`six_motor_controller.py:811-829`）按六轴写 `0x1A`。
- C1 Mock 已验证 watchdog、写失败和显式 graceful stop 的一次性 pause 语义。
- `J7SingleAxisServoTransport.stop()`（`j7_servo_transport.py:405-420`）有单向 stopped 状态和一次 pause callback。

### 缺少

- J7 transport 的 stop 只调用外部 callback，本身不执行六轴 `0x1A`，也没有六轴 pause 结果、失败轴和停止完成审计。
- 现有 node watchdog/stop 与 request-response controller 绑定，尚未接到 J7 write-only batch 的 delivery-uncertain、RX residual、deadline miss 和反馈异常状态。
- 没有六轴 stop 的一次性状态机，无法保证某一轴写失败后立即停止后续轴并对已发送/未发送轴统一收尾。

### 最小需要实现内容

- 在 J7 六轴 scheduler 中实现 one-way stop 状态机：目标超时、写失败、RX/feedback 错误、mode/fault 异常或 deadline miss 达到阈值时，停止陈旧目标并通过同一 writer 对六轴执行一次批准的 pause/stop。
- 记录每轴 pause 尝试和结果、停止原因、最后完整 cycle、delivery uncertainty；停止后禁止自动 retry 或自动恢复。
- 将 watchdog、writer、feedback 和 pause 置于同一 UART 仲裁边界，避免停止命令与残留反馈交错。

## 8. 最小实现清单（不在本阶段执行）

第一次六轴 Servo 实验前，至少还需要以下隔离实现：

1. 六轴 J7 Servo transport/writer：ID1-ID6 顺序、50 Hz batch、write-only `0x29`、TX/batch audit；
2. 六轴 mode=1/speed 配置和全轴 readback/arm gate；
3. 共享 UART backend 与六轴 RX residual demultiplexer；
4. round-robin position feedback + 低频 full safety feedback 调度；
5. 六轴 target watchdog、deadline/error stop、一次性 pause 状态机；
6. 独立 J7 实验入口和 Mock 回归测试，覆盖六轴配置失败、残留 response、feedback mismatch、单轴写失败、watchdog 和 graceful stop；
7. 完整审计输出：cycle jitter、target age、TX skew、feedback age、residual、overwrite、deadline miss、watchdog 和 stop reason。

上述内容完成并通过 Mock 验证后，才可重新进行实机前门禁评审。完成 C4 不等于授权打开串口或执行运动。

## 9. J6 frozen baseline 保护

本审查未修改 J6 standalone HOME/PARK、`mode=0` baseline、现有 ROS2 控制链路或底层协议基线。任何 J7 六轴实现都必须保持隔离，不得通过改变 `configure_all()` 默认 mode、复用 J6 runner 或修改冻结通信模块来获得“就绪”结论。
