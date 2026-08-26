# J7-C6 六轴 Servo Feedback Scheduler 实现记录

## 1. 实现范围

本阶段新增独立的六轴 Servo feedback scheduler，用于 J7 writer 的软件 Mock 验证。

未修改：

- `protocol.py`；
- `uart_bus.py`；
- `motor.py`；
- 现有 ROS2 节点；
- J6 frozen baseline；
- 电机参数和硬件配置。

未连接真实硬件、未打开串口、未发送真实运动命令。

## 2. 新增/变更文件

- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_servo_feedback_scheduler.py`
  - `J7ServoFeedbackScheduler`：单轴 round-robin feedback；
  - `full_scan()`：低频 ID1-ID6 full safety scan；
  - 非阻塞获取 C5 writer 共享 UART lock；锁忙或 Servo deadline 临近时只记录 defer，不执行 I/O；
  - 逐轴读取 status（位置/current/fault）和 `0x25` mode；每个 request 显式 `retries=0`；
  - 记录 feedback cycle、sample timestamp、feedback age、timeout、defer、RX residual、stale/mismatch/malformed 计数；
  - timeout、response mismatch、malformed RX 或 residual 无法安全分类时调用 writer 的 one-way stop。

- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py`
  - 仅增加 `uart_lock` 公开访问器，使 feedback scheduler 与 C5 writer 使用同一锁边界；
  - 原有六轴 write-only 逻辑未改变。

- `bone_ws/src/pus6_motor_driver/test/test_j7_c6_feedback_scheduler.py`
  - 新增纯 Mock bus 测试，不打开串口；
  - Mock 的 `hardware_write_count` 保持为 `0`。

## 3. 调度行为

### Round-robin

普通 `poll_once()` 每次只读取一个轴，顺序为 ID1 -> ID2 -> ID3 -> ID4 -> ID5 -> ID6，再循环。每轴读取两笔 request-response：

1. `0x30` status：position/current/fault；
2. `0x31` read register `0x25`：mode。

反馈读取不会使用 `0x29` 运动写入的 response 作为状态来源。

### Full scan

`full_scan()` 在同一共享 UART lock 下按 ID1-ID6 顺序读取六轴。scheduler 也支持按 `full_scan_period_s` 判断低频 full scan 到期。

### Servo deadline 保护

- lock 使用非阻塞 acquire；writer 正在发送时，feedback cycle 记录 `deferred=True`；
- 若当前时间进入 Servo deadline guard 窗口，同样 defer；
- feedback transaction 不执行 retry；
- 实际 bus backend 仍必须提供有界的 response timeout，不能将无限阻塞 backend 接入真实实验。

## 4. RX residual 与错误处理

- feedback transaction 前先通过 C5 writer 的 residual 接口清理 writer 已接收的残留；
- 再扫描 bus 可用字节，残留 frame 只计为 stale/residual，不作为当前 feedback response；
- 当前 response 必须同时匹配期望 motor ID 和 command；
- 错误 ID/command：记录 mismatch，进入 `feedback_mismatch` stop；
- 无法解析的 frame：记录 malformed，进入 `malformed_feedback_response` stop；
- response timeout 或 budget 超限：记录 timeout，进入 `feedback_timeout` stop；
- stop 状态由 writer 单向锁定，pause callback 只触发一次。

## 5. Mock 验证结果

执行命令：

```text
pytest -q bone_ws/src/pus6_motor_driver/test/test_j7_c5_six_axis_servo_transport.py \
       bone_ws/src/pus6_motor_driver/test/test_j7_c6_feedback_scheduler.py
```

结果：

```text
............                                                             [100%]
12 passed in 0.06s
```

C6 新增测试覆盖：

1. round-robin ID1-ID6 feedback 及 status/mode 顺序；
2. full six-axis scan；
3. writer 持锁时 feedback defer，不阻塞 Servo 调度；
4. write-only `0x29` stale response 不误匹配为 feedback；
5. 错误 ID response 触发 mismatch stop；
6. timeout 触发安全停止且不 retry；
7. malformed response 触发安全停止。

上述均为软件 Mock 结果，不能证明真实 UART feedback 延迟或实机运动安全性。

## 6. 当前限制

- scheduler 未集成 ROS2、J6 runner 或现有 `SixMotorController`；
- 未提供真实串口 backend，也未验证真实电机是否在 write-only `0x29` 后返回残留；
- feedback 只实现 position/current/fault/mode，未加入 temperature/force 等扩展状态；
- scheduler 只提供暂停 callback，不自行实现真实 `0x1A` 发送；
- 未执行 PARK/HOME、轨迹生成、六轴同步运动或任何硬件动作；
- Mock 的 50 Hz writer 与 feedback 时间预算不等价于真实 UART deadline。

本阶段完成后停止，不进入硬件测试，也不改变 J6 frozen baseline。
