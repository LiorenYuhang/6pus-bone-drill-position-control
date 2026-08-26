# J7-C5 六轴 Servo Transport 实现记录

## 1. 实现范围

本阶段新增了隔离的六轴 Servo transport，仅用于 J7 后续实验准备和 Mock 验证。

未修改：

- `protocol.py`；
- `uart_bus.py`；
- `motor.py`；
- 现有 ROS2 节点；
- J6 frozen baseline；
- 任何电机参数或硬件配置。

未打开串口、未连接硬件、未发送真实运动命令。

## 2. 新增文件

- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py`
  - `J7SixAxisServoTransport`：单一注入 bus、单一锁、固定 ID1->ID6 顺序；
  - `configure_servo(speed=...)`：逐轴写 `0x25=1` 和 speed，并读取 `0x25/0x28` readback；
  - `send_cycle(...)`：固定 50 Hz 周期定义，六轴 write-only `0x29` batch；
  - `SixAxisTxRecord`、`SixAxisCycleAudit`：记录 cycle、目标时间戳、每轴 TX、skew、batch 完成和 deadline；
  - 保留 `ServoRxPolicy` 接口、RX residual 限制/记录、单向 stop 状态和 pause callback；
  - 运动帧明确 `response_wait=False`、`retry_count=0`。

- `bone_ws/src/pus6_motor_driver/test/test_j7_c5_six_axis_servo_transport.py`
  - 仅使用协议感知 Mock bus；
  - Mock 的 `hardware_write_count` 保持为 `0`，不代表真实硬件写入。

## 3. Mock 验证结果

执行命令：

```text
pytest -q bone_ws/src/pus6_motor_driver/test/test_j7_c5_six_axis_servo_transport.py
```

结果：

```text
5 passed in 0.06s
```

已覆盖：

1. 六轴逐轴 mode=1/speed 配置和 readback；
2. ID1-ID6 固定顺序的六轴 `0x29` batch；
3. 运动帧无 response wait、无 retry，并输出 cycle audit；
4. 上一 batch 的 RX residual 在 `DISCARD_BEFORE_TX` 策略下被记录并清理；
5. 单轴写失败时停止且不发送后续轴、不重试；
6. `REJECT_RESIDUAL` 策略在下一 batch 前停止；
7. 所有测试中的 `hardware_write_count=0`。

这些是软件 Mock 结果，不是实机通信或运动结果。

## 4. 当前限制

- 新 transport 只提供注入式 bus 接口，没有真实串口打开逻辑；
- 未集成 ROS2 目标订阅、J6 runner 或现有 `SixMotorController`；
- 未实现 feedback scheduler，`send_cycle()` 不读取运动 response；
- RX residual 只提供策略接口和统计，尚未在真实硬件上确认电机是否对 `0x29` 产生 response；
- stop 通过外部 pause callback 交给上层，不在本模块内实现真实 `0x1A` 写入；
- 未实现六轴 mode/fault/current 的运动期间反馈监控；
- 50 Hz 是调度周期契约，Mock 通过不等于真实 UART deadline 或机械同步通过；
- 本阶段不包含 PARK/HOME 动作、轨迹生成或任何自动恢复逻辑。

## 5. 单电机/六电机硬件实验前仍需完成

在任何真实运动授权前，还需要另行完成：

1. 经评审的真实 UART backend/adapter，确认单一 writer、配置 response 和 write-only TX 的资源所有权；
2. RX residual、stale response、feedback mismatch 和 buffer overflow 的硬件观测方案；
3. feedback 调度实现及位置/current/fault/mode 的 age/timeout 监控；
4. 六轴 watchdog、pause/stop 结果采集和现场急停门禁；
5. 独立的实机启动检查、Phase 0/Phase 1/Phase 2 门禁和数据采集流程；
6. 重新进行 J7-C4 代码就绪与实机授权审查。

本报告完成后停止，不进入硬件测试，也不改变 J6 frozen baseline。
