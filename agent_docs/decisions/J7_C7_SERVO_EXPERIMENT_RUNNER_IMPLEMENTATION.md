# J7-C7 六轴 Servo Experiment Runner 实现记录

## 1. 实现范围

本阶段新增独立 J7 实验 runner，将 C5 六轴 write-only Servo transport、C6 feedback scheduler 和目标 watchdog 组合为一个软件 Mock 入口。

未修改：

- `protocol.py`；
- `uart_bus.py`；
- `motor.py`；
- 现有 ROS2 节点；
- J6 frozen baseline；
- 电机参数和硬件配置。

未连接真实硬件、未打开串口、未发送真实运动命令。

## 2. 新增文件

- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_servo_experiment_runner.py`
  - `J7ServoExperimentRunner`；
  - `RunnerState`：`INIT`、`CHECK`、`CONFIG_SERVO`、`VERIFY`、`ARMED`、`SERVO_RUN`、`STOP`；
  - `PARK_TARGET=(0,0,0,0,0,0)`；
  - `HOME_TARGET=(1000,1000,1000,1000,1000,1000)`；
  - 状态事件、Servo cycle、feedback cycle、overwrite、watchdog 和 stop audit。

- `bone_ws/src/pus6_motor_driver/test/test_j7_c7_servo_experiment_runner.py`
  - 纯 Mock bus 状态机测试；
  - Mock `hardware_write_count` 固定为 `0`。

## 3. 状态机与安全门禁

### INIT -> CHECK

执行六轴 full feedback scan，要求六轴反馈完整、无 timeout、fault 为 0。检查失败立即进入 `STOP`，不允许配置或运动。

### CHECK -> CONFIG_SERVO

调用 C5 `configure_servo(speed=...)`，按 ID1-ID6 写入 `mode=1`/speed，并等待每轴 readback。配置异常或覆盖不完整时进入 `STOP`。

### CONFIG_SERVO -> VERIFY

再次执行六轴 full scan，要求每轴 `mode=1`、fault=0、无 timeout。任何一轴失败都不能进入 `ARMED`。

### VERIFY -> ARMED

只有 transport 配置结果存在且全部门禁通过时允许 arm。此阶段不发送 `0x29`。

### ARMED -> SERVO_RUN

首次 `run_cycle()` 调用后进入 `SERVO_RUN`，通过 C5 按 ID1-ID6 发送 write-only `0x29` batch，再调用 C6 feedback scheduler 的非阻塞 feedback slot。

### SERVO_RUN -> STOP

以下情况进入一次性停止：

- Servo batch 写失败；
- feedback timeout、response mismatch 或 malformed RX；
- target watchdog 超时；
- 显式 operator stop。

runner 保留 C6 提供的具体停止原因，例如 `feedback_mismatch`、`feedback_timeout` 或 `watchdog_timeout`。重复 stop 不重复调用 transport pause callback。

## 4. Mock 测试结果

执行 C5/C6/C7 隔离回归：

```text
pytest -q bone_ws/src/pus6_motor_driver/test/test_j7_c5_six_axis_servo_transport.py \
       bone_ws/src/pus6_motor_driver/test/test_j7_c6_feedback_scheduler.py \
       bone_ws/src/pus6_motor_driver/test/test_j7_c7_servo_experiment_runner.py
```

结果：

```text
.................                                                        [100%]
17 passed in 0.06s
```

C7 测试覆盖：

1. 任一轴 Servo 配置失败时禁止 arm，且不发送运动帧；
2. 全部门禁通过后启动 PARK/HOME 六轴 Servo stream；
3. feedback response mismatch 触发 STOP；
4. target watchdog 超时触发 STOP；
5. stop 幂等，transport stop callback 只执行一次。

结果属于软件 Mock 验证，不代表真实 UART、执行器或六轴机构验证。

## 5. 当前限制

- runner 没有 ROS2 订阅入口，也没有修改现有 ROS2 节点；
- runner 只接受注入式 C5/C6 对象，没有真实串口打开逻辑；
- stop 依赖外部 pause callback，未在本阶段实现真实 `0x1A` 写入；
- feedback scheduler 的 backend timeout 仍由注入 bus 提供，未验证真实 UART response latency；
- 未执行 PARK/HOME 硬件动作、轨迹生成、姿态运动或振动测量；
- Mock 的 `hardware_write_count=0` 仅说明测试没有真实硬件写入。

本阶段完成后停止，不进入硬件测试，也不改变 J6 frozen baseline。
