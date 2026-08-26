# J7-C9 Real UART Adapter + Stop Path 实现记录

## 1. 实现范围

本阶段新增隔离的 J7 UART adapter、单一 RX owner 和六轴 stop adapter，用于 C5/C6/C7 的后续真实 backend 注入准备与 Mock 验证。

未修改：

- `protocol.py`；
- `uart_bus.py`；
- `motor.py`；
- 现有 ROS2 节点；
- J6 frozen baseline；
- 电机参数和硬件配置。

适配器要求调用方注入一个已经创建的 serial-like 对象；本模块不创建、不打开真实串口。未连接硬件、未发送运动命令。

## 2. 新增文件

- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_real_uart_adapter.py`
  - `J7UartAdapter`：`write()`、`request()`、`read_available()`、共享 `uart_lock`；
  - `J7RxOwner`：统一解析、缓存和交接完整 RX frame；
  - `J7StopAdapter`：六轴 `0x1A` 一次性 stop callback；
  - request、RX owner、stop audit 数据结构和 timeout/RX 异常类型。

- `bone_ws/src/pus6_motor_driver/test/test_j7_c9_real_uart_adapter.py`
  - 注入式 Mock serial；
  - Mock `hardware_write_count` 固定为 `0`。

## 3. UART adapter 行为

### 接口

`J7UartAdapter` 满足 C5/C6 需要的 bus surface：

```text
request(frame, expected_id, expected_command, retries)
write(frame)                 # 不读 response
read_available()             # 经过 RX owner 后交付完整 frame
uart_lock                    # 单 UART 仲裁
```

构造函数必须接收 `serial_instance`，因此不会在测试或导入时打开真实串口。真实调用方仍需在授权流程中创建串口对象并确认 `921600 baud`、超时和物理端口。

### request

- 发送后由 bounded timeout 等待 response；
- 通过 ID/command 校验；
- C5/C6 的调用使用 `retries=0`；
- request 之前会把前一 write-only 产生的残留分类为 stale，不能误当成当前 response；
- request TX/response/timeout 结果进入 request audit。

### write-only

`write()` 只调用 serial write、记录帧间隔并返回写入长度，不读取、不等待、不 retry。C5 的 `0x29` batch 因此保持 `response_wait=0`、`retry=0` 语义。

## 4. 单一 RX owner

`J7RxOwner` 是物理 RX 字节的唯一 parser/queue owner：

- `FrameStreamParser` 统一处理分片和完整 response frame；
- `read_available()` 只把已解析的完整 frame 一次性交给 C5/C6，避免 adapter 重复保留同一份 residual；
- request 会把未匹配的 ID/command/register frame 计为 stale；
- malformed frame、residual frame/byte overflow 进入 adapter RX error；
- audit 记录 residual、stale response、feedback response、malformed 和 overflow 数量。

因此当前 RX 交接链路为：

```text
serial.read
  -> J7RxOwner parser/queue
  -> C5 residual policy 或 C6 feedback matcher
  -> request 当前期望 response
```

真实 backend 仍必须保证同一 serial object 不被其他线程或旧 `UartBus` client 同时读取。

## 5. 六轴 stop adapter

`J7StopAdapter` 作为 C7 runner 的 pause callback：

- 按 ID1-ID6 顺序发送 `0x1A=[1]`；
- 每轴等待并记录 response、timeout、TX 时间和错误；
- 某一轴失败后继续尝试其余轴，但最终 `success=False`；
- 不 retry stop transaction；
- callback/stop_all 只执行一次，重复调用返回同一 `StopAudit`，不重复发送。

这使 C7 `transport.stop(reason)` 可以绑定到真实 stop callback，但本阶段没有把它接入真实 runner，也没有授权真实 `0x1A`。

## 6. Mock 验证结果

执行 C9 与既有 C5-C7 回归：

```text
pytest -q bone_ws/src/pus6_motor_driver/test/test_j7_c9_real_uart_adapter.py \
       bone_ws/src/pus6_motor_driver/test/test_j7_c5_six_axis_servo_transport.py \
       bone_ws/src/pus6_motor_driver/test/test_j7_c6_feedback_scheduler.py \
       bone_ws/src/pus6_motor_driver/test/test_j7_c7_servo_experiment_runner.py
```

结果：

```text
......................                                                   [100%]
22 passed in 0.07s
```

覆盖内容：

1. adapter 接口可被 C5 配置和 write-only batch 使用；
2. 单一 RX owner 将运动残留分类为 stale，不误匹配 feedback；
3. 六轴 stop 全部成功；
4. 单轴 stop write failure 会记录失败并继续其他轴；
5. 六轴 stop timeout 被记录且不 retry；
6. 所有 Mock `hardware_write_count=0`。

这些是软件 Mock 结果，不是真实 UART 或执行器结果。

## 7. 当前限制与下一道门禁

- adapter 没有串口创建/打开逻辑，真实端口、权限、线缆和波特率仍需现场授权；
- 尚未验证真实设备是否对 write-only `0x29` 返回 residual response；
- stop adapter 的 `0x1A` response/timeout 语义尚未在真实设备上验证；
- C5/C6/C7 仍需由受控调用方注入 `J7UartAdapter` 和 `J7StopAdapter`，当前没有 ROS2 集成；
- 真实实验前必须再次确认单一串口所有权、E-stop、限流电源、两人观察和 Phase 0-3 门禁。

C9 完成后仍不得打开真实串口或执行运动；下一步只能在新的授权阶段进行实机前 dry-run 审查。
