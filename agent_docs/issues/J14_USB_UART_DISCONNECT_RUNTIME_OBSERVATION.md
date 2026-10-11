# J14 USB-UART Disconnect Runtime Observation

记录日期：2026-10-11。范围：已有实机记录的问题归档；本次文档整理 `hardware_write_count = 0`。

## 1. Symptom

Classical P/PD Demo 在 ACTIVE 中因 driver fatal latch 停止：

```text
Driver fatal latch detected via /rosout:
J7 driver STOP latched: RxSafetyError: [Errno 5] Input/output error
```

本次为电机 UART 链路的接收异常，经 Parent 和 `/rosout` 传递给 controller；不是 Base IMU age timeout。

## 2. Evidence

运行目录：[feedforward_pd_20261011_191309_748317](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317)。

### USB / serial

本机 kernel journal 的 2026-10-11 19:13:28–19:13:48（UTC+8）窗口保存了以下 CH341 / ttyUSB0 事件：

| journal 接收墙钟，UTC+8 | Kernel source monotonic（µs） | Message |
| --- | ---: | --- |
| 19:13:36.204510 | 15872766805 | `usb 3-2: USB disconnect, device number 3` |
| 19:13:36.205147 | 15872767069 | `usb 3-2: failed to send control message: -19` |
| 19:13:36.205297 | 15872767259 | `ch341-uart ttyUSB0: ch341-uart converter now disconnected from ttyUSB0` |
| 19:13:36.605172 | 15873167501 | `usb 3-2: ch341-uart converter now attached to ttyUSB0` |
| 19:13:36.842535 | 15873405168 | `usb 3-2: USB disconnect, device number 12` |
| 19:13:37.392771 | 15873955359 | `usb 3-2: ch341-uart converter now attached to ttyUSB0` |

只读查询入口（按本机 UTC+8 解释时间）：

```bash
journalctl -k --since '2026-10-11 19:13:28' --until '2026-10-11 19:13:48' --no-pager -o json
```

表中 source monotonic 来自 `_SOURCE_MONOTONIC_TIMESTAMP`，墙钟来自 `__REALTIME_TIMESTAMP`，是 journal 接收墙钟。源时间与 `__MONOTONIC_TIMESTAMP` 接收时间有约 0.515 s 差额；未独立校准，不将其直接解释为传输延迟，不与 Child 时间戳直接相减宣称精确因果延迟。内核证据来自系统 journal，并非运行目录中已有的 kernel 导出文件；本次未新增或修改 results。

[production_configuration.json](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/production_configuration.json) 保存的 Child 端口是 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，robot.log 记录电机 UART 以 921600 baud 打开。此次 CH341 / ttyUSB0 是电机 UART 链路，不是两个 IMU 的独立串口。

### Driver and runtime propagation

| 层 / 事件 | 已保存证据 |
| --- | --- |
| UART Child | [child_fault.jsonl](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/child_fault.jsonl)：cycle 1252，`communication_fault = RxSafetyError: [Errno 5] Input/output error`，`feedback_ready_ns = 15873287810106`，`failed_audit = null`。 |
| Parent IPC | [parent_ipc.jsonl](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/parent_ipc.jsonl)：`parent_receive` / `type = fault` / cycle 1252，`ns = 15873288155223`，与 Child packet 对应。 |
| Parent fatal latch | [robot.log](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/robot.log)：FATAL 原始 ROS 时间 `1791717216.212772577`，即 2026-10-11 19:13:36.212772577，UTC+8。 |
| Controller stop | [controller_states.jsonl](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/controller_states.jsonl)：FAULT 状态记录 `monotonic_ns = 15873291230919`；[controller_final.json](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/controller_final.json) 保存相同 driver fatal reason。 |
| Relay | [relay_summary.json](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/relay_summary.json)：`exit_reason = clean_shutdown`；不是原始 UART 异常发生层。 |

最早可见的故障处理动作是 [Pause audit](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/stop_pause_audit.jsonl) 中 `rx_safety_failure` 的首次 Pause 尝试，`tx_start_ns = 15873286603972`，早于 fault packet 与 Parent fatal log。精确底层 EIO syscall / 异常抛出时间 **NOT RECORDED**；不把 fault packet 时间当作首次 I/O 失败时刻。

最后 controller telemetry 的 Base / Platform / feedback age 为 3.984694 / 0.852651 / 15.608829 ms，无电机 fault、无残差饱和。Child health 记录 `tx_timeouts = 0`、`fault_cycles = 1`。本次错误不能解释为普通 TX timeout 或 controller IMU freshness failure。

## 3. Diagnosis

**Confirmed evidence**

- USB-UART disconnect occurred：内核明确记录 CH341 / ttyUSB0 断连、重新枚举及再次断连。
- UART Child 保存 RxSafetyError，并通过 IPC 传给 Parent；Parent fatal latch 经 `/rosout` 触发 controller STOP。这一软件传播链有逐层记录。
- 接收路径包装位置见 [j7_six_axis_servo_transport.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py)：`drain_rx()` 底层异常包装为 RxSafetyError；运动 TX 前接收准备失败会调用 `stop("rx_safety_failure")`。
- Parent IPC 没有本次 `parent_IPC_exception` / `parent_lease_expired` 记录；Relay 未记录同类原始异常。

**Strong indication**

- physical link interruption caused EIO：同一电机 USB-UART 链路断连、Child EIO 和六轴 Pause 时的 EIO 相互吻合。断连事件和软件传播链已确认，但不把底层 syscall 的因果细节升级为完整物理根因证明。
- 本次不是已确认的旧诊断长尾、调度阻塞或 Parent lease starvation 机制复发。

**Unknown**

- USB 断连的物理触发因素：cable、connector、power、USB controller、EMI，均未确定。
- 底层失败发生于 `in_waiting` 查询还是 `serial.read()`；没有完整 traceback，`rx_read_events.csv` 没有数据行。
- 断连后 Pause 是否到达并被各执行器执行、执行器最终实际状态，均无法由缺失 ACK 确认。

[bridge_lifecycle.jsonl](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/bridge_lifecycle.jsonl) 还保存全会话 `drop_count = 8`、最大 feedback gap 334.179668 ms、`health_ok = false`。保留的故障前 Bridge 尾段没有对应长 gap，不能证明这些历史异常造成此次 EIO；该异常记录不删去。

## 4. Shutdown Behavior

[demo_final.json](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/demo_final.json)、[processes.json](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/processes.json)、[stop_pause_audit.jsonl](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/stop_pause_audit.jsonl) 和 [parent_lifecycle.jsonl](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_191309_748317/parent_lifecycle.jsonl) 保存：

| 项目 | 状态与证据 |
| --- | --- |
| Process cleanup | **PASS（最终资源清理）**：Child / Bridge joined，返回码均为 0；四个 runtime host exited，返回码均为 0；无残留进程。 |
| Serial release | **PASS**：`serial_release.free = true`。 |
| Six-axis Pause ACK | **FAILED**：六个 Pause 尝试均记录 EIO、无响应；`pause_ack_ids = []`，`pause_success = false`。 |
| Complete Demo shutdown | **FAIL**：`safe_shutdown = false`，`demo_shutdown_pass = false`。 |

失败原因：**USB link unavailable during pause** 是强支持的解释；已确认的直接证据是每轴 Pause 尝试均返回 EIO，没有六轴 ACK。不能以程序退出或串口释放成功替代 Pause 执行确认。

按 [Launcher 判定](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_launcher.py) 逐项复核，`safe_shutdown` 的失败条件是 `pause_success` 和六轴 ACK 两项，其余所检查条件满足。controller 的返回码 1 是本次预期故障退出，在该判定中允许。HOME recovery 按故障策略 SKIPPED，不是一次执行失败的 HOME 动作。

robot.log 另记录 control、kinematics、FK 节点从 SIGINT 升级到 SIGTERM 后退出。最终 process cleanup PASS 不表示所有进程都完成无升级的 graceful shutdown；该退出异常保留披露。

## 5. Classification

**Known limitation / future reliability enhancement**

**Automatic recovery/resume not qualified。** USB 重新枚举不等于当前控制会话自动恢复、重新锚定或运动已安全恢复；本记录不授予自动 reconnect / resume 资格。

本 issue 归档外部电机 USB-UART 链路失效及停止确认边界，USB 断连物理原因 **UNKNOWN**。不据此认定 P/PD 控制算法故障，也不将其与 [controller-side IMU freshness issue](/home/lyapunov/bone_drill/agent_docs/issues/J14_IMU_FRESHNESS_TIMEOUT_OBSERVATION.md) 合并为同一 root cause。

本文件不修改 [J14_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_FINAL_SUMMARY.md) 或 [J14_FINAL_HANDOFF.md](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_FINAL_HANDOFF.md)；既有资格结论仍限定于原来保存的配置和验证会话，本观察不扩展或改写该范围。
