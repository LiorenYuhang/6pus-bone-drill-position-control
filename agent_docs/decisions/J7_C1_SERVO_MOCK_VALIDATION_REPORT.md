# J7-C1 Servo Mode Mock Validation Report

审查阶段：J7-C1 Servo Mode Mock Validation  
日期：2026-08-25  
验证边界：仅软件 Mock/FakeBus；未连接真实硬件、未打开真实串口、未发送真实运动命令。

## 1. 创建文件

新增 J7 专用测试入口：

`bone_ws/src/pus6_motor_driver/test/test_j7_c1_servo_mock.py`

新增本报告：

`agent_docs/decisions/J7_C1_SERVO_MOCK_VALIDATION_REPORT.md`

未修改 `protocol.py`、`uart_bus.py`、`motor.py`、现有 ROS2 节点、J6 runner、配置文件或既有测试。

## 2. Mock 架构

测试入口由四个隔离组件组成：

- `VirtualClock`：只推进虚拟单调时钟，不使用 wall-clock sleep。
- `J7ServoMockBus`：使用生产协议的帧打包/解析函数检查寄存器和命令字段，但不持有串口对象。
- `LatestTargetBuffer`：单槽 Latest-Value-Wins 缓冲，记录覆盖次数和目标更新时间。
- `J7ServoScheduler`：固定 50 Hz 的六轴调度器，记录周期、逐轴 TX、deadline、目标年龄、watchdog 和停止原因。

配置路径为：

`mode=1, speed=50` → 每轴从 `0x25` 写入 `[1, 0, 0, 50]`（覆盖 `0x25..0x28`）→ 模拟响应 → 分别读回 `0x25`、`0x28`。

运动路径为：

`最新六轴目标` → `50 Hz scheduler` → ID1..ID6 各一帧 `0x29` → MockBus 记录 TX。

运动 `0x29` 路径明确不读取响应、不重试；异常只触发一次六轴 pause 记录（`0x1A`）。

## 3. 测试结果

运行命令：

```text
pytest -q bone_ws/src/pus6_motor_driver/test/test_j7_c1_servo_mock.py
```

结果：

```text
6 passed
```

覆盖场景：

| 场景 | 结果 | 关键证据 |
|---|---|---|
| mode=1 配置与读回 | 通过 | 6 个 `0x25` 配置帧，12 次配置读回，所有轴 mode=1、speed=50 |
| 正常 50 Hz stream | 通过 | 5 个周期、30 个 `0x29` 帧，单周期六轴发送 |
| Latest-Value-Wins | 通过 | 两次更新只发送最新目标，`overwrite_count=1` |
| 写入 deadline | 通过 | 注入 4 ms/轴，六轴批次 24 ms，正确标记 deadline miss |
| 目标停止与 watchdog | 通过 | 50 ms 超时后停止写流，记录一次 watchdog，并发送 6 个 pause |
| 写入失败 | 通过 | ID3 仅尝试一次；不发送 ID4..ID6 运动帧，随后六轴 pause |
| graceful stop | 通过 | 显式停止幂等，pause 只执行一次 |

所有测试断言 `hardware_write_count = 0`。这里的计数表示真实硬件写入；MockBus 中的协议帧仅为内存审计记录。

## 4. TX 周期统计

虚拟时钟下固定周期为 20,000,000 ns（50 Hz）：

- 正常验证周期数：5
- 周期间隔：均为 20,000,000 ns
- `cycle_jitter`：均为 0 ns
- 每周期运动帧数：6（每轴一帧 `0x29`）
- 正常总运动帧数：30
- 默认 Mock TX 时间：100,000 ns/轴；批次约 600,000 ns
- 注入慢 TX 场景：4,000,000 ns/轴；批次 24,000,000 ns，标记 deadline miss
- 运动帧 response wait：0
- 运动帧 retry：0

时间结果是虚拟调度测试结果，不代表真实 UART 或物理电机性能。

## 5. Watchdog 验证

测试先发送一组六轴目标，随后停止目标更新。watchdog 阈值为 50 ms；在目标年龄超过阈值的周期：

1. 设置 `watchdog_timeout=true` 并记录目标年龄；
2. 停止后续 `0x29` 写流；
3. 触发一次 `pause_all`，每轴记录一个 `0x1A` pause；
4. scheduler 进入 `stopped=true, stop_reason=watchdog_timeout`。

写失败路径同样进入 `stopped=true`，停止原因是 `motion_write_failure`，并保证失败运动帧没有自动重试。

## 6. 未解决问题

- 本阶段只验证软件调度、帧字段、记录字段和停止语义；没有验证真实设备对 mode=1、连续 `0x29` 或 pause 时序的响应。
- MockBus 没有模拟真实 UART 字节传输、半双工仲裁、操作系统调度抖动、串口缓冲区或电机内部位置环。
- 配置读回属于 Mock response/readback；运动流没有反馈闭环。Servo 实验仍需后续单电机阶段定义低频反馈采样和失联处理。
- 本测试入口未接入现有 ROS2 节点，也未改变 J6 frozen baseline（mode=0、HOME/PARK）。

结论：J7 Servo 架构在 Mock 层满足 mode=1 配置、50 Hz 六轴 write-only 运动流、Latest-Value-Wins、无重试、deadline 记录和 watchdog/graceful-stop 安全语义；可以进入 J7-C2 前置设计审查，但本报告不授权任何硬件测试。
