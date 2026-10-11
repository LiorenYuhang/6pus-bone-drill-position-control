# J14 IMU Freshness Timeout Observation

记录日期：2026-10-11。范围：已有实机记录的问题归档；本次文档整理 `hardware_write_count = 0`。

## 1. Symptom

Classical P/PD Demo 完成 startup HOME、reference capture 并进入 ACTIVE 后，controller 触发：

```text
Base IMU age timeout (>100ms)
```

controller 进入 FAULT，Launcher 随后报告 DEMO STOP。该次运行配置为 Feedforward + Classical P/PD（Ki = 0）；这是输入 freshness 观察，不是控制性能评价。

## 2. Evidence

运行目录：[feedforward_pd_20261011_185636_133413](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_185636_133413)。

主要记录：[controller_final.json](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_185636_133413/controller_final.json)、[controller_states.jsonl](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_185636_133413/controller_states.jsonl)、[demo_final.json](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_185636_133413/demo_final.json)、[processes.json](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_185636_133413/processes.json)、[robot.log](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_185636_133413/robot.log) 与 [Base IMU log](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_185636_133413/base_imu.log) / [Platform IMU log](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_185636_133413/platform_imu.log)。

| 项目 | 已保存记录 |
| --- | --- |
| controller ACTIVE 状态记录 | `monotonic_ns = 14858345684199` |
| controller FAULT 状态记录 | `monotonic_ns = 14858404924339`；距 ACTIVE 状态记录 59.240140 ms |
| 最后一行 telemetry | cycle 56，`timestamp_ns = 14858384438653`，state = ACTIVE |
| 最后 Base IMU age | 84.697320 ms |
| 最后 Platform IMU age | 85.577877 ms |
| 最后 driver feedback age | 102.493139 ms |
| controller 最终状态 / 原因 | FAULT / `Base IMU age timeout (>100ms)`；最终 cycle index = 57 |

FAULT 状态记录的墙钟约为 **2026-10-11 18:56:41.328，UTC+8**，按 Demo 起始 epoch/monotonic 时钟对推算，不是直接保存的 watchdog 检测墙钟。

cycle 57 的 watchdog 分支在 telemetry append 前返回，故障 tick 的精确 `timestamp_ns` **NOT RECORDED**。最后保存的 84.697320 ms 是 cycle 56 的 age，不能当作触发时的 age。

按已有字段反算 controller 最后一次 Base IMU 回调时间：

```text
14858384438653 - round(84.697320 * 1000000)
= 14858299741333 ns
```

该值是推算的主机回调接收时刻，约为墙钟 18:56:41.223。若无更新，到 FAULT 状态记录时约为 105.183006 ms；这也不是直接保存的故障 tick age。sensor sample time、消息 header 时间及完整 IMU receive sequence **NOT RECORDED**。现有源码将 age 定义为 `time.perf_counter_ns()` 与该 controller 最近一次输入回调时间的差，见 [J11M2PidResidualRunner](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_m2_pid_residual_runner.py)。

Launcher 的独立 observer 最后记录到 Base / Platform 回调分别为 `14858.405638852 s` / `14858.404963768 s`，即 FAULT 状态记录后约 0.715 ms / 0.039 ms；timed feedback 回调记录于 FAULT 前约 3.102 ms。这些是 observer 的接收时间，不能与 controller 回调或传感器采样时间混同，也不能排除消息积压。

该次命令未启用 `--record-telemetry`，目录没有 `telemetry.csv` / `metadata.json`。可访问的 kernel journal 在 2026-10-11 18:56:35–18:56:43（UTC+8）没有条目，未找到该窗口的 USB 断连证据；这不证明源端绝无短暂中断。

## 3. Diagnosis

**Confirmed evidence**

- controller 的 Base 输入 freshness watchdog triggered，进入 FAULT；trigger 是接收 age 超过 100 ms，不能把阈值超限本身称为 root cause。
- 最后一行中 Base、Platform 与 feedback age 同时变旧；独立 Launcher observer 在故障附近仍处理这些输入。
- 未记录 IMU 节点提前退出。Launcher 后续对两个 IMU 进程发送 SIGINT，两者返回码均为 0；没有证据确认节点退出导致本次 timeout。
- Child 最终原因是后续的 `parent_shutdown`，没有本次 `parent_lease_expired` 或 UART communication fault 记录。

**Strong indication**

- local callback/input freshness degradation：输入在 controller 进程内未及时刷新，比“Base IMU 已停止发布”更符合多个输入同时变旧、独立 observer 仍接收消息的证据。
- 最后一行到 FAULT 状态记录仅 20.485686 ms；age 超限不能解释为某一次 controller timer 等待了 100 ms。

**Unknown**

- DDS delivery delay、executor scheduling / callback starvation、局部阻塞的具体贡献。
- sensor source interruption、短暂停流或积压消息是否参与；没有完整发布/接收序列。
- 底层 root cause **UNKNOWN**，不确认 USB 断连，不确认 P/PD 控制律或参数异常，也不确认旧 J14 Parent starvation 机制复发。

## 4. Impact

controller 执行 freshness safety stop，停止主动姿态输出；故障退出按既有策略跳过 shutdown HOME。**现有记录未显示硬件失控**：最后记录无电机 fault，已保存六轴 Pause ACK。该表述仅限保存证据，未使用外部测量证明全部实际运动状态。

| 退出项目 | 记录 |
| --- | --- |
| Child 停止 / 关闭 | `parent_shutdown`，`closed = true`，`pause_success = true` |
| 六轴 Pause | ACK IDs = 1, 2, 3, 4, 5, 6 |
| 串口释放 | `serial_release.free = true` |
| 残留进程 | 空列表 |
| 完整 Demo shutdown | `safe_shutdown = false`，`demo_shutdown_pass = false` |

processes 记录 controller 返回码 −2；robot.log 还记录 control、kinematics、FK 节点退出时从 SIGINT 升级到 SIGTERM。资源最终释放成功不等于完整 shutdown PASS；退出异常发生在 freshness STOP 之后，不能反推为已确认的触发原因。

## 5. Classification

**Known limitation / future investigation**

这是 controller-side input freshness observation，root cause 尚未确定；不能归档为 IMU 节点退出、USB 故障、P/PD 算法失败或已验证的 executor 根因。

本 issue 补充已保存的运行观察，不改写 [J14_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_FINAL_SUMMARY.md) 或 [J14_FINAL_HANDOFF.md](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_FINAL_HANDOFF.md)，不撤回或扩大原资格验证的会话范围。与 [USB-UART disconnect issue](/home/lyapunov/bone_drill/agent_docs/issues/J14_USB_UART_DISCONNECT_RUNTIME_OBSERVATION.md) 分别归档，不合并为同一 root cause。
