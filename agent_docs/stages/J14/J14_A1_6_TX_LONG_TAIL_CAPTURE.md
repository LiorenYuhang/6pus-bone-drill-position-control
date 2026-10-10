# J14-A1.6 TX Long-Tail Root-Cause Capture

2026-10-08，Ubuntu / ROS 2 Jazzy；工作目录 `/home/lyapunov/bone_drill`，当前 `bone_ws/install` 指向当前源码。

A1.6 **软件 PASS；自然失败捕获未完成**。只执行 R1 一次，30 s 观察窗口内没有 TX timeout，到时停止。总 wall **30.522771 s** 含启动、SIGINT 后 pause 和清理。完整记录 **1438 cycles（0–1437）**。本次 trace 不是 qualification，不能升级 A1.5 的结论。

唯一主分类：**F — still unresolved；失败机制 confidence = UNKNOWN**。

唯一下一选择：**E — 继续 observation**。后续独立阶段应先减轻已经测到的 tracing 扰动，再采集自然失败的同次 write；本阶段未实施下一修改，也未追加真机运行。

A1-TRANSPORT：**PARTIAL / NOT YET QUALIFIED**。A1-RUNTIME：**NOT PASS**。

## 实际执行与停止层

原 production bringup 下的静态 Servo 路径，未启动 LADRC ACTIVE runner，没有外部目标或扰动指令。50 Hz dedicated ServoThread、async telemetry、age-based feedback 原样使用。仅给 motor driver 加 strace prefix；其他节点不被 tracing。

R1 stop_reason 为 `no_natural_timeout_within_30s`，不是捕获 timeout 后停止。`tx_write_events.jsonl` 为空，robot log 没有 `UART TX failure` / STOP / recovery 事件。无 recovery、re-anchor 或 resume 在本次执行中发生。退出时已有 shutdown 路径发送 0x1A，**6/6 pause ACK** 持久化，反馈 current=0、fault=0。进程无残留，ttyUSB0 / ttyACM0 / ttyACM1 已释放。

日志仍出现既有 `rcl_shutdown already called` 和关闭 context 后的 telemetry publish error；launch 返回 0 **不代表干净退出**。不满足用户“首次没有异常才允许第二次”的条件，因此没有 R2。不存在强制 timeout、改 deadline 或补跑长时。

[R1 元数据](../results/j14/j14_a1_6_r1_tx_trace/run_metadata.json) · [robot log](../results/j14/j14_a1_6_r1_tx_trace/robot.log) · [pause 审计](../results/j14/j14_a1_6_r1_tx_trace/stop_pause_audit.jsonl) · [最终资源检查](../results/j14/j14_a1_6_r1_tx_trace/final_resource_check.json)

## 要求的 failed-write 字段与实际可用证据

**本次没有 failed write。** 下表成功样本只用于核对采集链，不能被改称失败事件或用于替代历史 timeout 的同区间计数器。

| 字段 | 本次 failed write | 最长保留成功 write |
| --- | --- | --- |
| identity / Motor / cycle | 不存在 | write_seq **8578**，ID **4**，cycle **1427** |
| driver PID / native TID | 不存在 | **462811 / 462895** |
| configured TX timeout | 30 ms（配置） | **30 ms** |
| serial.write wall | 未测得失败区间 | **8.677353 ms** |
| transport wrapper wall | 未测得失败区间 | **11.456147 ms** |
| thread CPU | 未测得失败区间 | **0.509446 ms** |
| process CPU | 未测得失败区间 | **8.776975 ms** |
| RUSAGE_THREAD voluntary / involuntary delta | 未测得失败区间 | **61 / 0** |
| out_waiting before → after | 未测得失败区间 | **0 → 0** |
| in_waiting before → after | 未测得失败区间 | **60 → 80** |
| requested / returned bytes | 未测得失败区间 | **10 / 10**；os.write 也返回 10 |
| os.write syscall elapsed | 未测得失败区间 | **0.022000 ms** |
| select/poll elapsed | 未测得失败区间 | **pselect6 0.027000 ms**，返回 writable |
| tty syscall total（同次 serial.write） | 未测得失败区间 | **0.049000 ms**，一次 write + 一次 pselect6 |
| serial wall − tty syscall elapsed | 未测得失败区间 | **8.628353 ms** |
| wrapper wall − tty syscall elapsed | 未测得失败区间 | **11.407147 ms** |
| counter snapshot 扩展区间开销 | 未测得失败区间 | **0.216670 ms** |

wrapper extra 包括 serial 内部 syscall 之间的时间，以及 wrapper 的间隔等待、queue ioctl、计数器、Python 和 tracing/descheduling 时间，**不是纯 Python CPU 或纯 GIL wait**。process CPU 是全进程多线程 CPU，允许大于 serial wall；它的采样边界略宽于 wall，不能拿高 process CPU 单独确认 GIL。

详细应用 telemetry 是固定 300-cycle ring，保留 **cycles1138–1437 / 1800 writes**。完整 cycle summary 覆盖所有 1438 cycles；driver tty 原始 trace 覆盖启动到退出。未把 retained-window 的最大成功值冒充全程单笔最大值。所有 1800 个保留配对均为一次 os.write 返回10和一次 pselect6；没有 write error。

[应用 CSV](../results/j14/j14_a1_6_r1_tx_trace/tx_queue_state.csv) · [空失败事件文件](../results/j14/j14_a1_6_r1_tx_trace/tx_write_events.jsonl) · [逐笔对齐](../results/j14/j14_a1_6_r1_tx_trace/write_syscall_alignment.jsonl) · [汇总与配对验证](../results/j14/j14_a1_6_r1_tx_trace/alignment_validation.json)

## PySerial 3.5 路径与对齐

本机源码 `/usr/lib/python3/dist-packages/serial/serialposix.py`：有限 timeout 路径先 `os.write`，然后检查 deadline，再以剩余预算 select；处理已写 bytes 后还有一次 deadline 检查。即使 os.write 接受全部10 bytes，也可能在后续 Python 恢复或 deadline 检查时抛 timeout；这不确认电机已执行，失败 delivery 仍 uncertain，不能 blind retry。

采用 `strace -D -ff -ttt -T -yy -I 3 -e trace=write,select,pselect6,poll,ppoll,ioctl,read -P /dev/ttyUSB0`。18个 PID/TID trace 文件只针对 driver。没有 seccomp-bpf，因本机 strace 此选项隐式启用 kill-on-exit。路径过滤减少输出，但 ptrace 仍可能扰动未打印 syscall 的执行。

每笔 write 前记录 monotonic / epoch / monotonic 时钟配对。本机 perf_counter 和 monotonic 都来自 CLOCK_MONOTONIC。按 native TID、write_seq、cycle/ID 和同次 enter/exit 对齐，不使用 entry-to-entry CPU。映射误差以时钟采样 bracket 加 strace 1 µs 打印分辨率保留；成功样本误差预算约 **1.201 µs**，保留窗口最大 clock bracket **20.871 µs**。

成功样本的 monotonic 区间 **23250012316686–23250020994039 ns**；对应 epoch **1791444485290298817–1791444485298976170 ns**：

| 路径 | elapsed |
| --- | --- |
| app enter → write entry | 0.039183 ms |
| os.write | 0.022000 ms |
| write exit → pselect6 entry | **7.078000 ms** |
| pselect6（remaining budget 22.968 ms） | 0.027000 ms |
| pselect6 exit → app exit | **1.511170 ms** |

**CONFIRMED**：这个成功样本的大部分 serial elapsed 不在两次 tty syscall 的 elapsed 内。**INFERRED**：它与 traced process-local contention/调度等待相容。**UNKNOWN**：具体 GIL reacquisition、其他 Python 线程、ptrace stop、未打印 syscall 和 OS 调度各占多少；更不能据此确认历史40 ms timeout机制。syscall `<T>` 是 elapsed，含 tracing/调度影响，不是独占 kernel CPU 或纯设备等待。

[该 TID 原始 trace](../results/j14/j14_a1_6_r1_tx_trace/driver_trace.462895) · [本机 PySerial write 源码快照](../results/j14/j14_a1_6_software/installed_pyserial_write.txt) · [对齐汇总](../results/j14/j14_a1_6_r1_tx_trace/trace_alignment.json)

## Queue、heartbeat 与 scheduler 独立时间线

成功长尾样本 out_waiting **0→0**，write/select 仅49 µs，不支持该样本存在持续 kernel TX queue buildup。in_waiting 是 RX bytes，不能解释为 TX queue。没有失败样本，因此不能回答“本次 timeout 时 queue=0 是否仍有 write/select 阻塞”。不能写“TX queue 堆满”，也不能从瞬时0推断 USB 已排除。

HostHeartbeat 在独立 harness process 中10 ms采样，仅记录 delay >20 ms；本次 **0条 stall**。因此无证据提升 host-wide stall 因果关系，但不排除短 stall、单核/单线程调度或未观测区间。

| run / 事件 | monotonic 时间 ns | 已确认结果 |
| --- | --- | --- |
| A1.5 cycle13 scheduler | entry **20291394938320** | inter-gap458.853 ms、wake445.786 ms |
| A1.5 cycle607 / ID5 timeout | **20303419492493–20303459999534** | serial40.507041 ms |
| A1.5 cycle614 / ID1 timeout | **20303749859132–20303791278059** | serial41.418927 ms |
| A1.6 cycle217 最大 wake lateness | entry **23225824574022** | wake19.037 ms |
| A1.6 cycle388 stall | **23229225647500–23229264192839** | callback38.545 ms、wake0.110 ms |
| A1.6 cycle1427 / ID4 成功长尾 | **23250012316686–23250020994039** | 非失败；不与上述 stall 重合 |

A1.5 cycle13 与其607/614 timeout 分离约12秒，不能确认前者导致后者；A1.6 没有 timeout 可作时间重合检验。不同 run 保留独立标识，不把两次运行的指标拼成同次故障。A1.6 的四个 recorder stall 为cycles215/339/373/388，均保留原始事件。

[独立时间线 JSON](../results/j14/j14_a1_6_r1_tx_trace/separate_monotonic_timelines.json) · [全程 cycles](../results/j14/j14_a1_6_r1_tx_trace/cycle_summary_full.csv) · [heartbeat](../results/j14/j14_a1_6_r1_tx_trace/host_heartbeat_stalls.csv) · [stall events](../results/j14/j14_a1_6_r1_tx_trace/stall_events_all.csv)

## Kernel / USB 证据

`journalctl -k -b` 仅实验窗口前后2 s，读取成功，返回 `-- No entries --`。**no explicit kernel error observed**：窗口中未观察 disconnect/reset/URB/tty/CH340/autosuspend 错误。`dmesg --since/--until` 读取返回1，`Operation not permitted`；未 sudo、未改变日志权限。USB/CH340/tty路径仍未排除。

[journal 窗口](../results/j14/j14_a1_6_r1_tx_trace/kernel_journal_window.log) · [dmesg 结果](../results/j14/j14_a1_6_r1_tx_trace/dmesg_window.log)

## 诊断改动与软件验证

仅两处 tracked source 的 A1.6 增量：

- `j7_real_uart_adapter.py`：SerialWriteMetric 附加 optional 字段；adapter init / write 增加同次 process CPU、RUSAGE_THREAD CS、native TID、write_seq、clock sync 与边界开销。wall enter/exit 紧邻 serial.write；顺序 CPU/usage snapshots 略宽，并显式记录。diagnostics 关闭时新计数器不采集。serial.write 仍为一次调用，timeout/short-write 映射不变，无 retry。
- `stall_diagnostic_recorder.py`：已有 audit 处理转录这些字段，现有 dump 增加失败事件 JSONL。串口调用区间内没有新增文件 I/O。

新增阶段测试、独立 trace launch、capture harness 和离线 analyzer。launch 复用原 production generate_launch_description，只给 driver constructor 加 prefix；未改原 launch 或 driver shutdown。软件 Mock 初次发现 tracer 接收 SIGINT 导致 driver 未退出，用 `-D` 修正并复验；仅清理自身 Mock 进程，**硬件写入0**，相关证据保留。

| 验证 | 结果与边界 |
| --- | --- |
| targeted pytest | **22 PASS**，含新6case及已有 write semantics |
| 最终干净 ROS_DOMAIN_ID181 全套 pytest | **644 PASS / 44.22 s**；normal、8ms delayed、timeout、RUSAGE_THREAD、CPU及CSV/event身份配对；Mock/Fake硬件写入0 |
| py_compile / diff check | **PASS**，包括离线 analyzer |
| colcon build | motor driver / control / h30_imu 三包 **PASS / 1.51 s** |
| final Mock traced launch SIGINT | 退出、无残留、串口空闲；硬件写入0；已有shutdown异常仍记录 |
| final -D/no-BPF PTY flags | write/pselect6捕获 **PASS**，硬件写入0 |
| A1.6 scope核查 | 其余8个既有tracked修改diff与进入阶段前一致；adapter所有RX/request/drain方法AST不变 |

未 trace 的开销对照：FakeSerial 7×3000 alternating blocks，wrapper增量 **3.735 µs**；真实 PySerial PTY 5×1000，增量 **2.406 µs**（17.384→19.789 µs，约13.8%）；串口call median4.878→5.010 µs，snapshot overhead median2.004 µs。不是零开销；绝对增量小于历史几十µs写簇，未引入额外队列诊断或进一步基础设施。

**tracing 的扰动显著更大**：R1保留write serial median **181.100 µs**，A1.5保留成功write median **34.005 µs**；R1 wrapper median **754.609 µs**、snapshot overhead median **159.141 µs**。两次运行非受控A/B，不能把全部差值归因于strace，但trace条件确实改变了测得的时序分布。CS尤其包含ptrace相关等待，不能直接当作未trace GIL/OS故障证据。此次无 timeout 不能说明问题已修复。

TX **30 ms**、正常RX **5 ms**、drain **4 ms**、**80 steps / 283.3 ms / 1000 ms / 1500 mA** 不变。pause/fresh/reanchor、scheduler、Executor/QoS、priority、affinity、控制器、shaper算法均未更改。

[软件结果目录](../results/j14/j14_a1_6_software/) · [scope验证](../results/j14/j14_a1_6_software/scope_validation.json) · [PTY开销](../results/j14/j14_a1_6_software/instrumentation_overhead_pty.json) · [A1.6独立源码增量](../results/j14/j14_a1_6_software/a1_6_only_source_delta.patch)

## Git 与最终边界

当前 `main`，HEAD `feba8fd00c392ecaab93b53138db7c1d06bce000` 未改变。进入阶段前的10个 tracked modified files 和历史未跟踪工作全部保留；A1.6只在上述两份tracked source增加诊断，并新增阶段脚本、测试和证据。没有 staging、commit、push、tag、reset、production fix，未改写历史原始实验数据。

[Git status](../results/j14/j14_a1_6_software/git_status_final.txt) · [Git identity](../results/j14/j14_a1_6_software/git_identity_final.json)

本阶段已停止。自然30 ms timeout的 syscall/process CPU/CS 同区间根因证据仍缺失，失败捕获目标未达成；完整恢复与连续静态通信的旧资格结论保持不变。
