# J14-A1.2 RX Single-Read Long-Tail Attribution

2026-10-08，`main`，HEAD `feba8fd00c392ecaab93b53138db7c1d06bce000`。

本阶段完成诊断，停止在归因报告。第一次 Stage A 捕获了单次 read **57.579079 ms**，按要求提前结束；没有第二、第三次 run，没有进入 Runner ACTIVE，没有实施下一修复。

**A1-TRANSPORT：PARTIAL / NOT YET QUALIFIED。A1-RUNTIME：NOT PASS。** RX 为 **CONFIRMED SYMPTOM / CONTRIBUTOR**。同一次 read 的证据支持 R3 进程内其他线程/native 活动或争用，也支持 R1 等待；GIL、具体 syscall 和 USB/tty 根因仍为 UNKNOWN。唯一下一项建议是 **C：process-local scheduling / GIL diagnosis**。

## 1. 实际 PySerial RX Read Path

审计的是本机 PySerial 3.5 的 `/usr/lib/python3/dist-packages/serial/serialposix.py`，没有修改 site-packages。实际源码保存在 [installed_pyserial_read_source.txt](/home/lyapunov/bone_drill/results/j14/j14_a1_2_software/installed_pyserial_read_source.txt)。

```text
adapter._read_chunk()
  → serial.read(size)
    → Timeout(self._timeout): monotonic target = now + 0.005 s
    → select.select(fd, abort_pipe, timeout.time_left())
    → 若 ready，os.read(fd, remaining_size)
    → 累加 bytes，检查 timeout.expired()
    → return bytes(read)
  → adapter caller 恢复执行
  → parser / RX owner（在单次 read 计时区间之外）
```

当前实现使用 `select`，不是 `poll`；fd 使用 `O_NONBLOCK`。`time_left()` 返回 `max(0, target-now)`。未 ready 时结束 read；ready 时读剩余字节，然后检查 logical timeout。它没有一个能抢占当前 Python/内核执行的 5 ms wall-time 中断。等待返回、线程获调度、GIL 重新获取以及其间的 Python/native 执行，均可能使 caller 观察到的 wall 超过预算。

外层 4 ms drain deadline 也由线程执行到检查点时判断，无法中断尚未返回的 `serial.read()`。本阶段未改变这些语义。新增观测测量完整 `serial.read()` 调用及 caller 恢复执行的 wall，不能拆开 `select`、`os.read`、syscall 后调度和 GIL 获取的耗时。

## 2. 变更范围及区间语义

本阶段 production 增量只有三处，均为诊断观测：

| 文件 | 本阶段内容 |
|---|---|
| [j7_real_uart_adapter.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_real_uart_adapter.py) | 可选 read observer；每次实际 read 的 wall、配对 CPU/线程 CS、字节数、monotonic 起止及采样边界 |
| [motor_driver_node.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py) | runtime diagnostics 启用时接入 observer |
| [stall_diagnostic_recorder.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/stall_diagnostic_recorder.py) | Tier 1 七个标量、Tier 2 慢 read 完整事件及一次捕获通知 |

软件支撑文件：新增 [test_j14_a1_2_read_observation.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/test/test_j14_a1_2_read_observation.py)、[j14_a1_2_diagnostics.py](/home/lyapunov/bone_drill/bone_ws/scripts/j14_a1_2_diagnostics.py)、[j14_a1_2_analyze.py](/home/lyapunov/bone_drill/bone_ws/scripts/j14_a1_2_analyze.py)；已有未跟踪的 [test_j14_a0_instrumentation.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/test/test_j14_a0_instrumentation.py) 仅适配扩展后的 CSV header。本报告和 A1.2 results 为新资产。

`read_wall_ns` 的起止紧贴实际调用，原有 `perf_counter_ns()` serial call 计时继续保留。thread/process CPU 和 `RUSAGE_THREAD` 的 before/after **配对包围同一次 read**，没有借用相邻周期的 CPU 差值。使用当前 OS thread 的 `RUSAGE_THREAD`，不使用 process-wide CS。

必须说明采样限制：几个 clock/resource 查询只能顺序执行，不能原子地在同一个纳秒完成。CPU/CS 的采样窗口比纯 read wall 略宽，不能声称所有计数器端点完全重合。保存 `t_snapshot_enter/exit` 和 `snapshot_overhead_ns` 显示这个差异。本次完整采样外窗比 read wall 多 **11.177 µs（约 0.0194%）**；不能把这部分开销当成串口 read 长尾，也不影响本次低 thread CPU / 高 process CPU 的数量级判断。

`in_waiting_before` 复用原有 `_available()` 的结果，没有增加一次 queue ioctl。read size、原始返回 bytes、parser、drain loop/deadline、busy-wait、feedback scheduler 均未修改。observer 回调错误不能替换返回数据或原始 read 异常；默认关闭诊断时不启用新增采样。

Tier 1 保留原有 13 列前缀，追加七个标量：`rx_max_single_read_wall_ms`、对应同一最慢 read 的 thread/process CPU 和 voluntary/involuntary CS、`rx_read_call_count`、`rx_bytes`。CPU/CS 并非各自独立取最大值。Tier 2 仅保存 wall >5 ms 的完整事件。5/10/20/50/100 ms 均为 diagnostic bins，未成为 production safety gate。

首个周期内 >20 ms 事件返回后保存并打印一次捕获通知，由外部诊断脚本结束本次实验。通知不改变 driver 安全状态；打印发生在 read/counter 计时之后，可能影响 callback 后续耗时，但不在本次 57.579 ms read 区间内。

TX 保留 PySerial `serial.write()`、`write_timeout=0.030 s`，无 custom write、重试、reconnect 或 enlargement。80-step reject、283.3 ms warning、1000 ms hard stop、1500 mA，以及 J13 控制器、SO(3)、IK/FK、shaping、protocol、QoS、Executor 均无本阶段增量。

## 3. 软件验证

`py_compile` PASS；`colcon build --symlink-install --packages-select pus6_motor_driver` PASS；最终相关包 pytest **565 passed，0 failed/error/skipped，43.16 s**。这是软件验证，不是 runtime qualification。

新增测试覆盖 immediate、2 ms delayed、空返回 timeout、人工 25 ms delayed wrapper，配对 CPU/CS 差值、observer 异常、Tier 1/2 关联、原始 feedback frame 和 parser，以及真实本地 PTY 的 PySerial 5 ms timeout。额外验证 counter snapshot 人工延迟不会伪装成慢 read；后台 CPU 线程验证 process CPU 包含 reader 之外的线程执行。软件测试没有物理串口写，**hardware_write_count = 0**。

fake immediate read 观测开销：两次关闭为 0.560/0.569 µs/read，两次启用为 5.792/5.791 µs/read；平均增加 **5.227 µs/read**。若每周期两次 read，估算增加 0.010454 ms/cycle，仅是软件基准，不是硬件 timing 保证。慢事件打印/存储不包含在这个正常 read 均值的保证中。

证据：[pytest_final.log](/home/lyapunov/bone_drill/results/j14/j14_a1_2_software/pytest_final.log)、[pytest_final.xml](/home/lyapunov/bone_drill/results/j14/j14_a1_2_software/pytest_final.xml)、[colcon_build.log](/home/lyapunov/bone_drill/results/j14/j14_a1_2_software/colcon_build.log)、[overhead.json](/home/lyapunov/bone_drill/results/j14/j14_a1_2_software/overhead_final/overhead.json)。`benchmark_recorder` 是 synthetic software 数据，不纳入实机事件统计。

## 4. 真机 run 数量和全部慢事件

**一次 Stage A diagnostic**，上限 10 s，首次捕获代表性 >20 ms 后提前结束。启动至诊断进程清理约 1.727 s；该时间包含 startup/teardown，不等于 10 s 控制运行。没有 Runner ACTIVE、A1-R2/R3 或长时 qualification。

原有 launch 使用 J13 参数、active、maintenance、dedicated_thread、async_latest、age_based、runtime diagnostics；其余执行语义保持。记录 24 个周期，周期内实际 read 53 次，返回 2880 bytes。序号包含 startup/non-cycle read，因此 read_seq=63 不等于第 63 个周期内 read。

| Diagnostic threshold | 全部完整 read 事件数 | 事件 |
|---|---:|---|
| >5 ms | 1 | cycle 22 / read 63，57.579079 ms |
| >10 ms | 1 | 同一事件 |
| >20 ms | 1 | 同一事件 |
| >50 ms | 1 | 同一事件 |
| >100 ms | 0 | 本次未复现；不证明历史 105.997 ms tail 已消失 |

本次没有 TX timeout 记录，也未记录 driver hardware safety fault。普通 timing 异常未成为新的阻断条件。

原始证据：[run_metadata.json](/home/lyapunov/bone_drill/results/j14/j14_a1_2_rx_diag_r1/run_metadata.json)、[rx_read_events.csv](/home/lyapunov/bone_drill/results/j14/j14_a1_2_rx_diag_r1/rx_read_events.csv)、[cycle_timing.csv](/home/lyapunov/bone_drill/results/j14/j14_a1_2_rx_diag_r1/cycle_timing.csv)、[robot.log](/home/lyapunov/bone_drill/results/j14/j14_a1_2_rx_diag_r1/robot.log)。离线汇总：[analysis.json](/home/lyapunov/bone_drill/results/j14/j14_a1_2_analysis/analysis.json)、[all_slow_read_events.csv](/home/lyapunov/bone_drill/results/j14/j14_a1_2_analysis/all_slow_read_events.csv)。analysis 的零 write 字段仅指离线分析本身；Stage A 使用了真实驱动写操作。

## 5. 最严重单次 RX read 完整 telemetry

| 字段 | 实测值 |
|---|---:|
| cycle_seq / scope | 22 / cycle |
| read_seq / native_thread_id | 63 / 100027（ServoThread） |
| t_read_enter_ns | 5507713829333 |
| t_read_exit_ns | 5507771408412 |
| read_wall_ns | 57579079（57.579079 ms） |
| serial_call_wall_ns（原有计时） | 57578475（57.578475 ms） |
| t_snapshot_enter_ns | 5507713827194 |
| t_snapshot_exit_ns | 5507771417450 |
| snapshot_overhead_ns | 11177（0.011177 ms） |
| thread_cpu_before_ns / after_ns | 158632614 / 159309247 |
| read_thread_cpu_ns | 676633（0.676633 ms，wall 的 1.175%） |
| process_cpu_before_ns / after_ns | 1080536184 / 1141284336 |
| read_process_cpu_ns | 60748152（60.748152 ms，wall 的 105.504%） |
| voluntary CS before / after / delta | 1327 / 1521 / 194 |
| involuntary CS before / after / delta | 4 / 4 / 0 |
| in_waiting_before | 24 bytes |
| requested_read_bytes / returned_read_bytes | 24 / 24 |
| configured_read_timeout_s | 0.005 |
| read_error | 空，无 read exception |

对应 cycle 22：callback 64.018 ms，TX 3.286 ms，drain 60.399 ms，进入 callback 前 idle gap 10.612 ms，wake lateness 0.075 ms。单次 read 占 drain 的约 95.33%，是此 callback 延迟的直接贡献项。

## 6. 分类矩阵与 syscall 边界

| Class | 当前事件支持程度 | 依据及限制 |
|---|---|---|
| R1：blocking/sleep/syscall wait | 支持等待行为 | thread CPU 低、voluntary +194；不能单凭 CS 判定具体 syscall，也可能包含 GIL 等待 |
| R2：OS preemption | 本事件缺少直接支持 | involuntary +0；不排除整体调度机制参与 |
| R3：process-local activity/contention | 最有针对性的后续方向，INFERRED | thread CPU 很低、process CPU 很高、无同期记录的 heartbeat stall；尚未确定线程或 GIL |
| R4：CPU execution | 不支持 read 中持续 CPU 执行 | thread CPU/wall 约 1.175%；parser 在本测量区间外 |
| R5：host-level stall | 未见同期证据 | heartbeat >20 ms stall 记录为 0；外部 heartbeat 不能排除单线程饥饿或全部主机因素 |

分类可以重叠，不是互斥裁决。离线脚本的比例参考值只用于整理证据，不是安全阈值。

整个进程 CPU 是各线程 CPU 累加；大于 wall 在多线程/native 多核执行中合理。它不是 ServoThread CPU，不能解释为计数器错误，也不能直接解释为 GIL 已确认。候选包括 telemetry、Executor/Python callback、DDS/RMW native、其他 native library 工作；本阶段没有识别具体贡献线程。

**没有使用 syscall trace。** 首次代表性 >20 ms 已捕获，按停止规则不再追加 trace run。现有 wrapper 足以把下一步聚焦到进程内活动，但不足以分解 select、os.read 或 syscall 外时间。A1.1 的正常期 trace 不能回答本次事件的 syscall 归因。CH340/tty/USB local stall 保持 UNKNOWN，低 line utilization 也不等于排除此类 stall。

## 7. Scheduler 时间关联和额外观察

历史 A1-R1 cycle 15 的 idle 505.823 ms 和 cycle 90 的 read 105.997 ms 位于不同周期，不能当成同期事件；本次 57.579 ms 则来自另一 run，更不能与历史 505 ms 拼接。

本次最大 callback idle 出现在 cycle 12：**185.765 ms**，scheduler gap 192.810 ms，wake lateness 174.435 ms。实际 idle 为上个 callback exit `5507291951777` 至当前 entry `5507477717123`。cycle 22 慢 read 为 `5507713829333` 至 `5507771408412`，没有重叠，read 在该 idle 结束后约 236.112 ms 才开始。cycle 12 内 read 最大仅 0.010515 ms。

另一个不可忽略的实测是 cycle 3：callback **135.169 ms**、drain **130.844 ms**，但单次 read 最大仅 **0.035584 ms**。这说明 drain wall tail 不全部发生在 `serial.read()` 内；queue ioctl、外层逻辑、调度或其他路径的具体份额仍 UNKNOWN。没有因此改 drain 或 scheduler。

旧 cycle CPU 列是 **entry-to-entry**，不是当前 callback 的严格区间；例如本次 cycle 4 的 thread 9.420 ms、process 142.865 ms 包围 entry 3→4，而不能直接当成 cycle 3 的精确 callback CPU。新增 read CPU 字段才是围绕指定 read 的配对采样，二者不可混用。

## 8. 更新后的 runtime root-cause tree

```text
A1-RUNTIME NOT PASS
├─ TX（A0/A1 历史证据保留）
│  ├─ pathological write / finite timeout failure：CONFIRMED
│  ├─ line-rate saturation：没有结构性证据
│  └─ 30 ms PySerial failure deadline 保留；wall-bound qualification 未完成
├─ RX callback contributor：CONFIRMED SYMPTOM / CONTRIBUTOR
│  ├─ 历史 single read 105.997 ms；本次 57.579079 ms
│  ├─ 同次 read off-CPU 主导：CONFIRMED 测量特征
│  ├─ R1 等待 + R3 process-local activity/contention：支持 / INFERRED
│  └─ select vs os.read vs post-syscall/GIL、具体线程、USB/tty：UNKNOWN
├─ drain 内其他位置的 tail：CONFIRMED SYMPTOM
│  └─ 本次 130.844 ms drain、最大 read 0.035584 ms；机制 UNKNOWN
└─ callback 外 scheduler idle：独立 CONFIRMED SYMPTOM
   ├─ 历史 505.823 ms；本次 185.765 ms，与本次慢 read 不重叠
   └─ process-local/native 候选 INFERRED，GIL/Executor/host 具体根因 UNKNOWN
```

## 9. Q1–Q8 和唯一下一项

| 问题 | 回答 |
|---|---|
| Q1 >100 ms 是否再次复现？ | 没有；本次只有一个 57.579079 ms >50 ms read。短期未复现不等于排除历史 >100 ms。 |
| Q2 完整同次指标？ | wall 57.579079 ms、thread 0.676633 ms、process 60.748152 ms、voluntary 194、involuntary 0；完整 before/after 在第 5 节和原始 CSV。 |
| Q3 最符合哪类？ | R3 是最有针对性的方向，R1 同时支持；不是 CPU execution，也没有本次 involuntary preemption 证据。具体 blocking syscall/GIL UNKNOWN。 |
| Q4 5 ms read +4 ms drain 能否形成可靠 engineering wall bound？ | 当前不能；57.6 ms 单次 read 及独立 130.8 ms drain 实测远超普通微小 overshoot。不是要求 wall 精确 ≤5.000 ms。 |
| Q5 syscall 时间去哪？ | 未 trace，无法拆分。 |
| Q6 RX 能否升级为 CONFIRMED ROOT FAILURE？ | 不能；保持 CONFIRMED SYMPTOM / CONTRIBUTOR，完整阻塞机制仍未知。 |
| Q7 历史 505 ms 是否同期？ | 不是。历史不同周期、本次不同 run；本次独立 idle 与慢 read 也不重叠。 |
| Q8 唯一下一 modification？ | **C：process-local scheduling / GIL diagnosis**，只推荐，尚未实施。 |

选择 C 的理由是同次 read 同时显示极低 ServoThread CPU、高全进程 CPU、194 次 voluntary CS 和无记录的 heartbeat stall。这个证据比立即改 read/drain、指定 Executor 或归咎 USB 更能确定下一诊断对象。下一阶段应先回答哪个线程/native 工作与长尾同期，以及 GIL 是否参与；不能先宣称 GIL 根因，更不能在本阶段加入 mitigation、priority/affinity、Executor/QoS 调整。

## 10. 停止、暂停和资源清理

必须单独报告本次 teardown 异常：诊断脚本对 launch process group 发 SIGINT，ROS launch 又向子节点转发 SIGINT；motor 节点退出时在 `destroy_node()` 的 `_telemetry_thread.join(timeout=1.0)` 收到第二次 `KeyboardInterrupt`，原有 pause 清理未执行到。launch returncode=0 不能证明正确停止。这是停止流程的异常，不是 RX 根因或硬件 qualification PASS。

确认无残留进程、端口无人占用，并确认日志未执行到原有 pause 后，使用现有 `J7StopAdapter.stop_all()` 补完中断的清理：每轴一次 0x1A pause，无运动目标写、无 retry。**6 次 pause 尝试，6/6 response ACK，success=true**。它属于该 run 的清理，不是第二次诊断。此次清理保留 read timeout 5 ms、write timeout 30 ms；没有修改 production shutdown 实现。

证据：[cleanup_pause_audit.json](/home/lyapunov/bone_drill/results/j14/j14_a1_2_rx_diag_r1/cleanup_pause_audit.json)。最终复查：[final_resource_check.json](/home/lyapunov/bone_drill/results/j14/j14_a1_2_rx_diag_r1/final_resource_check.json)：residual_processes=[]，fuser returncode=1，三个 serial resources free。ACK 是暂停命令响应确认，不是独立物理位移测量。

## 11. Git、冻结边界与最终 verdict

`git diff --check` PASS。与本阶段开始时的 tracked diff 逐文件比较，仅三处 production observation 文件产生本阶段增量；其他六处原有 tracked edits 保持一致。保留原有全部 J14/J13 未提交与未跟踪资产，没有 reset/checkout/commit/push/tag。

最终 Git：`main...origin/main`，9 个原有 tracked modified 文件；A1.2 报告、两个脚本、新测试和 results 仍未跟踪。完整列表见 [git_status_final.txt](/home/lyapunov/bone_drill/results/j14/j14_a1_2_software/git_status_final.txt)，边界核查见 [final_verification.json](/home/lyapunov/bone_drill/results/j14/j14_a1_2_software/final_verification.json)。当前 sourced workspace 使用本次构建的 diagnostic instrumentation，不代表当前运行路径等价于冻结历史代码；最终无本 run 残留节点。

**A1-TRANSPORT：PARTIAL / NOT YET QUALIFIED。A1-RUNTIME：NOT PASS。** 软件验证通过和一次短诊断不能提升为 runtime PASS。本阶段结束，不进入 C 或其他修复。
