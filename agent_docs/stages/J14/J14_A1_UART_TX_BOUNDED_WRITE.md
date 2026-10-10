# J14-A1 UART TX Bounded-Write：软件通过，实机验收未通过

日期：2026-10-08（Asia/Shanghai）。主工作区 `/home/lyapunov/bone_drill`；ROS 2 Jazzy。HEAD `feba8fd00c392ecaab93b53138db7c1d06bce000`，分支 `main`。

本阶段停止于真实 timeout。A1-TRANSPORT **FAIL（严格 30 ms wall deadline 标准）**；A1-RUNTIME **FAIL（启动 Stage A 停止，ACTIVE 未执行）**。没有放大 timeout，没有执行 R2/R3，没有 commit/push/tag。

## 1. J14-A0 per-axis write distribution

A0 两轮的 `transport_phase_timing.csv` 均只有最后 300 个周期：R1 cycle 397–696，R2 cycle 1374–1673。因此下面是保留窗口分布，不是完整运行时长的逐轴分布。均为 ms，百分位使用 NumPy linear interpolation。完整 Tier 1 分别覆盖 697 / 1674 个周期。

A0-R1（10 s session，保留窗口）：

| Motor | n | Mean | P50 | P95 | P99 | P99.9 | Max | >=5 / 10 / 20 / 50 / 100 ms |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 300 | 0.528 | 0.076 | 0.901 | 9.507 | 33.602 | 39.623 | 6 / 3 / 1 / 0 / 0 |
| 2 | 300 | 0.705 | 0.040 | 0.445 | 1.475 | 125.733 | 175.339 | 2 / 1 / 1 / 1 / 1 |
| 3 | 300 | 0.385 | 0.033 | 0.205 | 0.695 | 57.833 | 71.183 | 2 / 2 / 2 / 1 / 0 |
| 4 | 300 | 0.227 | 0.036 | 0.261 | 6.380 | 15.307 | 16.708 | 4 / 3 / 0 / 0 / 0 |
| 5 | 300 | 0.314 | 0.033 | 0.148 | 3.685 | 39.606 | 49.515 | 2 / 2 / 1 / 0 / 0 |
| 6 | 300 | 0.195 | 0.033 | 0.808 | 3.077 | 9.463 | 10.350 | 2 / 1 / 0 / 0 / 0 |
| combined | 1800 | 0.392 | 0.038 | 0.594 | 4.796 | 53.870 | 175.339 | 18 / 12 / 5 / 2 / 1 |

A0-R2（30 s session，保留窗口）：

| Motor | n | Mean | P50 | P95 | P99 | P99.9 | Max | >=5 / 10 / 20 / 50 / 100 ms |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 300 | 0.349 | 0.078 | 1.429 | 4.114 | 14.633 | 14.824 | 2 / 2 / 0 / 0 / 0 |
| 2 | 300 | 0.121 | 0.042 | 0.252 | 1.543 | 6.901 | 7.881 | 1 / 0 / 0 / 0 / 0 |
| 3 | 300 | 0.336 | 0.037 | 0.119 | 0.795 | 56.193 | 78.567 | 1 / 1 / 1 / 1 / 0 |
| 4 | 300 | 0.337 | 0.035 | 0.139 | 1.983 | 53.616 | 75.079 | 1 / 1 / 1 / 1 / 0 |
| 5 | 300 | 0.103 | 0.035 | 0.091 | 0.647 | 9.836 | 12.878 | 1 / 1 / 0 / 0 / 0 |
| 6 | 300 | 0.127 | 0.032 | 0.093 | 1.319 | 13.552 | 18.224 | 1 / 1 / 0 / 0 / 0 |
| combined | 1800 | 0.229 | 0.040 | 0.351 | 2.607 | 29.652 | 78.567 | 7 / 6 / 2 / 2 / 0 |

两窗口合计 3600 次 write：Mean 0.311、P50 0.039、P95 0.470、P99 3.144、P99.9 58.204、Max 175.339 ms；>=5/10/20/50/100 ms 次数为 25/18/7/4/1。

原始分布机读结果：[write_distribution.csv](../results/j14/j14_a1_analysis/a0/write_distribution.csv)。已记录最严重 Top 20：[top20_write_long_tail.csv](../results/j14/j14_a1_analysis/a0/top20_write_long_tail.csv)。Top 20 合并保留窗口与窗口外的全生命周期慢轴事件，未重复计数；窗口外只保存了 slowest-axis wall，单轴 CPU 没有保留，以空值明确标注，不能用 callback thread CPU 替代。其他未保留逐轴样本无法重建。

最严重已记录事件：R2 cycle 178、ID3 write 295.903 ms、TX 299.954 ms、callback 306.780 ms、motion feedback drain 6.008 ms、prior drain 0.027 ms；该事件不在 R2 最后 300 周期窗口中。R1 全生命周期已记录单轴 Max 245.188 ms。

正常簇为数十微秒到低毫秒量级，窗口 P99=3.144 ms；5–30 ms 是少量中间尾部，不能全部称为正常簇。已记录 50–296 ms 的写延迟伴随低 CPU 的例子支持既有 off-CPU long-tail 结论，不重新推翻 A0 根因。

## 2. finite write_timeout 最终选择值

当前未提交候选固定为 **0.030 s（30 ms）**，由 `J14_UART_WRITE_TIMEOUT_S` 指定，在生产 `serial.Serial(...)` 创建时应用。实机失败后未修改该值。这是未完成资格验证的候选实现。

## 3. 选择理由与边界

30 ms 约为窗口 P99 的 9.5 倍，明显高于正常簇，容纳大多数 10–20 ms 中间尾部；两窗口 >=30 ms 仅 6/3600（0.167%），但窗口有限且带删失，不能据此预测完整运行 timeout 率，也不能据此认定这 6 次都是 false timeout。30 ms 显著低于 100–300 ms 病态尾部，远离 283.3 ms warning。

30 ms = 1.5 个 20 ms servo period，只是单次写失败期限，不是整个六轴 transaction 或 callback 的 20 ms deadline，也不保证 Linux 用户态的硬实时返回。第一处 timeout 即中止，避免同周期多次 retry 或后续轴 timeout 累加；既有一次性暂停请求的耗时仍计入 callback。

保留 `serial.write(frame)`，使用 [PySerial 官方 write_timeout / SerialTimeoutException 契约](https://pyserial.readthedocs.io/en/latest/pyserial_api.html#serial.Serial.write)。本机 PySerial 3.5 POSIX 实现先 `os.write`，随后检查 timeout 并调用 `select`；没有更换底层实现。40.613 ms 事件显示配置值未构成实测 30 ms wall 上界；现有数据不能区分该超额发生在 kernel write、select 唤醒还是线程重新获得执行机会。

## 4. 修改文件列表（区分 A1 与既有 A0）

本次 A1 修改：

- `bone_ws/src/h30_imu/scripts/j13_ladrc_shadow_runner.py`：canonical feedback reception age 字段及 legacy alias。
- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_real_uart_adapter.py`：30 ms 常量、专用 TX timeout 异常、未知 partial write 字节数 -1。
- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_six_axis_servo_transport.py`：失败写审计保存、异常附带 cycle audit，继续复用既有 stop。
- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py`：finite serial timeout、sample age telemetry wiring、失败分类日志、既有暂停之后关闭 owned serial handle。
- `bone_ws/src/pus6_motor_driver/pus6_motor_driver/stall_diagnostic_recorder.py`：保留具体 `write_idN` 失败 phase，避免 generic worker phase 覆盖。
- `bone_ws/src/pus6_motor_driver/test/test_j14_a0_instrumentation.py`：补充两处遥测语义测试（文件原先已未跟踪）。
- `bone_ws/src/pus6_motor_driver/test/test_j14_a1_bounded_write.py`：新增正常/延迟/超时/PTY/node failure/close 测试。
- `bone_ws/scripts/j14_a1_analyze.py`：纯离线统计，不打开硬件。
- `bone_ws/src/pus6_motor_driver/test/test_j2_b_bringup.py`、`test_j6_phase3_cartesian_commissioning.py`：旧 501 mA 注入改为冻结上限 +1（1501 mA），同步错误文本断言。
- `bone_ws/src/pus6_motor_driver/test/test_j2_c_vector.py`：匹配既有 J2-C runner 的 500 mA 错误文本；其独立实验限制不变。
- 本报告及 `results/j14/` 中新的软件/实验/离线分析证据。

`j13_ladrc_staged_launcher.py` 的 Git diff 全部是交接时已有 A0 修改，本轮未改。其余 A0 instrumentation 保留。

## 5. 实际行为变化

正常 frame bytes、轴顺序 ID1→ID6、协议、读策略和安全控制不变。单次 PySerial write 配置有限 timeout，捕获 SerialTimeoutException 后不给 motion write 重试，停止当前 transaction，后续轴不继续发送。返回字节数无法确认的失败记录为 -1，而不是宣称 0 字节已发送。

`timed_feedback_age_at_tick_ms = tick_monotonic − last_timed_feedback_receive_monotonic` 是 canonical；`delta_t_runner_queue_ms` 保留相同数值作为 legacy CSV alias。该数值不是 ROS callback 排队延迟；A0 全时段 333.187 ms 也不再如此解释。

Tier 1 原先读取不存在的 `per_axis_feedback_age_ms`，因此恒为 0。现在从既有 `FeedbackSample.feedback_age_ns` 取六轴最大值：这是 ingest 时、更新该轴时间戳前的反馈间隔/旧样本龄期，包含前一周期至本次 TX/drain 的等待。watchdog 判断和门限不变。fault cycle 199 未执行 ingest，其 age/scheduler 默认 0 不代表新鲜反馈；离线 age 分布排除该故障行。

## 6. SerialTimeoutException 传播路径

`serial.write(frame)` → `SerialTimeoutException` → `J7AdapterWriteTimeout`（原始 cause + serial metric）→ six-axis send_cycle 保存失败轴及 cycle audit → 既有 `stop("motion_write_failure")`、delivery uncertain、一次性 `J7StopAdapter` 六轴 0x1A 请求 → 异常向 node 传播 → 具体 TX 日志/CSV/fault snapshot → 既有 STOP latch → launcher Stage A 检测 STOP、关闭 subprocesses。

原有 stop callback 会继续尝试所有六轴一次，捕获各轴暂停异常；没有新增 motion retry。软件 fake 验证了六轴 pause ACK 与 callback 幂等。本次实机未持久化 StopAudit，所以不能单凭 STOP latch 声称六轴 pause ACK 均已在硬件确认。

## 7. 软件验证与回归

- 遥测修正阶段：py_compile PASS；J14 instrumentation 5 项 PASS；2 s dry-run 100 tick，hardware_write_count=0。
- TX/遥测相关测试：13 项 PASS；覆盖正常 bytes/order、2 ms 成功延迟、ID1/3/6 timeout、batch abort、无后续静默 motion、无 retry、暂停幂等、CSV 分类及未知 partial bytes、node STOP latch、关闭 serial handle。
- 实际 PySerial 在本地 PTY backpressure 下产生有限 timeout，不接物理串口；软件测试允许普通 Ubuntu 调度超额，不能证明 CH340 30 ms wall 上界。
- `colcon build --symlink-install --packages-select pus6_motor_driver h30_imu` PASS（两包）。
- 两包全部 Python pytest：547 PASS，0 failure/error/skip；覆盖 J7/J11/J13/J14。首次全回归的三个旧测试不一致被仅在测试层校正；保留初次失败日志。
- h30 C++ protocol：2 PASS（分包/单位、checksum rejection/resynchronization）。
- 软件、Mock、PTY、dry-run 和离线分析：hardware_write_count=0。

[软件验证 JSON](../results/j14/j14_a1_software/verification.json)、[最终 pytest 日志](../results/j14/j14_a1_software/pytest_verified.log)、[JUnit](../results/j14/j14_a1_software/pytest_verified.xml)。

## 8. A1-R1 / R2 / R3 实机结果

R1：**FAIL，在 Stage A startup 停止，未到 runner ACTIVE**。driver 记录 cycle 0–199，共 200 周期（199 完成六轴 batch、1 个失败 batch），首 cycle entry 至 fault exit 为 5.207 s；真实 runner 10 s Static Active Hold 没有执行，Classical/LADRC 的 Active 控制性能没有被测。不能把这个结果描述为控制器 FAIL。

真实 timeout：cycle 199 / Motor ID1，serial 内部 wall=40.613 ms，配置 deadline=30 ms；per-axis wrapper wall=40.673 ms、thread CPU=1.131 ms。未发送 ID2–ID6 的 motion frames。fault callback=61.697 ms、transport_total=61.358 ms，额外约 20.7 ms 包含既有暂停/故障处理，不能当成 RX/drain。

0 个已检测 motor fault / 0 个 1000 ms hard stop；有一次 513.096 ms scheduler warning，阶段没有因普通 warning 中断，而是在明确 TX timeout/STOP latch 后退出。heartbeat 0 个 >20 ms stall。实机末次暂停 ACK 未单独持久化；进程和串口资源已确认释放。

R2 30 s：**NOT RUN**。R3 60 s：**NOT RUN**。R1 未通过，按验收阶梯停止延长。launcher 捕获异常后进程 exit code 0，因此 exit code 0 不能作为实验 PASS；判定使用 Stage A 错误、STOP latch 和 fault CSV。

首个启动尝试仅在既有 `pkill -f` 清理阶段中止：其匹配到了同 shell 的编译文件名，尚未进入 Stage A/启动 driver。保留 `launcher_aborted_before_stage_a.log`，独立启动命令随后才产生本次真实 R1 数据。

[R1 launcher 日志](../results/j14/j14_a1_r1_static_10s/launcher.log)、[driver 日志](../results/j14/j14_a1_r1_static_10s/robot.log)、[fault snapshot](../results/j14/j14_a1_r1_static_10s/fault_snapshot.json)、[退出检查](../results/j14/j14_a1_r1_static_10s/cleanup_check.json)。

## 9. A0 vs A1 数据（非严格匹配 before/after）

没有 A1-R2 完整 Active 数据，无法完成要求的严格 A0-R2 vs A1-R2 before/after，也不能据本表宣称性能改善。以下仅并列 A0-R2 全生命周期与 A1-R1 startup，运行阶段/时长不同。逐轴分布 A0 是最后 300 周期；A1 200 周期全部保存。

A1-R1 startup 逐轴分布（包含 failed write，排除没有发出的轴）：

| Motor | n | Mean | P50 | P95 | P99 | P99.9 | Max | >=5 / 10 / 20 / 50 / 100 ms |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 200 | 0.490 | 0.095 | 0.717 | 1.197 | 36.383 | 40.673 | 2 / 2 / 1 / 0 / 0 |
| 2 | 199 | 0.056 | 0.043 | 0.139 | 0.397 | 0.754 | 0.819 | 0 / 0 / 0 / 0 / 0 |
| 3 | 199 | 0.045 | 0.036 | 0.081 | 0.226 | 0.484 | 0.538 | 0 / 0 / 0 / 0 / 0 |
| 4 | 199 | 0.043 | 0.035 | 0.060 | 0.184 | 0.665 | 0.709 | 0 / 0 / 0 / 0 / 0 |
| 5 | 199 | 0.050 | 0.036 | 0.068 | 0.380 | 0.761 | 0.785 | 0 / 0 / 0 / 0 / 0 |
| 6 | 199 | 0.153 | 0.034 | 0.058 | 0.488 | 15.190 | 17.622 | 2 / 1 / 0 / 0 / 0 |
| combined | 1195 | 0.140 | 0.040 | 0.268 | 0.728 | 18.825 | 40.673 | 4 / 3 / 1 / 0 / 0 |

| Metric | A0-R2 P95 / P99 / Max (ms) | A1-R1 startup P95 / P99 / Max (ms) |
|---|---:|---:|
| Six-axis TX | 6.660 / 31.661 / 299.954 | 4.128 / 21.590 / 40.673 |
| RX/drain | 8.604 / 29.979 / 190.092 | 3.219 / 64.478 / 108.941 |
| Callback total | 18.471 / 58.439 / 306.780 | 10.340 / 68.486 / 113.414 |
| Inter-callback idle gap | 25.365 / 50.983 / 157.735 | 35.041 / 119.263 / 505.823 |
| Wake lateness | 14.609 / 39.409 / 144.055 | 23.590 / 108.965 / 492.106 |

`inter_gap_ms` 是当前 entry − 前次 exit 的空档，不能直接叫 start-to-start period。scheduler/watchdog gap 是距上次反馈 ingest 完成的时间；A1 Max=513.096 ms。A0-R2 已记录 scheduler gap Max=306.230 ms。

A0 Tier 1 feedback age 全为 0，属于缺失 wiring，不能用 0 与 A1 比改善。A1 有效 ingest age Max=513.1 ms。A0-R2 全 runner 的 reception-age legacy alias Max=333.187 ms；仅 ACTIVE ticks 的 Max=232.095 ms、driver feedback reception age Max=232.365 ms。A1 没启动 runner，因此 reception-age / canonical field / runner wall 与控制 tracking 的硬件 before/after 均无数据。字段语义已由软件测试验证。

A1 cycle 90：callback wall 113.414 ms vs thread CPU 6.774 ms、process CPU 8.342 ms；其中 RX/drain=108.941 ms、最大单 read wall=105.997 ms，write TX=4.048 ms。A1 cycle 15：inter-gap=505.823 ms、scheduler gap=513.096 ms，本次 callback 自身只有 7.695 ms；其 recorder thread CPU delta 13.717 ms、process CPU delta 532.388 ms 的采样跨度含前次 callback 之后的空档，不能当成本次 callback CPU 利用率。label 为诊断分类，未确认 GIL/Executor 根因。

两次 baseline heartbeat 和本次 startup heartbeat 都为 0 个 >20 ms 事件，表示独立 observer 没检测到该级 host stall，不能证明所有线程无调度问题。

![Latency evidence](../results/j14/j14_a1_analysis/latency_evidence.png)

## 10. 真实 write timeout count

**1 次，ID1=1，ID2–ID6=0，cycle 199**。只有一次真实失败，无法判定频繁 false timeout；没有以 false timeout 名义增大 deadline。

## 11. 最大 write wall time

A1 serial 内部实际测量 **40.613 ms**（driver error 日志）；CSV per-axis wrapper **40.673 ms**。A0-R2 已记录最大单轴 **295.903 ms**。本次未记录 100–300+ ms 单轴 write，但仅覆盖 startup 200 周期，不能替代长时资格验证。

## 12. 最大 callback / scheduler long-tail

A1 callback Max **113.414 ms**（cycle 90）；inter-callback idle gap Max **505.823 ms**（cycle 15）；wake lateness Max **492.106 ms**；scheduler/watchdog gap Max **513.096 ms**；missing interval Max **513.518 ms**。A0-R2 callback Max 306.780 ms，inter-gap Max 157.735 ms，最大 missing interval 306.793 ms；运行阶段不同，以上只是观测并列。

## 13. RX/drain 是否成为 dominant long-tail

本次 startup 的最大回调内 transport 尾部由 RX/drain 主导：108.941 ms > TX phase Max 40.673 ms；最重的整体停顿则是 scheduler idle gap 505.823 ms。可以指出这两个后续证据点，不能因 A1-R2 未完成就声称已完成分支切换。RX/drain 策略完全没有修改。

## 14. A1-TRANSPORT verdict

**FAIL（按用户要求“在 finite deadline 内失败”的严格验收）**。异常分类、失败轴/周期/耗时记录、abort 和受控故障路径已在硬件发生并记录；但配置 30 ms，实际 serial wall 40.613 ms，超额 10.613 ms（35.4%），没有满足所选 wall deadline。不能宣称 PySerial 已可靠约束 CH340 到 30 ms，也不能宣称仍存在 A0 相同的 100–300 ms 无界 write：本轮没有这种样本。进一步区分内核 write、select 和线程调度超额需要新的限定测量。

## 15. A1-RUNTIME verdict

**FAIL**。Stage A 出现 motion_write_failure，未完成启动稳定性条件，10 s runner Active Hold 未执行；R2/R3 按规则停止。没有通过加大 timeout 让实验跑过去。

## 16. 下一 evidence-driven branch 条件

有条件提出下一步 **A1 TX deadline 超额定位**：在固定同一 transport 路径下分离 kernel write / select wait / thread scheduling，并判断 PySerial finite timeout 是否满足所需工程上界；本次未实施 custom os.write/select。当前不具备 A1 完成、后续 120/300/600 s 延长验证或 J14-B 的资格。RX/drain 与 scheduler 保留为后续证据驱动候选，本轮停止修改和测试。

## 17. Git status

`main`，HEAD 与接手时一致、与 origin/main 一致；未 stage。tracked 修改 9 文件（其中既有 A0 混合在前 6 文件），新增/未跟踪 analysis script、J14 tests、本报告及 `results/j14/`。所有原有 J13 未跟踪结果保留。详见 [最终 git status](../results/j14/j14_a1_software/git_status_final.txt) 和 [完整 diff](../results/j14/j14_a1_software/git_diff_final.patch)。`git diff --check` PASS；控制器、安全、协议冻结文件、RX scheduler 的 diff 均空。

## 18. 明确声明

- Commit：**NO**。Push：**NO**。Tag：**NO**。
- Safety thresholds：**NO 修改**（80-step、283.3 ms、1000 ms、1500 mA 保持）。
- J13 controller / Classical / LADRC / SO(3) FF / IK/FK / shaping：**NO 修改**。
- RX/drain policy / busy-wait / scheduler / QoS / Executor / ROS topic rate：**NO 修改**。
- A1 实机结束后 residual robot process=0、三串口 open handle=0。
- 仅软件/Mock/PTY/dry-run/offline 阶段 hardware_write_count=0；真实 R1 确实打开硬件并发送了 driver 静态 servo commands，不能把该阶段报告成 0 hardware writes。没有 runner ACTIVE control 写入。

所有新证据在新的 `results/j14/j14_a1_*` 命名空间。A0 原始数据保留，不覆盖冻结输出。
