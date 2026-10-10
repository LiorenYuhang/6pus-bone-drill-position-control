# J14-A2-Q3J Parent Active-Command Runtime Starvation Fix

本阶段完成 Parent 最小软件修复，60 s / 300 s 动态 Mock 和 60 s 静态 Mock 的 Parent lease 目标通过。硬件验证未执行，hardware_write_count=0。整个 runtime 不宣称完成硬件 qualification。Mock Child 计算长尾、测试组合 Context 污染和 stage relay 退出超时作为独立未解决项保留。

## A. Source-level root cause 与证据等级

**CONFIRMED（软件）**：`production_driver.main` 原来的 `MultiThreadedExecutor(num_threads=3)` 调用 rclpy `Executor._wait_for_ready_callbacks`。其 `while not yielded_work` 循环反复创建 wait set、扫描 subscriptions/services/events，却少量执行实际回调。8 s 剖析 A/B 中，旧路径 `get_nodes` 31212 次、`EventHandler.is_ready` 311972 次全部 False、`_take_subscription` 66 次；修复后分别为 802、8010、400。运行窗口内接收由 65 条变为 400 条。

主线程占据绝大部分 native CPU ticks，executor pending / pool queue 最多 1，没有证明 FIFO/队列增长。回调仅做六轴数据处理和单槽覆盖；原 IPC worker 有 `select(..., 0.010)`，不是没有 sleep 的 sender 忙循环；production telemetry worker 直接返回，feedback 仍由 Bridge 独占。

**STRONGLY SUPPORTED**：MTE 将 handler 提交线程池后，存在 ready subscription 数据尚未被 worker take 的时间窗口；wait-set 继续唤醒／扫描，竞争 Python 执行机会。inline executor 立即取走数据，消除这一放大窗口。原始 R1C 与软件复现的症状和最小 A/B 修复一致。

**UNKNOWN**：具体 native Fast DDS/rmw 唤醒条件没有 native trace；GIL 的独立贡献没有被分离测量，不能写“GIL 是最终已验证根因”。R1C 本身没有本阶段的 executor profile，故硬件事故的精确 native 叶因仍非 CONFIRMED。

证据：[源码审计](../results/j14/j14_a2_q3j_software/source_audit_final.json)、[剖析比较](../results/j14/j14_a2_q3j_software/executor_profile_comparison.json)、[旧 8 s 剖析](../results/j14/j14_a2_q3j_baseline_event_profile_8s/parent_main_profile.txt)、[新 8 s 剖析](../results/j14/j14_a2_q3j_fixed_profile_8s/parent_main_profile.txt)。剖析会放大 timing 开销，正式指标使用无 cProfile 的独立运行。

## B. CPU 从约 1% 到接近 100% 的原因

无动态 command 时，ROS executor 阻塞等待，既有 IPC 线程每约 100 ms 刷新 lease。R1B 原始 Parent send 时间／CPU 字段核对得到间隔 CPU 中位数约 1.210%。有动态命令时，消耗来自上述 executor 扫描放大，而不是合理的每秒 50 次六轴 copy/JSON 工作。旧 Mock 30 s 输入 1500 条，实际只接受 702 条，Parent 0.5 s 采样 CPU 中位数 72.609%、最大 97.885%。

## C. 超过 1000 ms 的 lease starvation

IPC 位于 Parent 的 Python 专用线程，仍需要获得执行机会。主线程放大使 callback / sender 进度下降。原 R1C 最长 IPC 间隔 1045.614037 ms 内，process CPU 增加 1125.387120 ms，sender thread CPU 只增加 29.728618 ms。最后 lease 至 Child STOP 为 1005.654713 ms。Child 的冻结 1000 ms lease STOP 正确。

12 s 剖析 Mock baseline 也真实停止于 `parent_lease_expired`，最后已成功发送 lease 至 STOP 为 1013.125330 ms；这轮是受剖析扰动的复现，不作为正常负载定量 baseline。普通 30 s baseline 未达到 expiry，但有 11 次 >250 ms gap 和显著降频，不能称 PASS。

[历史原始字段复核](../results/j14/j14_a2_q3j_software/historical_r1b_r1c_recheck.json)；[剖析失败记录](../results/j14/j14_a2_q3j_baseline_profile_12s/summary.json)。

## D. Production 修改与 command 路径

仅改 `bone_ws/src/pus6_motor_driver/pus6_motor_driver/production_driver.py`：为 `ProductionDriverNode` 选择 `SingleThreadedExecutor`；legacy 仍选择原 `MultiThreadedExecutor(3)`。没有增加 production process / command authority，没有改变发送频率或 mailbox。executor 在 node factory 后选择，初始化失败时仅增加必要的 None cleanup guard。

callback 路径仍为 `_handle_command_joint_steps`（长度/模式检查）→ base `_enqueue_latest_target`（recovery / reconnect gate，`_target_lock` 单槽、seq）→ J14 `_enqueue_latest_target` → `ProcessClient.update`（`client.lock` 单槽）。无 UART、无文件 IO、无 command FIFO。IPC thread 在 `send_state` 中序列化非阻塞 socket packet，Child 原 bounded receive / validation / lease / 50 Hz servo 原样使用。

新增 stage-only Mock stress 脚本和 4 项 Parent 测试；Child lease 观测由测试进程透明 wrapper 完成，不改 Child production source。兼容 relay 仍为已有 Q3 脚本。

[本阶段 production patch](../results/j14/j14_a2_q3j_software/q3j_only.patch)。

## E. Frozen safety：0 变化

1000 ms lease、Child 50 Hz、1500 mA、raw/software travel bounds、80-step reconnect、timeout recovery、六轴 pause、UART/Bridge ownership、startup handshake、latest-target semantics 全部保持。`j14_process_uart.py`、协议、控制算法、Classical/LADRC、IK、geometry、Q3 relay、launch/YAML 没有本阶段修改。修改前保存的 Python 源码逐文件 byte 比较，仅 `production_driver.py` 发生变化。

[源码保护检查](../results/j14/j14_a2_q3j_software/source_preservation_check.json)。

## F. 修复前后指标

Parent CPU 为约 0.5 s 采样，包含测试观察线程开销；CPU % 为一个 core=100%。IPC gap 是相邻成功 state packet 的 lease timestamp 差；命令 ingress 从实际 driver callback 记录，不用 publisher 配置频率代替。Child lease age 直接采于 Mock Child 每次 cycle 入口，不是 feedback Message Age。统计仅在已保存 measurement window 内；shutdown packet 不纳入稳定流 gap。

| 运行 | CPU % P50 / P95 / Max | IPC gap ms P50 / P95 / P99 / P99.9 / Max | ingress / published | 实际 ingress Hz | Child lease age Max ms |
| --- | --- | --- | --- | --- | --- |
| 修复前动态 30 s | 72.609 / 92.936 / 97.885 | 20.500 / 164.427 / 276.088 / 379.660 / 441.436 | 702 / 1500 | 23.39994 | 439.280 |
| 修复后动态 60 s | 8.493 / 10.024 / 10.890 | 20.484 / 20.710 / 21.253 / 22.234 / 30.831 | 3000 / 3000 | 49.99984 | 101.262 |
| 修复后动态 300 s | 8.346 / 10.253 / 12.967 | 20.494 / 20.722 / 21.291 / 30.678 / 33.636 | 15000 / 15000 | 49.99998 | 31.041 |
| 修复后静态 60 s | 1.978 / 2.372 / 2.720 | 102.138 / 102.654 / 102.983 / 104.096 / 105.299 | 0 / 0 | 0.00000 | 103.326 |

60 s 动态 lease Max 101.262 ms 位于首条动态命令尚未到达的静态→动态过渡；稳定 command-driven IPC Max 30.831 ms。所有值由原始字段提取。[机读比较](../results/j14/j14_a2_q3j_software/before_after_comparison.json)。

## G. 60 s Active Mock

PASS。3000/3000，seq 1→3000，134 种六轴动态向量。0 expiry、0 send_not_accepted、0 send seq duplicate/regression、0 Child fault/timeout/safety STOP，0 >250 ms IPC gap；hardware_write_count=0。Child 运行窗口有 3000 cycles，interval Max 21.096 ms。实际 timed feedback / JointState relay 输出均被 observer 接收到。

[summary](../results/j14/j14_a2_q3j_active_mock_60s/summary.json)。

## H. 300 s Active Mock

Parent lease 修复目标 PASS。15000/15000、seq 1→15000、134 种动态向量；0 expiry / send 异常 / Child fault / deadline safety STOP；0 >250 ms Parent gap；hardware_write_count=0。

**Child timing 的独立限制**：运行窗口 14990 cycles，cycle interval Max 110.384 ms。cycle12883 wall 110.035533 ms、当前 cycle thread CPU 109.289942 ms，voluntary CS=0；同期 Parent 持续约 20 ms 发送 seq12670→12677。另有 43.659 / 56.235 / 70.332 ms Child 周期。这是 Mock Child 的计算长尾，不得称为真实 `serial.write` 故障。其具体 allocation/compute 叶因 UNKNOWN；没有修改 Child 或借此放宽门限。300 s 不能据此宣称 Child 已获严格周期 qualification。

Recorder `thread_cpu_delta_ms` 和 `gc_pause_ms` 在 `start_cycle` 按上次入口采样，不能与同一行 callback wall 当作同一周期 CPU 配对；相关下一入口 full CPU/GC 字段未被现有 300-row timing ring 保留，full tier1 summary 又不含 CPU/GC，故没有证据确认 GC 叶因。分类标签 `B_IN_CALLBACK_UART_BLOCK` 只是一项 heuristic，不能替代以上字段解释。

[summary](../results/j14/j14_a2_q3j_active_mock_300s/summary.json)、[当前 cycle 长尾证据](../results/j14/j14_a2_q3j_software/child_mock_tail_audit.json)、[recorder 字段语义复核](../results/j14/j14_a2_q3j_software/child_mock_tail_recorder_crosscheck.json)。

## I. 静态 60 s regression

PASS。0 commands，既有约 10 Hz heartbeat 保持；0 expiry / Child fault / IPC 异常。CPU P50 1.978%，IPC P50 102.138 ms / Max 105.299 ms；hardware_write_count=0。

[summary](../results/j14/j14_a2_q3j_static_mock_60s/summary.json)。

## J. Tests / build / 失败证据

针对性 54 项 PASS，覆盖 entry executor selection、真实 callback、invalid vector、500 command burst、latest overwrite、seq、fault rejection、实际 50 Hz ROS stream、idle heartbeat、shutdown；现有 process/production/relay 测试包含 timeout abort、80/81-step reconnect、Child/Bridge death、pause等。

完整 relevant package 789 项 + script 43 项，独立测试进程共 832 项 PASS，覆盖 motor driver/J7/J8/J13/J14、control、kinematics、h30_imu、Q3 harness。**合并到单进程的第一次运行 FAIL**：827 PASS、1 FAIL、4 setup ERROR，均为后续 ROS Context 重复 init。分组完整重跑通过；具体污染 fixture 尚未确认，不修改生产 Context/shutdown 来掩盖测试污染。

`pus6_motor_driver` colcon build PASS（1.49 s），3 个新增/修改 Python 文件 py_compile PASS，`git diff --check` PASS。初次 py_compile 使用了错误的相对路径而未编译，随后更正；该中间错误也记录在 compile_checks.log。

[完整合并失败日志](../results/j14/j14_a2_q3j_software/full_regression.log)、[package](../results/j14/j14_a2_q3j_software/package_regression.log)、[scripts](../results/j14/j14_a2_q3j_software/script_regression.log)、[targeted](../results/j14/j14_a2_q3j_software/targeted_tests.log)、[build](../results/j14/j14_a2_q3j_software/build.log)。

## K. Git 与证据保护

首次操作 `git status --short`，完整 dirty/untracked 与源码 snapshot 已保存。既有 tracked diff 前后 byte 相同，HEAD 相同。`git diff --stat` 仍为原用户工作：12 files、1142 insertions、76 deletions。`production_driver.py` 原已 untracked，故上述 stat 不包含本阶段 production patch；Q3J 独立 production delta 为 **1 file、12 insertions、4 deletions**。另外新增 stage harness / targeted test / 本报告及实验数据。

原 J14 namespace 文件没有发现本阶段开始之后的 mtime；这只是 mtime 检查，不冒充历史 raw data 全量 byte snapshot。无 reset/checkout/clean/stash、无 commit/push/tag。

[Git 保护验证](../results/j14/j14_a2_q3j_software/git_preservation_final.json)、[独立 stat](../results/j14/j14_a2_q3j_software/q3j_production_diff_stat.txt)、[最终 status](../results/j14/j14_a2_q3j_software/git_status_final.txt)。

## L. Q3-R1D 条件与 STOP

Parent active-command starvation 修复已满足本轮规定的软件前置检查，可提交 Q3-R1D Classical 60 s 的后续审查；**不无条件宣布整机 runtime / Child timing 已 qualified**。Mock Child 110 ms 计算尾部、测试 Context 组合失败、兼容 relay cleanup 超时仍需单独确认。Mock 未运行 Base IMU/IK/Classical 全链，也不能证明物理 UART timing。

所有正式 Mock 正常停止原因是 harness `parent_shutdown`，Child 一次 pause 后关闭，Parent/Child/Bridge joined。现有 stage relay 收到 SIGINT 后 cleanup 等待 5 s 超时，harness 仅回收自己启动的 relay（returncode=-9），未改 relay；**运行指标 PASS 不等于所有进程 clean shutdown PASS**。第一轮 baseline 还因 relay timeout 未完成 summary，但原始 Parent/Child/IPC 日志已保留，第二轮 nominal baseline 完整。

最终无本阶段残留进程，串口资源未占用。所有软件 Mock hardware_write_count=0。本轮已 STOP：没有执行 active 硬件、没有自动进入 R1D、没有 commit/push/tag。

[资源退出核查](../results/j14/j14_a2_q3j_software/resources_final.json)。
