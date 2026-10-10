# J14-A2-Q1F TX Queue Diagnostics Hot-Path Isolation

**Software PASS；F-R1 30 s PASS；F-R2 60 s PASS；Q1F PASS，已 STOP。** 本结论支持正式profile关闭per-frame TX queue诊断及其短时回归，不等于Linux内核根因已确认或修改后长时资格已通过。没有运行120/300/600 s、LADRC、trajectory或扰动；没有commit/push/tag。

[修改前方案](J14_A2_Q1F_MINIMAL_CHANGE_PLAN.md) · [独立handoff复核](../results/j14/j14_a2_q1f_software/handoff_independent_analysis.json) · [六组比较](../results/j14/j14_a2_q1f_software/six_run_comparison.json) · [Q1F-only diff](../results/j14/j14_a2_q1f_software/q1f_only.patch)。原Antigravity Q1资料保持原路径与内容。

## 1. Antigravity handoff复核结果

重新读取P1报告、旧120/300s的metadata/final/raw cycles/raw writes/Bridge/stalls/summary及当前factory、adapter、transport、Parent、Bridge、launch/YAML。独立读取原始JSONL，不调用会覆盖旧结果的analyzer。

| Run | Nominal | Executed / Bridge | Resets | skipped estimate | nominal−executed |
|---|---:|---:|---:|---:|---:|
| P1-R1 30s | 1500 | 1500/1500 | 0 | 0 | 0 |
| P1-R2 60s | 3000 | 3000/3000 | 0 | 0 | 0 |
| 旧Q1-R1 120s | 6000 | 5997/5997 | 1 | 3 | 3 |
| 旧Q1-R2 300s | 15000 | 14990/14990 | 3 | 11 | 10 |
| F-R1 30s | 1500 | 1500/1500 | 0 | 0 | 0 |
| F-R2 60s | 3000 | 3000/3000 | 0 | 0 | 0 |


旧120s核心cycle5210 wall80.957870ms，一次reset。旧300s三次reset对应5210/8591/11972，wall59.150663/93.483413/122.493502ms。两轮Child/Bridge sequence连续、无missing/duplicate，write errors0、fault cycles0。

## 2. 是否认可120/300s根因定位

认可**测量边界定位**：主要长尾发生在write后queue snapshot，即queue_after→_read_tx_queue_state；它拉长cycle并对应deadline phase reset。以下300s事件全部ID5：

| Cycle | Queue-after ms | Serial.write ms | Wrapper ms |
|---|---:|---:|---:|
| 1829 | 22.069135 | 0.027362 | 22.167116 |
| 5210 | 51.644497 | 0.032158 | 51.736002 |
| 8591 | 86.430070 | 0.031926 | 86.515673 |
| 11972 | 115.453645 | 0.044866 | 115.562389 |

相邻seq差均3381；实际start间隔67.620034/67.659372/67.693736s，近似67.62s。四点幅值呈monotonic increasing trend，STRONGLY SUPPORTED；未证明严格线性增长。TTY锁、USB-serial／CH341、ioctl spin、interrupt interaction仍UNKNOWN／INFERRED，没有确认kernel lock root cause。

## 3. tx_queue_diagnostics历史来源

A1.1新增可选queue-before/after，用于检查自然TX长尾前后host-reported backpressure。adapter默认False；原driver仅在enable_runtime_diagnostics开启时使用。A1.6进一步把write processCPU/CS/clock-sync快照绑定到同一选项。A1.9实验UART Child构造adapter时hardcodeTrue，P1正式化时保留该literal。证据为A1.1报告、A1.9-only patch、pre-P1/pre-Q1F source快照；没有把当前Git全量dirty diff当作Q1F新增。

## 4. Production此前为何开启

不是正式profile主动选择：ServoCore active factory统一传tx_queue_diagnostics=True。production profile只调整保留容量及long-tail筛选，没有分离adapter采样开关。

## 5. Qualification此前为何开启

同一factory继承True；qualification保留全窗口rows，意味着每次write均执行before/after查询。六轴50Hz约300个motion frame/s，每frame两次queue snapshot，各snapshot读取out_waiting/in_waiting。必要RX availability读取是另一路径，本阶段保留。

## 6. 最小修改方案

唯一production增量位于j14_process_uart.py的adapter构造：`tx_queue_diagnostics=(not self.production or self.telemetry_mode == 'debug')`。没有新增架构／scheduler／安全参数。关闭正式profile的同步TX诊断，同时保留debug及历史非production实验行为。先输出MINIMAL CHANGE PLAN，再改source。

## 7. 最终profile行为

| 路径/profile | TX queue diagnostics |
|---|---|
| formal production | OFF |
| formal qualification | OFF |
| formal debug | ON，通过driver_telemetry_mode:=debug显式选择 |
| 历史nonproduction experiment | 保持ON |
| legacy_single_process | 原enable_runtime_diagnostics wiring保持 |

保留_read_tx_queue_state、queue_before/after数据结构和debug完整功能。OFF失去queue-before/after、扩展逐write processCPU/CS/clock-sync/native-ID/seq快照；Serial.write wall/threadCPU/timestamps/bytes/error/configured timeout、wrapper、cycleCPU、RX observer、fault/pause仍保留。qualification队列分布为EXPECTED ABSENT，不生成伪造distribution或零值。

## 8. 修改文件

修改production：bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py。新增test_j14_a2_q1f_queue_profiles.py、bone_ws/scripts/j14_a2_q1f_capture.py、j14_a2_q1f_analyze.py、test_j14_a2_q1f_analysis.py，以及本报告和最小方案。新capture基于已有P1capture独立复制，仅改阶段/路径及F-R2的Q1F准入；历史P1/Q1脚本未覆盖。

## 9. Q1F-only diff audit

[pre-Q1F snapshots](../results/j14/j14_a2_q1f_software/before/)保存192个source/config/reference文件；pre git status、pre full diff、HEAD/tag refs保存。191个其余snapshot文件逐字节保持；UART Child整个module AST的唯一差异就是factory布尔参数。pre-hardware与最终stage-only patch均可reverse-check，不执行apply。F-R1/R2间及之后无production修补。

## 10. 安全参数未变

921600 baud；50Hz/20ms；TX30ms、RX5ms、drain4ms；**283.3ms warning、1000ms hard stop**；lease1000ms；target freshness200ms；reconnect80steps；current validator1500mA。frame bytes、ID1→6、pause/fault/latest、startup handshake、socket/RMW/QoS不变。adapter/transport/safety/protocol/controller/geometry/IK/FK source与pre-Q1F bytes一致。

## 11. Scheduler与ownership未变

child_run scheduler/handshake AST完整不变，absolute-deadline loop没有改。Parent、Bridge、两subprocess main、production factory/launch/YAML bytes不变。F-R1 serial owner Child121000，F-R2 Child122761；两轮fuser只识别对应Child，Child单OS task，modules_without_ros=true；三个反馈topic唯一publisher为Bridge。

## 12. 针对性软件测试

14cases全部通过(0.45s)，实际factory monkeypatch为protocol-aware FakeSerial，路径/dev/DO_NOT_OPEN，硬件写入0：production/qualification不调用TX probe；debug每个motion frame记录before/after；正常六轴bytes/order/完整RX；ID1/3/6 timeout保留metric、abort/pause/fault文件；fault/1501mA保留stop；bounded telemetry；历史nonproduction保留ON。

## 13. 完整回归、syntax/build

driver/control/kinematics/h30及script相关回归**790 passed /73.65s**，含J7/J8/J13/J14、default three_process、legacy factory、ownership/进程death/handshake、timeout/RX/safety。计数与14针对性重叠，不相加。py_compile、git diff --check PASS；colcon5packages PASS/4.28s。保留执行日志，无软件失败后删断言或重写旧证据。

## 14. Mock验证

正式normal launch Mock100/100cycles、完整握手/sole Bridge publishers、无reset/skipped/drop；显式legacy正常launch观察完成、无Child/Bridge、资源释放。所有软件hardware_write_count=0。Mock只证明软件与入口，不当成物理UART qualification。两条Mock路径保留已有shutdown exception。

## 15. F-R1 30s

正常ros2 launch pus6_bringup pus6_system.launch.py、hardware_mode:=active、原J13 YAML、qualification、duration30.0、capacity4096和独立Q1F audit directory。没有driver_architecture override或PUS6_J14 experiment flags。1500/1500、9000motion writes、0timeout/fault/missing/duplicate，满足F-R2准入。

## 16. F-R2 60s

同source/正式入口/profile，duration60.0；3000/3000、18000motion writes、0timeout/fault/missing/duplicate。完成STOP，没有120s。metadata的r2_admissible是只针对R1(1500cycles)的helper字段，R2 false不代表F-R2 verdict；q1f_analysis为实际最终判断。

## 17. Nominal slots

F-R1=1500，F-R2=3000，按授权servo window/20ms计算；startup/cleanup不计入window。旧120/300nominal6000/15000，不改写成其实际cycles。

## 18. Executed cycles

F-R1=1500，F-R2=3000；Child与Bridge raw seq分别0–1499、0–2999，完整且无duplicate。Child-cycle JSONL与write数量(6/cycle)相符。全部pause ACK及health fault计数没有过流/硬件fault记录；不将Mock故障注入当真实硬件故障。

## 19. Deadline resets

F-R1/F-R2=0/0。通过相邻expected_ns差必须为20,000,000ns独立复核，没有修改scheduler或加live reset counter。旧120/300分别1/3。

## 20. Skipped slots计数口径

F-R1/F-R2：nearest-slot phase-displacement estimate=0，nominal−executed=0。旧120phase shift3.062261slots→estimate3；旧300三次phase shift合计10.792013slots，逐事件nearest estimate2+4+5=11，而15000−14990=10。两者明列，不能称为同一计数或IPC丢包。OFF未更改原deadline-reset策略。

## 21. Serial.write统计

所有分布以raw strict call边界计算，inclusive linear interpolation；P99.9不是Max。F-R1/F-R2均无TX timeout。完整修改后分布见下表，包含所有9000/18000motion write，不混入startup/config/pause。

F-R1 (30s):

| Metric (ms) | N | P50 | P95 | P99 | P99.9 | Max |
|---|---:|---:|---:|---:|---:|---:|
| serial_write_ms | 9000 | 0.034392 | 0.082214 | 0.113598 | 0.202192 | 0.390253 |
| wrapper_wall_ms | 9000 | 0.050862 | 0.120037 | 0.158413 | 0.260630 | 0.423418 |
| cycle_wall_ms | 1500 | 6.835097 | 7.288598 | 7.700733 | 8.346688 | 9.495147 |
| cycle_interval_ms | 1499 | 19.999301 | 20.098388 | 20.134560 | 20.169082 | 20.186248 |
| wake_lateness_ms | 1500 | 0.162596 | 0.264877 | 0.290424 | 0.376431 | 0.413314 |
| thread_cpu_ms | 1500 | 6.819402 | 7.281359 | 7.685500 | 8.337340 | 9.472103 |
| feedback_ipc_age_ms | 1500 | 0.438617 | 0.617005 | 0.737662 | 0.840420 | 2.382708 |
| feedback_gap_ms | 1499 | 19.995029 | 20.573778 | 20.846130 | 21.677362 | 22.170380 |


F-R2 (60s):

| Metric (ms) | N | P50 | P95 | P99 | P99.9 | Max |
|---|---:|---:|---:|---:|---:|---:|
| serial_write_ms | 18000 | 0.033933 | 0.083361 | 0.104406 | 0.160579 | 0.222165 |
| wrapper_wall_ms | 18000 | 0.050200 | 0.119123 | 0.148866 | 0.223119 | 0.358754 |
| cycle_wall_ms | 3000 | 6.810580 | 7.311992 | 7.597620 | 9.617741 | 11.471005 |
| cycle_interval_ms | 2999 | 19.999826 | 20.129226 | 20.169058 | 20.216888 | 20.283748 |
| wake_lateness_ms | 3000 | 0.178689 | 0.301525 | 0.338234 | 0.413339 | 0.536563 |
| thread_cpu_ms | 3000 | 6.800137 | 7.288129 | 7.575464 | 9.619067 | 11.451585 |
| feedback_ipc_age_ms | 3000 | 0.441695 | 0.599066 | 0.691924 | 0.823643 | 0.983718 |
| feedback_gap_ms | 2999 | 20.004841 | 20.567269 | 20.850362 | 22.779735 | 24.063334 |


## 22. Wrapper统计与六组Max比较

单位ms。wrapper包含call外adapter/transport边界的耗时，不能当成实际Serial.write。关闭queue诊断后仍记录wrapper，F-R1 Max0.423418、F-R2 Max0.358754ms；同长度旧P1为6.392623/23.256763ms。

| Run | Serial.write Max | Wrapper Max |
|---|---:|---:|
| P1-R1 30s | 0.230416 | 6.392623 |
| P1-R2 60s | 0.467778 | 23.256763 |
| 旧Q1-R1 120s | 2.441765 | 73.387695 |
| 旧Q1-R2 300s | 1.091457 | 115.562389 |
| F-R1 30s | 0.390253 | 0.423418 |
| F-R2 60s | 0.222165 | 0.358754 |


## 23. Cycle Wall统计

采用child_cycles的end_ns−start_ns，含cycle计量记账；区别于stall recorder中稍窄的callback_total_ms。F-R1 Max9.495147，F-R2 Max11.471005ms；旧Q1为80.957870/122.493502ms。cycle threadCPU统计也在两张完整表中，测量边界略有差异，不按不同样本Max相减归因。

## 24. Cycle Interval / Wake Lateness

相邻start_ns为cycle interval；start−expected为wake lateness。F-R1 interval Max20.186248/wake0.413314ms；F-R2 interval20.283748/wake0.536563ms。50Hz配置与实际fullcycle记录同时给出，不宣称所有周期硬实时。

| Run | Cycle Wall Max | Cycle Interval Max | Wake Max | Thread CPU Max |
|---|---:|---:|---:|---:|
| P1-R1 30s | 13.689969 | 20.242864 | 0.429925 | 13.687561 |
| P1-R2 60s | 30.583256 | 30.703355 | 10.818032 | 30.558697 |
| 旧Q1-R1 120s | 80.957870 | 81.060466 | 14.538431 | 80.562908 |
| 旧Q1-R2 300s | 122.493502 | 122.615672 | 9.674047 | 122.540320 |
| F-R1 30s | 9.495147 | 20.186248 | 0.413314 | 9.472103 |
| F-R2 60s | 11.471005 | 20.283748 | 0.536563 | 11.451585 |


## 25. Feedback Gap

F-R1 Max22.170380、F-R2 Max24.063334ms。旧120/300 Max93.845032/134.440858ms。本次无IPC packet缺失；gap改善与Child cycle收敛一致。30/60s窗口较短，不能单凭Max对比确认旧300s所有迟发事件已长期消失。

## 26. Feedback IPC Age

F-R1 Max2.382708、F-R2 Max0.983718ms。定义为Child feedback-ready到Bridge receive，不是下游controller总latency。Fast DDS事实与QoS保持；remote graph history/depth UNKNOWN/0不是实际本地队列深度。

| Run | Feedback Gap Max | IPC Age Max |
|---|---:|---:|
| P1-R1 30s | 26.632080 | 1.995654 |
| P1-R2 60s | 43.549389 | 1.864654 |
| 旧Q1-R1 120s | 93.845032 | 2.414872 |
| 旧Q1-R2 300s | 134.440858 | 2.454091 |
| F-R1 30s | 22.170380 | 2.382708 |
| F-R2 60s | 24.063334 | 0.983718 |


## 27. Pause ACK

每轮StopAudit六轴response_received=true、timeout=false、error=null、current0/fault0，success=true。Child final pause_success=true。窗口结束各一次六轴0x1A pause，无blind motion retry；timeout/error的pause路径另由FakeSerial测试。

## 28. Cleanup

每轮Child/Bridge exit0、parent joins/destroy记录、0residual process、三个串口free；最终再次核查结果同样通过。control/kinematics仍有rcl_shutdown already called/exit1，为已知独立问题，本阶段未修。不能把launch return0当作全ROS系统干净退出。Bridge末尾EOF healthfalse是停止后状态，不是active-period IPC failure。

## 29. 旧120/300证据保留

原目录、raw、metadata、分析、日志不删、不覆盖、不重命名。运行前后文件集合/size/mtime一致，记录old_q1_preservation_final.json。分类为PRE-FIX QUALIFICATION EVIDENCE和ROOT-CAUSE / INSTRUMENTATION EFFECT EVIDENCE，不把旧结果改叫垃圾FAIL。新增比较与audit只写Q1F目录。

## 30. 是否允许重新开始120→300→600

Q1F短时软件/硬件PASS，可作为新长时qualification的输入。**因production source已改变，新资格链强制重新120→300→600**；旧120/300不能与未来600合并。当前未启动该链。移除诊断调用已由source/Fake测试/actual null queue records验证，长期运行仍须新版本实测；没有证明Linux内核完全无问题。

## 31. Git status

branch main，HEAD feba8fd00c392ecaab93b53138db7c1d06bce000保持；原大量J13/J14 dirty/untracked保留。既有tracked full diff与阶段开始逐文件一致，因为本次唯一修改的production模块原本untracked。没有reset/checkout/clean/stash/revert/staging/commit/push/tag。最终status/full diff、only diff和tag比较保存，不能把整个工作区diff算作Q1F。

## 32. 唯一下一建议

用户审查后，进入**修改后全新的Long-Duration Static Qualification：120s→300s→600s**，沿用当前正式profile、冻结参数与完整证据容量。当前保持STOP，不追加任何硬件run。
