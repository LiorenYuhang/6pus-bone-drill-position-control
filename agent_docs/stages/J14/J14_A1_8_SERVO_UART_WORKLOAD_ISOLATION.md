# J14-A1.8 Servo UART Process-Local Isolation

2026-10-08，`/home/lyapunov/bone_drill` / ROS 2 Jazzy。

**Software PASS；第一项隔离的硬件长尾改善未成立。** A1.8-R1 只运行一次，cycle52 / ID1仍出现自然TX timeout：serial **288.827530 ms**、thread CPU **8.729981 ms**、process CPU **308.016493 ms**、voluntary/involuntary CS **867/1**。首个失败后停止，未执行R2，未做第二种隔离或硬件结果后的production修补。

Process-local contention方向仍 **STRONGLY INFERRED**；具体GIL involvement **UNKNOWN**。选中的workload不是消除故障所需的充分隔离，尚不能称confirmed contributor，也没有排除其他telemetry、executor、native DDS或kernel/tty路径。

唯一下一建议：**下一独立阶段设计并审查 Servo UART separate-process 的最小A/B**，隔开ROS/DDS与UART的执行域，保持串口单一owner及现有协议、安全和deadline。这里只建议，没有实现daemon、IPC或新ROS node。

**A1-TRANSPORT：PARTIAL / NOT YET QUALIFIED。A1-RUNTIME：NOT PASS。** 本次不是qualification。

## 1–2. Motor-driver process/thread map与工作分类

修改前完整阅读 `motor_driver_node.py`、`j7_six_axis_servo_transport.py`、`j7_real_uart_adapter.py`、`stall_diagnostic_recorder.py`及原launch；检查executor、callback groups、telemetry、timers、logging和shutdown。另作一次Mock进程线程快照，硬件写入0；当前RMW为 **rmw_fastrtps_cpp**。

| 执行主体 / 工作 | Python vs native | 分类 | 代码与作用 |
| --- | --- | --- | --- |
| Pus6ServoThread | Python `threading.Thread` | A UART critical；部分B/C | absolute-deadline schedule、target lock、slew/preflight、TX/RX、feedback ingest/STOP；也同步构造snapshot和记录诊断 |
| main / ROS executor spin | Python调用native rcl/rmw等待 | D | `MultiThreadedExecutor(num_threads=3)`；生产main执行spin |
| Executor ThreadPoolExecutor workers | Python workers，按需创建 | A/B/D | sub_cb_group处理steps/meters ingress、re-anchor ACK、pause/configure/single-step；worker/poll各有独立MutuallyExclusiveCallbackGroup |
| Pus6TelemetryThread | Python；publisher进入native rcl/rmw | C，反馈/fault传播有B意义 | async_latest取snapshot后构造JointState、TimedFeedback、TimedCommand、driver diagnostics及raw steps/current/fault消息 |
| Runtime recorder | **无独立线程**；通常在ServoThread | C，failure preservation支持B | counters/checkpoints、per-axis row、GC hook、内存ring；fault/exit时CSV/JSON dump |
| logging / rosout | 在调用的Python线程；native后端 | B/C/D | warn/fatal可以在ServoThread执行；源码未创建独立Python logging worker |
| DDS `dds.shm.wdog`, `dds.ev.0`, `dds.udp.*`, `dds.shm.*`, `dds.asyn.0.0` | 不在Python threading枚举；DDS命名native线程 | D | Mock快照直接观察这些comm名称；具体函数职责仅由名称推断，未测fault-time CPU owner |
| `python3-ust`及若干无Python名称的`python3` task | native/unregistered | E UNKNOWN | 已存在的runtime task；没有据comm名称认定它们就是高CPU来源 |
| shutdown/destroy | Python main；join既有workers | B/C/D | join、dump、六轴pause、serial close、ROS context shutdown |

Mock快照共18个task：4个Python枚举项（MainThread、ServoThread、TelemetryThread、**审计专用**executor-spin线程），14个未注册到Python threading的task。该快照没有executor callbacks，3个配置worker可惰性创建；不是声称真实失败时正好有18线程或3个active executor worker。标准生产main承担审计中额外spin线程的角色。

50 Hz dedicated模式下没有ROS write timer；J7 active模式下legacy background poll timer为None。源代码中没有另一个常规diagnostics timer。原 `_telemetry_lock` 只保护snapshot reference取放，发布在锁外；async仍共享进程/GIL，而且float/list/message构造分布在Servo与telemetry两个Python线程上，不能从线程名称认定已隔离。

[修改前Mock线程证据](../results/j14/j14_a1_8_software/process_thread_audit.json)

## 3–7. 唯一隔离对象与architecture change

选择 **`/pus6/driver_diagnostics` 的11字段performance statistics格式化与ROS publication**。它属于C类，包含target计数、timing、stop-age等统计，未参与driver的安全判定；仓库Python源码未发现此topic订阅者。其他反馈／fault channel有控制或故障观测用途，因此本轮保留。

唯一改动是 **把这个channel的工作延后到UART退出后**：

- stage opt-in `PUS6_J14_DEFER_DRIVER_DIAGNOSTICS=1`，只作用于J7 / async_latest；默认关闭。
- Servo snapshot保留同顺序11个原始数值的tuple，延后float格式化。
- telemetry线程继续其余全部消息，但跳过这一项Float64MultiArray构造与publish。
- 既有worker join后，将最后一个完整snapshot的11字段格式化并输出 `driver_diagnostics_deferred.jsonl`，明确 **latest_only**。不重放ROS历史消息；实验中该live performance topic停止发送样本，publisher名称/type/QoS仍保留。

没有新增queue、worker、共享锁、IPC或串口owner。原diagnostic recorder全部保留，failure同区间CPU/CS/queue/identity及cycle/RX/scheduler仍在ring；`driver_diagnostics_deferred`只是最后成功snapshot的统计，不能替代或冒充失败cycle telemetry。

唯一production修改文件：`bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py`。涉及初始化flag、snapshot的`diag_data`构造、该channel的publish分支和exit export。新增阶段测试、capture harness及Mock benchmark；复用A1.7离线analyzer，未修改它。

critical `_execute_servo_cycle` AST与进入A1.8前比较，**除了diag_data表达式完全一致**。UART/transport/recorder文件及其他既有tracked diff原样；没有改frame bytes、ID1→6顺序、50 Hz/20 ms schedule算法、feedback ingest、pause/fresh/re-anchor、80-step、command/shaper、控制算法、STOP逻辑或其他publisher。

TX **30 ms** / 正常RX **5 ms** / drain **4 ms** / warning **283.3 ms** / hard stop **1000 ms** / protocol current validator **1500 mA**保持。原配置`max_current_mA:500`也未动。50 Hz为配置与代码语义；当前硬件存在long-tail，不能称实际稳定50 Hz。

[A1.8-only diff](../results/j14/j14_a1_8_software/a1_8_only_source_delta.patch) · [进入阶段前diff](../results/j14/j14_a1_8_software/git_diff_before.patch) · [范围与AST核查](../results/j14/j14_a1_8_software/verification_before_hardware.json)

## 8. 软件验证

**652项相关pytest PASS / 44.21 s**（motor driver、control、h30_imu），含新增8项隔离测试；harness额外 **10 PASS**。py_compile、git diff --check、三包colcon build **PASS / 1.56 s**。软件 **hardware_write_count=0**，Mock节点及FakeSerial均不打开物理串口。

验证覆盖：正常six-axis bytes/order和50 Hz period、feedback cache、其余JointState/TimedFeedback/TimedCommand/raw fault channel、实际feedback fault仍STOP、ID1/3/6 timeout立即abort、6/6 pause及fresh HOLD、同次failure metric仍dump、默认模式原publication、被隔离sink即使不能执行也不会阻塞UART或fault topic、exit输出在worker结束后发生。既有suite覆盖80/81-step边界、recovery/reset握手等。

未新增queue/thread，因此没有新的等待关系；原shutdown join边界保持。Mock专用审计初次因fixture读取不存在的MockServoBus计数字段而异常，清理自身Mock进程后以实际Mock类型/无serial对象核对硬件写入0。新增测试初次的ROS array/list和启动frame统计问题已修正，失败日志保留；没有因此修改production。

[软件证据目录](../results/j14/j14_a1_8_software/) · [pytest full](../results/j14/j14_a1_8_software/pytest_full.log)

## 9. 软件benchmark

最终结果为三组交替blocks：每条件300个Mock callbacks、600次直接telemetry calls、3000次FakeSerial writes。对照载入明确保存的pre-A1.8源码，并比较修改后default和isolated模式。Mock移除inter-frame busy-wait以观察bookkeeping；原async worker停止后直接测方法，没有executor spin。最终benchmark保持watchdog阈值；初版临时Mock watchdog override结果不用于结论。没有改变任何硬件配置。

单位 **µs**，格式 **P50 / P95 / P99 / Max**，inclusive线性分位数。

| 指标 | pre-A1.8 | 修改后default | A1.8 isolated |
| --- | --- | --- | --- |
| Fake write wrapper | 8.231 / 8.695 / 10.837 / 83.707 | 8.321 / 8.824 / 10.854 / 44.315 | 8.299 / 8.796 / 11.475 / 44.006 |
| Mock callback wall | 205.794 / 255.458 / 429.302 / 776.989 | 209.097 / 277.228 / 740.871 / 15126.255 | 205.036 / 258.760 / 373.823 / 797.804 |
| telemetry formatting/publication total | **62.388 / 69.628 / 82.171 / 312.144** | 61.818 / 69.207 / 78.476 / 237.910 | **55.933 / 61.427 / 71.181 / 236.592** |
| snapshot handoff（lock+event合计） | 2 / 3 / 4.01 / 7 | 2 / 4 / 8 / 31 | 2 / 3 / 4.01 / 8 |

软件telemetry P50减少约6.456 µs，callback/wrapper主体基本相同。default组15.126 ms孤立Max未定位原因，保留而不解释为改动收益或回归；RMW native activity、GC及OS噪声未隔离。handoff不是独占lock wait，且原记录舍入至1 µs。没有新queue wait。软件benchmark **不能代替硬件结果**。

[最终benchmark全分布含thread/process CPU](../results/j14/j14_a1_8_software/mock_benchmark_frozen.json)

## 10–17、19. R1与A1.7公平对比

双方均原Static Servo launch、NO STRACE、同一J13 YAML、dedicated_thread、async_latest、age_based、runtime diagnostics及同次write counters、相同HostHeartbeat、fresh_state_only；没有LADRC ACTIVE、外加Cartesian target或人工扰动。新增变量仅为上述performance channel deferral。两次均少于300 cycles，详细ring完整保留；失败写不混入成功分布。

| 实际结果 | A1.7-R1 | A1.8-R1 |
| --- | --- | --- |
| timeout count | 1 | **1** |
| cycles记录 / first failed cycle | 279 / 278 | **53 / 52** |
| launch→failed serial exit | 7.627650 s | **2.398694 s** |
| launch→stop request | 7.632169 s | **2.424538 s** |
| total wall含cleanup | 8.049570 s | **2.742294 s** |
| Servo第一cycle→failed serial exit | 6.890238 s | **1.659291 s** |
| failed Motor / position / write_seq | ID6 / 6/6 / 1686 | **ID1 / 1/6 / 325** |
| failed serial / wrapper wall | 61.809583 / 61.945897 ms | **288.827530 / 289.323612 ms** |
| failed write thread / process CPU | 1.549785 / 65.313498 ms | **8.729981 / 308.016493 ms** |
| failed voluntary / involuntary CS | 156 / 0 | **867 / 1** |
| failed out_waiting | 0→0 | **0→0** |
| failed in_waiting | 96→120 | **0→20** |
| delivery | ID6 uncertain；ID1–5返回成功 | **ID1 uncertain；ID2–6未发送** |

A1.8 requested10 bytes、returned=-1、TID529582（driver PID529510）。同次write区间 **26454421598413–26454710425943 ns CLOCK_MONOTONIC**；CPU/CS顺序snapshot扩展 **23.456 µs**。约3.02%的wall体现为该thread CPU，全进程CPU则显著更高，仍符合process-local等待/竞争方向；没有本次os.write/select拆分或直接GIL测量。

成功写分布单位 **ms**，格式 **P50 / P95 / P99 / Max**：

| 指标 | A1.7（1673笔） | A1.8（312笔） |
| --- | --- | --- |
| serial wall | 0.029463 / 0.267440 / 0.934707 / 18.525765 | **0.035218 / 1.018123 / 10.434091 / 19.004500** |
| wrapper wall | 0.061599 / 0.438666 / 1.504138 / 18.566907 | 0.081277 / 1.297512 / 13.451725 / 19.057327 |
| write thread CPU | 0.029801 / 0.076940 / 0.120457 / 0.417533 | 0.035098 / 0.126758 / 0.264796 / 0.521868 |
| write process CPU | 0.032364 / 0.277215 / 0.999243 / 19.757442 | 0.039604 / 1.131304 / 10.863957 / 19.958927 |
| voluntary CS（count） | 0 / 1 / 5.28 / 67 | 0 / 5 / 21.67 / 28 |
| involuntary CS（count） | 0 / 0 / 0 / 1 | 0 / 0 / 0 / 1 |
| 完整成功callback wall | 6.9885 / 12.0156 / 39.75377 / 212.979 | **7.0655 / 23.96765 / 106.28515 / 137.668** |

这里未观察write/callback整体分布改善。成功cycle的legacy entry-to-entry ServoThread CPU P50为6.625→6.869 ms、process CPU P50为11.1615→10.2265 ms；voluntary CS P50为5→4。它们含idle，不是精确callback CPU，也不能替代failed-write counters。完整分布保存在JSON。

不同运行长度、启动阶段占比、系统状态和单次样本均限制因果／概率比较。不能由更早失败、更大的Max就宣布隔离提高了总体故障概率，也不能由某些RX/scheduler Max下降宣布修复。

**R2未执行：R1已有自然timeout，不满足无timeout前提。** 没有30/60 s PASS结论，不得升级selected workload为confirmed contributor。

[完整对比JSON](../results/j14/j14_a1_8_r1_static_no_trace/a1_7_a1_8_comparison.json) · [failed identity](../results/j14/j14_a1_8_r1_static_no_trace/failed_write_identity.json) · [原failed event](../results/j14/j14_a1_8_r1_static_no_trace/tx_write_events.jsonl)

## 18. RX、scheduler、heartbeat及前兆

| 全run max / 记录 | A1.7 | A1.8 |
| --- | --- | --- |
| wake lateness | 419.799 ms | **141.345 ms（cycle16）** |
| callback idle gap | 432.508 ms | **154.435 ms（cycle16）** |
| single read wall | 205.453154 ms | **69.196555 ms（cycle22）** |
| drain wall | 208.667 ms | **133.463 ms（cycle19）** |
| heartbeat stalls >20 ms | 0 | **0** |

A1.8 cycle16 scheduler callback结束距离failed-write enter **902.078 ms**，cycle19 drain尾结束相隔 **724.260 ms**，cycle22 read尾结束相隔 **607.980 ms**。是同run的独立confirmed症状，未与failed write重合，不把它们写成直接原因。

A1.8最后5个完整cycles47–51的TX phase依次 **9.604 / 6.280 / 18.168 / 5.048 / 18.499 ms**，已有间歇性抬高，非单调ramp；最后cycle51 callback21.622 ms刚超20 ms，cycle52 idle0.016 ms、wake1.745 ms，随后ID1 timeout。该前兆不同于A1.7末尾正常窗口，必须保留。此时无紧邻的几十ms RX precursor（cycle49 max read0.719318 ms，cycle51为0.008750 ms）。

所有write before out_waiting=0，failed after也0；正常after的小瞬时值不形成持续积累，RX in_waiting不是TX queue。Heartbeat在独立harness进程中10 ms采样、仅记录>20 ms，未观察到重合host stall；不能解释成host延迟=0或排除所有OS/USB影响。

前50 cycles **2–51**及failed52的完整时间线已导出；failure callback347.962 ms包含既有pause/fresh恢复。failed-cycle未完成drain/ingest字段为0不证明这些阶段正常完成；entry-to-entry CPU不包含随后failed write。

[前50cycle汇总](../results/j14/j14_a1_8_r1_static_no_trace/failure_timeline_50_cycles.csv) · [逐轴数据](../results/j14/j14_a1_8_r1_static_no_trace/failure_timeline_per_axis.csv) · [完整关联](../results/j14/j14_a1_8_r1_static_no_trace/failure_timeline_50_cycles.json) · [heartbeat](../results/j14/j14_a1_8_r1_static_no_trace/host_heartbeat_stalls.csv)

## 20–24. Verdict与唯一后续选择

Process-local contention：**STRONGLY INFERRED，持续存在**。这次小隔离没有消除同类低thread CPU、高process CPU、多voluntary CS的TX long-tail。选中channel不是充分的隔离对象，或竞争来源仍在其他工作中；**selected workload contributor：NOT ESTABLISHED**。没有把一项performance channel的结果泛化成“所有telemetry已排除”。

GIL verdict：**UNKNOWN**。高process CPU可以与TTY/select等待并发，voluntary CS不是GIL专属计数；某个Python/native线程和kernel内部等待占比仍未知。

唯一下一建议为前述 **Servo UART separate-process设计与审查**。下一阶段再做，不在本阶段隔离第二对象、改executor/native架构或调GIL参数。

A1-TRANSPORT **PARTIAL / NOT YET QUALIFIED**；A1-RUNTIME **NOT PASS**。

## 25–26. Cleanup与Git

首个失败checkpoint可读后 **5.346 µs** 请求停止；failed serial exit到stop request为25.843806 ms，期间已有exception/log/recovery路径。既有recovery_count仅1，完成 **6/6 pause ACK→6/6 fresh→HOLD**，resumed=false。pause status的六轴current=0、fault=0；无re-anchor、0x29 measured hold或静态resume。既有`rcl_shutdown already called`及context无效后telemetry错误继续记录，launch返回0不作干净退出证明。

无shutdown信号升级、无残留相关进程，ttyUSB0 / ttyACM0 / ttyACM1全部释放。deferred文件确实落盘，来源是最后成功cycle **51**，不是失败cycle52。

当前branch **main**、HEAD **feba8fd00c392ecaab93b53138db7c1d06bce000**。进入阶段前10个tracked修改和全部J14工作保留；A1.8仅在其中一份production文件增加上述stage开关与单项延后。已分别保存pre-A1.8 diff、A1.8-only diff、最终diff/status。硬件结果后源码未再改动。没有reset、checkout、staging、commit、push或tag；历史raw数据保留。

[R1元数据](../results/j14/j14_a1_8_r1_static_no_trace/run_metadata.json) · [pause](../results/j14/j14_a1_8_r1_static_no_trace/stop_pause_audit.jsonl) · [deferred输出](../results/j14/j14_a1_8_r1_static_no_trace/driver_diagnostics_deferred.jsonl) · [最终资源检查](../results/j14/j14_a1_8_r1_static_no_trace/final_resource_check.json) · [Git status](../results/j14/j14_a1_8_software/git_status_final.txt)

**A1.8已停止。**
