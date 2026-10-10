# J14-A1.7 Low-Intrusion Natural Timeout Capture

2026-10-08，`/home/lyapunov/bone_drill` / ROS 2 Jazzy。

**A1.7 outcome：CAPTURED。** A1.7-R1 在 **NO STRACE** 条件下捕获一个自然 `SerialTimeoutException`，cycle278 / ID6，随后停止。未执行 R2、LADRC ACTIVE、人工扰动或额外 Cartesian target；未实现下一修改。

唯一主分类：**A — process-local scheduling / contention：STRONGLY INFERRED**。**GIL involvement：UNKNOWN**，没有直接确认 GIL 的证据。具体 syscall 等待位置仍 UNKNOWN。

唯一下一 modification 选择：**A — process/thread architecture modification**。建议下一独立阶段做 driver 内 Servo UART 与非 UART 工作隔离的最小对照，保持协议、timeout 和控制算法不变，比较同区间 CPU/CS 与自然 TX long-tail。当前尚不能指定哪一个线程是直接责任方，本阶段没有实施隔离或改代码。

**A1-TRANSPORT：PARTIAL / NOT YET QUALIFIED。A1-RUNTIME：NOT PASS。** 本阶段为 failure capture，不是资格验证。

## Run 与停止证据

R1 完整记录 **279 cycles（0–278；278个完整正常事务后，cycle278失败）**。从启动 launch 到请求停止 **7.632169 s**；总 wall **8.049570 s** 含启动、既有 pause/fresh 路径和清理，低于30 s范围。

harness 读到第一条 `UART TX failure` 后，确认既有 recovery `COMM_TRANSIENT` checkpoint 已关闭文件并可读，再发送一次 SIGINT。确认 checkpoint 到 stop request 为 **5.400 µs**；failed serial.write exit 到 stop request 为 **4.519074 ms**。完整 failed-write CSV/JSONL 随既有 destroy dump 落盘后核对一致。没有强制 timeout、盲重试或继续等待第二个故障。

SIGINT 请求期间，原 production path 完成一次 communication-transient pause、fresh-state-only recovery 并进入 **HOLD / motion inhibited**；6/6 pause ACK 已持久化。没有 re-anchor、0x29 measured hold、静态 resume 或第二次 recovery。失败 callback 的111.863 ms包含这次既有恢复，不能全算作 serial.write。

退出日志仍有既有 `rcl_shutdown already called` 及 context 关闭后的 telemetry publish error；未修改 shutdown。launch returncode=0不作干净退出证明。最终核查无残留相关进程，ttyUSB0 / ttyACM0 / ttyACM1 均空闲。R1已经 CAPTURED，按 Case A 不执行 R2。

[运行元数据](../results/j14/j14_a1_7_r1_static_no_trace/run_metadata.json) · [robot log](../results/j14/j14_a1_7_r1_static_no_trace/robot.log) · [pause 审计](../results/j14/j14_a1_7_r1_static_no_trace/stop_pause_audit.jsonl) · [最终资源检查](../results/j14/j14_a1_7_r1_static_no_trace/final_resource_check.json)

## 同一次 failed write 的20项字段

| 字段 | 实测 / 审计值 |
| --- | --- |
| 1. cycle_seq | **278** |
| 2. write_seq | **1686**（包括启动配置等 adapter writes，不等于cycle计数） |
| 3. Motor ID | **6** |
| 4. transaction position | **6 / 6**，attempted IDs=[1,2,3,4,5,6] |
| 5. requested bytes | **10** |
| 6. returned bytes / delivery | **-1 / UNKNOWN partial count；ID6 delivery uncertain**；ID1–5 write返回成功，无未尝试轴 |
| 7. configured write_timeout | **30.000 ms** |
| 8. serial.write wall | **61.809583 ms** |
| 9. transport wrapper wall | **61.945897 ms** |
| 10. write thread CPU | **1.549785 ms** |
| 11. write process CPU | **65.313498 ms** |
| 12. RUSAGE_THREAD voluntary CS delta | **156** |
| 13. RUSAGE_THREAD involuntary CS delta | **0** |
| 14. out_waiting before | **0** |
| 15. out_waiting after/failure | **0** |
| 16. in_waiting before | **96** |
| 17. in_waiting after/failure | **120** |
| 18. native TID | **488240**（driver PID488165） |
| 19. write enter CLOCK_MONOTONIC | **24483564809256 ns** |
| 20. write exit CLOCK_MONOTONIC | **24483626618839 ns** |

wall enter/exit 紧邻同一次 serial.write。顺序 thread/process CPU和getrusage快照围绕该次调用，边界略宽；snapshot区间 **24483564807416–24483626634947 ns**，扩展开销 **17.948 µs**。不能把它们说成纳秒级完全同起止，但这不是相邻cycle entry的替代计数器。event与CSV的每个字段均核对一致，epoch ns保留整数精度。

NO STRACE意味着 **os.write / select / Python恢复各自elapsed没有本次直接测量**。30 ms是PySerial配置的deadline，不是能在ServoThread未恢复运行时强制抢占的wall上限。这里确认的是自然超出预算并抛异常的真实事件。

[原失败事件](../results/j14/j14_a1_7_r1_static_no_trace/tx_write_events.jsonl) · [配对CSV](../results/j14/j14_a1_7_r1_static_no_trace/tx_queue_state.csv) · [身份与事务位置](../results/j14/j14_a1_7_r1_static_no_trace/failed_write_identity.json)

## 失败前50 cycles时间线

全部279 cycles的数据在300-cycle ring内。提取 **cycles228–277（50个前置周期）及失败cycle278**，提供51行汇总、306条对应per-axis write记录及完整JSON关联。字段包含callback wall、idle gap、wake、feedback scheduler gap/age、TX、per-axis wall/thread/process CPU/CS、RX/drain、max read和heartbeat重合。

| cycles228–277 指标 | median | max |
| --- | --- | --- |
| callback wall | 6.872 ms | 8.263 ms |
| inter-callback idle gap | 13.2575 ms | 45.925 ms |
| wake lateness | 0.192 ms | 33.225 ms |
| six-axis TX phase | 3.4955 ms | 4.213 ms |
| single serial.write wall | **0.027591 ms** | **0.686531 ms** |
| UART drain wall | 2.700 ms | 3.945 ms |
| max single read wall | 0.017390 ms | 0.993159 ms |
| legacy entry-to-entry process CPU | 9.6965 ms | 53.625 ms |
| legacy entry-to-entry ServoThread CPU | 6.5195 ms | 8.304 ms |

最后五个完整周期273–277：callback6.971–7.271 ms、TX3.538–3.665 ms、drain2.564–2.870 ms、max read≤**0.026082 ms**；未见连续写延迟爬升。失败同batch的ID1–4 serial wall为0.019579–0.042341 ms；**ID5先出现0.462764 ms**（thread0.030774 ms、process0.528975 ms、voluntary2/involuntary0），然后ID6突增到61.809583 ms。

因此单笔失败表现为突然long-tail，没有跨前几cycle持续抬高。并非所有前兆都不存在：同batch ID5短暂变慢、失败cycle entry已有轻度wake延迟及process CPU增量。它们只支持时序相关观察，未证明直接因果。

**CPU边界说明**：原recorder的`process_cpu_delta_ms`、`thread_cpu_delta_ms`、CS deltas在相邻entry之间采集，包含idle；不是纯callback CPU。失败cycle中26.657/7.429 ms来自前一entry到本entry，不包含后续61.810 ms failed write，不能冒充失败CPU。导出明确命名`entry_interval_*`，另列每笔write的严格配对CPU/CS。不存在精确失败callback CPU的测量，不补造。失败cycle的callback/read汇总含pause/fresh；未完成post-TX drain/feedback age字段的0不证明正常完成这些阶段。

[51行汇总CSV](../results/j14/j14_a1_7_r1_static_no_trace/failure_timeline_50_cycles.csv) · [逐轴时间线CSV](../results/j14/j14_a1_7_r1_static_no_trace/failure_timeline_per_axis.csv) · [完整关联JSON](../results/j14/j14_a1_7_r1_static_no_trace/failure_timeline_50_cycles.json) · [分析汇总](../results/j14/j14_a1_7_r1_static_no_trace/application_analysis.json)

## RX与scheduler的独立分析

**no immediate RX precursor observed**：前50 cycles最高read0.993159 ms，最后五周期均低于0.027 ms；failed transaction prior drain为0.572 ms。没有紧邻TX失败的历史级30–200 ms read/drain tail。failed cycle aggregate max read0.099294 ms也未出现长read，但包含随后既有fresh读取，不能全部归于失败前。

本次run早期确实捕获独立RX异常：**cycle6 read205.453154 ms**，thread CPU2.649246 ms、process CPU217.417041 ms、voluntary582/involuntary0；cycle6 drain208.667 ms、callback212.979 ms。read区间 **24476863015962–24477068469116 ns**，距failed write约6.5 s，不与失败重合。cycle14另有7.050706 ms read。没有修改RX。

| 本次run scheduler事件 | 时间 / 距failed write | 解释边界 |
| --- | --- | --- |
| cycle9 wake419.799 / idle432.508 / feedback scheduler gap439.604 ms | entry24477529315418 ns；callback结束距failed write **6028.086 ms** | 独立confirmed症状；不能确认导致cycle278 TX失败 |
| cycle267 wake33.225 / idle45.925 ms | callback24483345707217–24483352538172 ns；结束距failed write **212.271 ms** | 后面有10个完整正常事务268–277，不能拼接成连续TX故障 |
| cycle270 wake7.613 / idle20.854 ms | 结束距failed write157.351 ms | 分离事件 |
| cycle278 wake7.771 / idle20.593 ms | entry24483560322176 ns；failed write在entry后**4.487080 ms**开始 | 本次有轻度scheduler precursor，idle等待已结束，不在failed write内部 |

failure callback111.863 ms、TX phase65.729 ms为本次测量；后续pause/fresh用时41.839207 ms包含在callback。不得从长callback反推“OS preemption造成TX timeout”。本次没有failed write的involuntary CS增加证据。历史A1.5/A1.6时间线不参与本次因果拼接。

[完整callback原始数据](../results/j14/j14_a1_7_r1_static_no_trace/stall_diagnostic_timing.csv) · [全程cycle summary](../results/j14/j14_a1_7_r1_static_no_trace/cycle_summary_full.csv) · [RX事件](../results/j14/j14_a1_7_r1_static_no_trace/rx_read_events.csv)

## Process-local证据、heartbeat与queue

**CONFIRMED**：failed write wall61.810 ms时，本ServoThread CPU仅1.550 ms（约2.51%），而进程CPU65.313 ms；约**63.764 ms**的进程CPU增量来自本线程以外的执行（含极小采样边界偏差）。voluntary CS156、involuntary0。其他线程活动显著，本线程大量elapsed未体现为thread CPU。

**STRONGLY INFERRED**：在这个NO STRACE事件中，process-local scheduling/contention是当前最支持的主方向；初期RX tail也呈现相似计数器形态，仅作同run的模式佐证，不把它当作TX前兆。

**UNKNOWN**：GIL是否参与、哪个Python/native线程占CPU、os.write或select是否等待、TTY/USB/CH340内核路径各耗时多少。别的线程可以在UART等待期间独立工作，因此高process CPU/voluntary CS不能单独排除I/O wait，更不是GIL CONFIRMED。involuntary0不排除所有OS调度影响。

独立harness进程的HostHeartbeatObserver保持10 ms周期、仅记录delay>20 ms，**全run 0条stall**，无与failed write重合的host-wide stall记录。未记录小于阈值的延迟，不能声称host最大延迟=0或完全排除host影响。

前50cycles的300笔write，**out_waiting_before全部0**；after最大10（285笔暂时为10），下一笔before又回0，未见持续积累。failed write out_waiting **0→0**。in_waiting96→120为RX bytes，不能写“TX queue堆满”，也不能由queue瞬时0排除USB/kernel等待。此处没有本次kernel/device直接证据，保持UNKNOWN。

[heartbeat](../results/j14/j14_a1_7_r1_static_no_trace/host_heartbeat_stalls.csv) · [queue汇总](../results/j14/j14_a1_7_r1_static_no_trace/pre50_queue_summary.json)

## Runtime与instrumentation边界

原 `ros2 launch pus6_bringup pus6_system.launch.py`，J13配置，active static Servo，50 Hz dedicated_thread、async_latest、age_based、runtime diagnostics开启，A1.6同次write telemetry开启，HostHeartbeat开启。继承A1.6的`PUS6_J14_RECOVERY_STAGE=fresh_state_only`，未启用static_resume。没有strace prefix；未运行ptrace/perf/bpf/gdb、syscall wrapper或PySerial monkey patch，没有高频新log或per-write文件I/O。

TX30 ms / 正常RX5 ms / drain4 ms / 80-step recovery reject / warning283.3 ms / hard stop1000 ms / 协议安全1500 mA均不变。配置中原有`max_current_mA:500`也未改；1500 mA指现有status validator的`MAX_CURRENT_MA`，不是本阶段把配置参数改成1500。没有修改pause、fresh、re-anchor、RX、scheduler、Executor/QoS、priority、affinity、控制器、IK/FK或shaper。

| 对照 | 保留成功serial.write median | 条件差异 |
| --- | --- | --- |
| A1.5 | 34.005 µs | NO STRACE；无A1.6新增write process CPU/CS字段；此前两次timeout |
| A1.6 | 181.100 µs | strace/ptrace，新增application计数器；30 s无timeout |
| A1.7 | **29.463 µs** | NO STRACE；复用A1.6 application计数器；自然timeout在R1重现 |

这些不是随机受控A/B。A1.7说明问题在低侵入观察下仍存在；分布回到几十µs写簇，与tracing会改变时序相容，不证明strace为何抑制故障，也不证明完全没有instrumentation影响。问题不能升级为已修复。

[runtime config副本](../results/j14/j14_a1_7_r1_static_no_trace/runtime_config.yaml) · [启动前记录](../results/j14/j14_a1_7_r1_static_no_trace/run_metadata_start.json)

## 软件与Git

**Production source modification = ZERO。** 只新增A1.7 harness、offline analyzer及各自测试，不重构或修复production。既有tracked diff与进入阶段前逐字节一致；原launch安装内容与当前源码一致，driver和adapter实际build import指向当前source。

最小验证：既有A1.6 telemetry **6 PASS**、harness **10 PASS**（首个持久化failure后一次SIGINT、无trace入口、R2条件）、offline analyzer **3 PASS**（实际interval overlap、50-cycle join、epoch整数精度）；relevant py_compile、git diff --check均PASS。软件 **hardware_write_count=0**。没有重复全套pytest或colcon build来替代硬件证据。

测试初次的fake Popen影响git check_output已在测试fixture修正；离线配对初次发现epoch ns经float转换丢精度，改为整数比较并补回归测试。两者只涉及本阶段新脚本/测试，失败证据保留，production source不变。

当前branch **main**，HEAD **feba8fd00c392ecaab93b53138db7c1d06bce000**。已有10个tracked修改及全部J14工作保留。没有reset、checkout、staging、commit、push、tag、production fix；未改写历史raw数据。

[软件验证目录](../results/j14/j14_a1_7_software/) · [源码范围核查](../results/j14/j14_a1_7_software/source_scope_final.json) · [Git status](../results/j14/j14_a1_7_software/git_status_final.txt)

A1.7已停止。捕获目标达成，根因支持方向加强；GIL和syscall内部精确归因尚未证实，transport/runtime资格结论保持不变。
