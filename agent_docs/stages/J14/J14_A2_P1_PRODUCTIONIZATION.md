# J14-A2-P1 Three-Process Architecture Productionization

J14-A2-P1 **PASS；硬件已 STOP**。软件回归、正式 production 入口 30 s / 60 s Static Servo 均通过。此结论限定为 architecture productionization 与短时静态回归，尚未完成 long-duration qualification，也不证明 LADRC ACTIVE、轨迹或扰动性能。

原始证据：[P1-R1](../results/j14/j14_a2_p1_r1_static_production/)、[P1-R2](../results/j14/j14_a2_p1_r2_static_production/)、[软件与审计](../results/j14/j14_a2_p1_software/)。[操作说明](J14_PRODUCTION_DRIVER.md)给出入口与回退；[设计审计](J14_A2_P1_DESIGN_AUDIT.md)在代码修改前输出。[本阶段独立 diff](../results/j14/j14_a2_p1_software/p1_only.patch)以阶段开始快照为基准，区别于含既有工作的大范围 git diff。

## 1. Productionization 前架构

A1.13 已验证 Driver Parent + UART Child + Feedback Bridge 和三方 ready 握手，但 normal main 通过 PUS6_J14_* 环境变量 opt-in，stage capture 控制窗口与诊断。Parent 基类先创建 feedback publishers 后清空引用，DDS endpoints 仍可能保留。A1 Architecture Debugging 已关闭；本阶段没有创建 A1.14 / A1.15。

## 2. Production 最终架构

正常 `motor_driver_node` 入口默认 `three_process`；driver 根据实际 ROS CLI/YAML 参数选择实现。Parent 创建、管理和 join Child/Bridge。Parent 在创建时抑制反馈 publishers；Bridge 成为三个 feedback topic 的唯一发布者。UART Child 不加载 ROS executor / DDS。没有新增架构或手工分别启动三个进程的要求。

## 3. 正式 architecture 参数

| 参数 | 默认 | 用途 |
|---|---|---|
| driver_architecture | three_process | 显式 legacy_single_process 回退 |
| enable_coordinated_handshake | true | three_process 不允许关闭 |
| driver_telemetry_mode | production | production / qualification / debug |
| driver_audit_directory | 空 | 自动创建 results/j14/production 下唯一目录 |
| servo_run_duration_s | 0.0 | 0 持续至显式停止；正值限定窗口 |
| driver_telemetry_capacity_cycles | 4096 | qualification/debug 有界证据容量，production recent ring 为 300 |

配置优先级：installed three_process.yaml → selected system/J13 YAML → 非空显式 launch arguments。这些选项用于启动配置；没有实现运行中切换架构或安全阈值。正常入口不依赖 PUS6_J14_* flags。

## 4. Launch 方式与实际运行配置

```bash
source /opt/ros/jazzy/setup.bash
source /home/lyapunov/bone_drill/bone_ws/install/setup.bash
ros2 launch pus6_bringup pus6_system.launch.py hardware_mode:=mock
```

用户要求的正式 active 入口保持 `ros2 launch pus6_bringup pus6_system.launch.py hardware_mode:=active`。此次两轮硬件实际使用该正常入口，附加未修改的 J13 YAML、audit directory、qualification telemetry 与 30/60 s window；没有传入 driver_architecture，也没有 A1 环境变量。完整 argv 位于各轮 run_metadata.json，实际 defaults/YAML/child options 位于 production_configuration.json。

R1 实际命令（R2 仅输出目录和窗口为 60.0）：

```bash
ros2 launch pus6_bringup pus6_system.launch.py hardware_mode:=active \
  config_file:=/home/lyapunov/bone_drill/bone_ws/src/h30_imu/config/j13_ladrc_params.yaml \
  driver_audit_directory:=/home/lyapunov/bone_drill/results/j14/j14_a2_p1_r1_static_production \
  servo_run_duration_s:=30.0 driver_telemetry_mode:=qualification
```

当前报告写作时硬件已停止。未启动 LADRC runner、人工扰动、Cartesian trajectory 或 IMU。

## 5. Legacy rollback 方式

```bash
ros2 launch pus6_bringup pus6_system.launch.py hardware_mode:=mock \
  driver_architecture:=legacy_single_process
```

显式选择原 Pus6MotorDriverNode；没有删除 legacy 实现。`motor_driver_legacy_debug` 单独保留旧实验环境变量入口。若要使用历史 J7/J8 scheduling/feedback policies，应显式提供对应原配置，不能把新默认 YAML 下的 legacy 实例直接视为历史 tag 的等价 runtime。read_only 使用 explicit legacy。rollback 硬件未重跑。

## 6. Parent 职责

只负责 ROS command ingress、latest vector、command sequence、lease delivery、子进程健康和启动协调。软件 Serial guard、feedback consumer guard 与实际进程边界证实 Parent 不打开 UART、不消费 feedback socket、不重复发布反馈。其继承的 telemetry compatibility thread 启动后立即返回，不再执行 Parent feedback publication。命令 authority 仍在 Parent。

## 7. UART Child 职责

唯一 motor UART owner；独立 50 Hz absolute-deadline servo；ID1→ID6 TX/RX、feedback acquisition、fault/current 校验、local pause、handshake。真实 fuser 仅识别 Child PID：R1 808798，R2 810739。Child 的 /proc task 快照均为一个 OS thread。Child 没有 ROS executor、publisher、LADRC、IK/FK 或高频复杂诊断。

## 8. Feedback Bridge 职责

唯一 feedback socket consumer；构造/发布 JointState、TimedJointFeedback、TimedJointCommand 和轻量健康统计。TimedJointCommand 是命令执行遥测，不授予 Bridge command authority。Bridge 不访问 UART、不作控制决策。真实 Bridge PID：R1 808799，R2 810740。

## 9. Startup handshake

CHILD_READY + BRIDGE_READY + PARENT_READY → SERVO_START → Cycle 0 保持；无 fixed sleep workaround。两轮记录的 SERVO_START 不早于三方 ready，Cycle 0 在其后：R1 +0.300475 ms，R2 +0.248125 ms。Cycle 0 sequence 均为 0，startup drop 均为 0。Parent lifecycle 中稍后落盘的 handshake_complete event 不用作 ready 时间。

握手失败的软件路径为不开始 servo、local pause、close 和 fault report。进程死亡/握手异常只在 Mock 测试，未人为制造实机故障。

## 10. IPC Lease = 1000 ms

`LEASE_NS = 1_000_000_000` 保持；Parent command-channel lease 的失效触发 Child 安全停止。CommandState AST 与阶段前一致，未将 lease 改为 200 ms。

## 11. ROS Target Freshness = 200 ms

`target_timeout_sec=.2`、200_000_000 ns target freshness 保持，区别于 1000 ms lease。latest target、seq 和 reconnect 行为保持；没有把 target freshness 标签解释成 lease watchdog。

## 12. 安全、通信与时序阈值

| 项目 | 保持值 |
|---|---:|
| Baud | 921600 |
| Child rate / nominal period | 50 Hz / 20 ms |
| TX timeout | 30 ms |
| RX timeout | 5 ms |
| Drain budget | 4 ms |
| Feedback age warning | 283.3 ms |
| Feedback hard stop | 1000 ms |
| IPC lease | 1000 ms |
| ROS target freshness | 200 ms |
| Reconnect jump rejection | 超过 80 steps |
| Current validator | 1500 mA |

原 frame bytes、ID1→ID6 order、pause/fault、latest command、slew 和 startup handshake 不变。protocol.py / uart_bus.py / motor.py 未修改。IPC age >100 ms、gap >100 ms、wake >30 ms 为 diagnostic watchlist，无新增 STOP gate。max_current_mA 初始化配置与 1500 mA feedback validator 是不同语义，未混写。

## 13. ROS topics / types / QoS

| Topic | Type | Local publisher QoS |
|---|---|---|
| /joint_states | sensor_msgs/msg/JointState | KEEP_LAST 10 / RELIABLE / VOLATILE |
| /pus6/joint_feedback/timed | pus6_msgs/msg/TimedJointFeedback | KEEP_LAST 5 / BEST_EFFORT / VOLATILE |
| /pus6/joint_command/timed | pus6_msgs/msg/TimedJointCommand | KEEP_LAST 5 / BEST_EFFORT / VOLATILE |

两轮实际 ROS graph 每个 topic 均只有 `pus6_feedback_bridge_node` 一个 publisher；类型、reliability、durability 与上表一致。depth/history 由源码与软件 Node publisher 测试确认；远程 Fast DDS introspection 返回 UNKNOWN / depth 0，不把该值解释成实际队列深度。无 QoS tuning。Legacy 原 JointState topic 为 /pus6/joint_states，回退时保留历史差异。

## 14. RMW 事实

实际 `rmw_fastrtps_cpp` / eProsima Fast DDS，两轮 run metadata 确认。未切换 RMW。报告不将 telemetry 名称中的 tf/tc 当作 tf/tf_static 或 tool_center 实际接口；也未宣称 GIL confirmed root cause。

## 15. Production telemetry 保留项

保留 lifecycle、timeout/fault、六轴 pause audit、Child 累计 cycles/writes/errors/extrema、bounded recent cycle 和 TX/RX long-tail evidence、Bridge 累计 receive/drop/age/gap/health。production recent ring 300；qualification/debug 按配置 capacity，有界 deques 替换不断增长的采样数组。启动/故障/pause/退出边界落盘；没有 per-cycle 同步文件 IO 或 Parent 高频 ROS diagnostics。

## 16. Qualification / debug-only telemetry 与历史脚本

qualification 保留窗口内 cycle/write/compact Bridge timing rows，debug 额外保留 message-build/publication phase rows。Bridge 原发布顺序和标量计时调用保持，未宣称移除所有计时开销或完成性能优化。当前 qualification capacity=4096 覆盖 30/60 s；未来 600 s 全窗口证据需增加到例如 32768（软件验证配置接受能力，未运行600 s）。

历史 A1.6–A1.13 results/docs/raw data 保留。脚本分类记录在 script_classification.json：offline analyses KEEP；历史 capture / benchmark LEGACY DEBUG；没有执行 ARCHIVE 移动或删除。本阶段正常路径独立于这些脚本。

## 17. 修改文件

以下清单相对于本阶段开始状态；部分 A1 文件本来就是 Git untracked。

| 文件（相对 repo） | 本阶段用途 |
|---|---|
| bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py | 正常 main 转入 formal factory；旧实验 main 具名保留 |
| bone_ws/src/pus6_motor_driver/pus6_motor_driver/production_driver.py（新增） | ROS 参数/factory、Parent ownership 与 health wiring |
| bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py | production duration/profile、bounded evidence、累计 health |
| bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_feedback_bridge.py | production validation、bounded/compact evidence、Bridge health |
| bone_ws/src/pus6_motor_driver/config/three_process.yaml（新增） | 正式 driver defaults |
| bone_ws/src/pus6_bringup/launch/pus6_system.launch.py | defaults/YAML/launch 参数 wiring |
| bone_ws/src/pus6_motor_driver/setup.py | explicit legacy debug executable |
| bone_ws/src/pus6_motor_driver/test/test_j14_a1_11_pipeline_instrumentation.py | 两处 class MagicMock 用 monkeypatch 自动还原，修复测试泄漏 |
| bone_ws/src/pus6_motor_driver/test/test_j14_a2_p1_production.py（新增） | ownership/config/handshake/fault/death/shutdown/profile 测试 |
| bone_ws/scripts/j14_a2_p1_capture.py（新增） | 正常 launch 的 P1 capture 与 R1→R2 admission |
| bone_ws/scripts/test_j14_a2_p1_capture.py（新增） | capture argv / admission 测试 |
| bone_ws/scripts/j14_a2_p1_analyze.py（新增） | 离线分布、序列、基线对比 |
| docs/J14_A2_P1_DESIGN_AUDIT.md（新增） | 修改前最小变化设计 |
| docs/J14_PRODUCTION_DRIVER.md（新增） | 用户操作参考 |
| docs/J14_A2_P1_PRODUCTIONIZATION.md（新增） | 本报告 |

## 18. Production diff audit

[最终源码审计](../results/j14/j14_a2_p1_software/source_audit_final.json)确认：legacy class AST、socket_pair / CommandState / dispatch_slew AST 不变；j14_process_driver.py、两 subprocess main、J13 YAML 与 stage-start bytes 一致。controller、shaper、IK、FK、geometry、motor protocol、安全阈值没有本阶段改动。已有 control/J13/adapter/transport/recorder 等 tracked diff 与 stage-start diff 逐文件一致。

七个 runtime source/config 文件与软件通过后、硬件之前的快照逐字节一致；R1/R2 后未修 production。socket 配置请求8192、内核返回16384，与 A1.13 相同；无 buffer tuning。无 affinity / SCHED_FIFO / GIL 参数改动。实现属于 parameter/launch wiring、三进程 formalization 与 telemetry 收敛。

## 19. Software tests

最终四包相关回归与 capture tests **770 passed (75.37 s)**；阶段针对性测试 **24 passed (4.71 s)**，两者有重叠，不能相加。软件 `hardware_write_count=0`。J7/J8/J13 相关既有回归通过。

| 要求 | 实际软件验证 |
|---|---|
| A–F / X | 默认 factory/no env、legacy exact class、Serial guard、Parent consumer guard、actual graph sole Bridge publishers、capture 参数 |
| G–I | ready barrier / Cycle0 order、原绝对 deadline scheduler 与 PERIOD_NS20ms |
| J–Q | latest replacement、timeout abort、lease1000/target200/TX30/RX5/drain4/reconnect80/current1500；既有80/81-step边界回归 |
| R–S | ID timeout 后不继续 partial batch、local pause；motor fault/overcurrent local pause |
| T | 实际 kill Mock command-channel Parent，Child parent_ipc_closed 后 pause/close |
| U | kill Mock Bridge，Child feedback_ipc_closed 后 pause/close，health失效 |
| V | kill Mock Child，Parent stopped、Bridge EOF/health失效；已杀死 Child 本身无法执行 pause |
| W | 正常 Mock close 顺序 pause→serial close→child exit，两个 subprocess exit0 |
| telemetry | bounded containers；三个profile；watchlist 超限不作为 STOP |

初始失败记录全部保留：构建前新模块尚未安装；首次 colcon 新 YAML symlink destination 缺失；全套测试暴露 A1.11 class Mock 泄漏与 graph discovery 测试时序，修复测试隔离/等待后通过。没有以删除断言或忽略失败制造 PASS。

## 20. Syntax / build

py_compile、git diff --check PASS。最终 colcon build **5 packages finished [3.93s]**。首次 symlink-data 安装新 YAML 的局部生成目标异常，仅补齐该 install destination 后重建通过；未安装/升级系统、ROS 或依赖。完整 build 与初始失败日志保留。

## 21. P1-R1 30 s Static Servo

正式默认 three_process + qualification profile，1500 Child cycles /1500 Bridge cycles；9000 motion 0x29 writes；0 TX timeout、0 hardware fault、0 startup/steady/missing/duplicate drop、6/6 pause ACK；实际 command argv/进程/serial ownership/graph/握手均有证据。窗口正常结束 window_complete。R1 PASS 后才启动 R2。

## 22. P1-R2 60 s Static Servo

相同正式入口，3000/3000 cycles；18000 motion writes；0 timeout/fault/drop/unrecoverable IPC failure，6/6 pause ACK。软件/源代码与 R1 一致。累计 4500 cycles、27000 motion writes、90 s servo windows；启动与退出不包含在该90 s中。R2 PASS，完成后 STOP。

## 23. TX timeout count 与 TX distribution

R1=0，R2=0。以下 serial_write 来自实际 Serial.write wall 字段，不把 wrapper wall 当成串口写耗时。

| Run | N | P50 (ms) | P95 (ms) | P99 (ms) | Max (ms) |
|---|---:|---:|---:|---:|---:|
| P1-R1 | 9000 | 0.033904 | 0.079407 | 0.101108 | 0.230416 |
| P1-R2 | 18000 | 0.034276 | 0.080260 | 0.100253 | 0.467778 |

Wrapper wall（含软件 wrapper 开销）：

| Run | N | P50 (ms) | P95 (ms) | P99 (ms) | Max (ms) |
|---|---:|---:|---:|---:|---:|
| P1-R1 | 9000 | 0.079521 | 0.178613 | 0.257143 | 6.392623 |
| P1-R2 | 18000 | 0.079789 | 0.180970 | 0.268527 | 23.256763 |

R2 wrapper Max23.256763 ms 高于 A1.13-R2 wrapper Max2.967 ms；同时 serial Max0.467778 ms、0timeout/fault/drop，gap/wake tails 未超过历史 maxima。该离散 wrapper tail 保留为后续观察项，未归因 GIL，也未把它重新解释成 TX timeout。P1 未进行性能优化。

## 24. Packet drop

两轮 child/bridge sequence 集合完整：first0，1500/3000 unique sequences；startup、steady、missing、duplicate 均0。Bridge累计 drop0。这是 Child→Bridge feedback packet 完整性；没有将 ROS observer endpoint 图当成每个 downstream subscriber 的 packet-loss 测量。

## 25. Feedback IPC Age

| Run | N | P50 (ms) | P95 (ms) | P99 (ms) | Max (ms) |
|---|---:|---:|---:|---:|---:|
| P1-R1 | 1500 | 0.457231 | 0.623186 | 0.756586 | 1.995654 |
| P1-R2 | 3000 | 0.460719 | 0.628925 | 0.749693 | 1.864654 |

Cycle0 age：R1 0.312065 ms；R2 1.679805 ms。A1.13-R2 P50/P99/Max=0.545/1.154/32.828 ms，Cycle0 .244727 ms；P1未丢失Cycle0。以上是 Child packet 到 Bridge 的 IPC age，不代表下游控制闭环总延迟。

## 26. Feedback Gap

| Run | N | P50 (ms) | P95 (ms) | P99 (ms) | Max (ms) |
|---|---:|---:|---:|---:|---:|
| P1-R1 | 1499 | 20.002913 | 20.699705 | 21.301772 | 26.632080 |
| P1-R2 | 2999 | 20.002181 | 20.721560 | 21.175616 | 43.549389 |

A1.13-R2 Max52.404 ms；P1-R2 Max43.549389 ms，无持续恶化/drop/timeout。IPC age/gap >100 ms watchlist counts 两轮均0。

## 27. Child Wake Lateness

| Run | N | P50 (ms) | P95 (ms) | P99 (ms) | Max (ms) |
|---|---:|---:|---:|---:|---:|
| P1-R1 | 1500 | 0.226540 | 0.295880 | 0.329790 | 0.429925 |
| P1-R2 | 3000 | 0.140158 | 0.260462 | 0.286580 | 10.818032 |

A1.13-R2 Max19.714 ms；P1-R2 Max10.818032 ms。>30 ms watchlist counts 两轮0。50 Hz 是配置；1500/30 s、3000/60 s 和真实 wake/interval 字段支持本阶段有效短时50Hz，不证明所有 cycle 完全无抖动。

## 28. Pause ACK

R1/R2 stop_pause_audit.jsonl 各有六轴 response_received=true、timeout=false，success=true。Child final pause_success=true。每轮窗口结束一次六轴 pause，没有 blind motion retry。真实故障触发 pause 未在本阶段人为注入；由 Mock regression 验证。

## 29. Cleanup 与保留异常

两轮 UART Child / Bridge exit0；Parent lifecycle 确认 joins 与 destroy。最终再次读取 /proc / fuser：0 relevant residual process，/dev/ttyUSB0、/dev/ttyACM0、/dev/ttyACM1 三个串口全部 free。证据：[resources_final.json](../results/j14/j14_a2_p1_software/resources_final.json)。

两轮 robot.log **仍有 control/kinematics 的既有 `rcl_shutdown already called` traceback / process exit1**，发生在 Child pause/serial close 清理之后。没有把 launch exit0 单独当作干净退出证明，也不宣称完整 ROS 系统无 shutdown 异常。P1驱动/硬件 criteria通过；该上游清理限制如实保留，没有在 R2 后继续修代码。

Bridge EOF 后 bridge_stopped 记录 `health_ok=false`、exit_reason=child_feedback_eof，表示退出后不再提供活反馈；两轮是在 Child window_complete / pause / close 后正常结束，而非 active-period IPC failure。run metadata 的 R2 `r2_admissible=false` 是 R1-specific admission函数要求1500cycles造成，不是 R2 verdict；R2最终 verdict位于p1_analysis.json。

## 30. Legacy rollback test

原 class AST 未改变；software factory返回原class、没有Child/Bridge；正常 launch 显式 legacy_single_process 的2 s Mock观察完成，hardware_write_count0、进程和串口释放。默认 three_process Mock launch另外100/100cycles、0drop、handshake与唯一publisher通过。Legacy Mock同样保留上游 shutdown exception；没有将原硬件J7/J8执行能力重新标为本阶段硬件verified。

## 31. Frozen baseline 污染检查

所有tag refs与阶段开始一致，包括 `v1.0-j13-ladrc-attitude-residual-control-qualified`。J7/J8/J9/J11/J12/J13 historical assets/tag 未重写，历史A1证据未删除。当前working tree原本已含J13/J14未提交修改；本阶段并不声称当前runtime与历史tag逐字节相同。本阶段runtime文件确与软件通过后的快照一致，源文件/阈值保护与真实30/60s证据共同支持新J14路径。

## 32. 是否允许进入 120/300/600 s Qualification

按本阶段判据，software PASS + P1-R1 PASS + P1-R2 PASS，**允许作为下一阶段 J14-A2-Q1 的输入**。尚未运行120/300/600 s、未qualified long-duration。配置支持更长窗口不等同授权自动启动；本任务结束STOP。

## 33. Git status

branch main；HEAD `feba8fd00c392ecaab93b53138db7c1d06bce000` 保持。开始10个tracked dirty，结束12个（加入本阶段 launch/setup 修改；motor_driver已dirty，追加入口wiring）。既有未提交/untracked资产保留；stage-only changed_files_final.json 明确6个修改、9个新增，与 Git untracked 状态区别。

完整status与diff保存在 git_status_final.txt、git_diff_final.patch；stage-only diff单独为 p1_only.patch。没有 reset / checkout / stage / commit / push / tag。

## 34. 唯一下一建议

进入 **J14-A2-Q1 Long-Duration Static Qualification**，另行授权后按120 s →300 s →600 s顺序执行；沿用本生产入口和冻结安全语义，增加全窗口证据容量并继续观察IPC/gap/wake与wrapper tail。当前保持STOP，不运行长时、LADRC或扰动。
