# J14 Final Summary

**阶段状态：** CLOSED — J14-A Runtime Reliability、Active Stabilization Demo 与第二轮 representative comparison 已完成收口。

**整理日期：** 2026-10-11。**本次工作：** 文档整理；`hardware_write_count = 0`。

本文件负责总结、证据索引和阶段闭环，不替代设计报告、原始日志或详细分析。设计过程保留在 [J14 阶段目录](/home/lyapunov/bone_drill/agent_docs/stages/J14)，实验记录保留在 [J14 results](/home/lyapunov/bone_drill/results/j14)。此路径原有的 J14-A 总结仍可从冻结提交 `ce48ee9` 的同一路径查阅。

## 1. Overview

J14 的重点是把已有控制算法验证推进到机器人系统工程可靠性和可操作的主动稳定 Demo：建立可追溯的 runtime qualification，完成从启动、reference capture、ACTIVE 到受控退出的生命周期，再利用双 IMU 保存并分析真实硬件中的输入扰动与平台残余姿态。

- **Runtime Reliability：** Parent / UART Child / Feedback Bridge 三进程分工，处理已观察到的 UART blocking、反馈连续性、Parent executor starvation、启动协调与受控停止问题。
- **Active Stabilization Demo：** 统一 Base SO(3) geometric feedforward，支持纯前馈、Classical P/PD 和 First-order LADRC residual compensation 三种模式。
- **Controller Comparison：** 第二轮真实硬件记录完成 Base 输入统计、Platform 姿态残余和归一化指标分析，结论限定为 representative comparison。

版本边界分别为：runtime 冻结提交 `ce48ee9`，Demo / 文档收口提交 `5c2b3d1`，本次整理前 HEAD `ba14b79`。当前工作树还有 telemetry recording wrapper 修改、离线分析脚本和本地实验结果；这些未提交内容不属于冻结 tag。实验 metadata 中的 `git_head` 也不能单独证明工作树干净、已 source workspace 或实际运行路径与 tag 完全一致。

## 2. Runtime Reliability Qualification

### Architecture and resolved issues

| Process | 职责与边界 |
| --- | --- |
| Parent | 接收 ROS 目标，以 latest-target 语义交付命令，维护 heartbeat/lease 并协调子进程；production 使用 `SingleThreadedExecutor`，不直接访问电机 UART。 |
| UART Child | 电机 UART 唯一所有者；执行配置为 50 Hz 的控制/通信循环、反馈采集、安全检查和六轴 Pause；lease 失效可在 Child 本地停止。 |
| Feedback Bridge | Child 反馈 IPC 唯一消费者，向 ROS 发布反馈；不访问 UART，不产生运动命令。 |

兼容 Relay 转换既有反馈接口，Base / Platform IMU 由独立传感器进程提供。配置 50 Hz 不等于每个周期均严格满足 20 ms，三进程架构也不构成硬实时保证。

- **UART blocking：** bounded-write 与通信/诊断路径隔离处理已观察到的阻塞和长尾；写入结果不确定时不盲目重复运动写入。
- **Feedback continuity：** Bridge 从 Child 独立接收反馈，避免反馈转发依赖 Parent 命令回调。
- **Executor starvation：** 修复已观察到的 Parent executor / IPC starvation，后续 ACTIVE 会话提供验证；GIL、DDS 与 OS 调度的全部底层贡献尚未独立确认。
- **Startup coordination：** Child / Bridge readiness 与 Parent 的 `SERVO_START` handshake 协调启动。
- **Controlled shutdown：** 保留本地停止、六轴 Pause、反馈确认、串口释放与进程清理的证据。

设计与实现依据见 [productionization](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A2_P1_PRODUCTIONIZATION.md)、[diagnostic isolation](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A2_Q1F_TX_QUEUE_DIAGNOSTICS_ISOLATION.md) 和 [Parent starvation fix](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A2_Q3J_PARENT_ACTIVE_COMMAND_STARVATION_FIX.md)。

### Qualification evidence

| 验证 | 已建立的范围 | 证据 |
| --- | --- | --- |
| Q2-R3 static 600 s qualification | Child / Bridge 各 29,995 个连续 cycle；记录中无 IPC 缺序或重复、无 TX timeout 或 hardware fault；Pause ACK、串口释放及无残留进程已记录。 | [metadata](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/run_metadata.json)、[详细分析](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/q2_detailed_analysis.json) |
| Q3-R1D Classical P/PD active runtime verification | 已保存 analysis 判定 PASS，并记录算法执行、实际执行链路与物理响应；名义会话为 60 s，含非 ACTIVE 阶段。 | [Classical analysis](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/q3_analysis.json) |
| Q3-R2 First-order LADRC active runtime verification | 已保存 analysis 判定 PASS，并记录三轴残差控制执行、实际执行链路与物理响应；名义会话为 60 s，含非 ACTIVE 阶段。 | [LADRC analysis](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/q3_analysis.json) |
| 三模式 Demo 正常退出 | ACTIVE、HOME_CONFIRMED、六轴 Pause ACK、serial release 与无残留进程均已记录；具体 final record 见第 6 节。 | [Demo 生命周期报告](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_DEMO_F1_HOME_SHUTDOWN.md) |

Q2-R3 保留了 **3 次 deadline reset**，metadata 还记录 `shutdown_exception=true`。其 Pause 与资源释放结果并未因此被删去，但不能将该会话描述为零抖动或无退出异常。后续 Demo 的正常退出 PASS 是对应会话的独立证据，不抹去历史异常。

**自动 recovery / resume 未 qualified。** 已有恢复相关软件验证不等于当前三进程 production 已完成自动通信恢复、重新锚定和自动恢复运动的硬件资格验证。

## 3. Active Stabilization Demo

三个模式共享同一控制关系：**Base geometric compensation + different residual controllers**。Platform IMU 提供平台姿态观测；带残差控制的两种模式使用其反馈形成姿态闭环。

### 3.1 Base SO(3) Feedforward

使用 Base IMU 相对捕获参考的姿态，通过 SO(3) 逆旋转进行几何补偿，再经过既有 Cartesian → IK → trajectory shaper → Parent → Child 执行链路。此模式没有 Platform residual controller，Platform IMU 用于观测效果。

入口：[run_j14_demo_feedforward.sh](/home/lyapunov/bone_drill/bone_ws/scripts/run_j14_demo_feedforward.sh)。

### 3.2 Feedforward + Classical P/PD

在 Base feedforward 上叠加 Platform 姿态残差反馈。当前配置实际为 **Classical P/PD，Ki = 0**：Roll 使用 P，Pitch / Yaw 使用 PD。实验目录与历史接口保留 `pid` 命名，这不表示本次启用了积分控制。

入口：[run_j14_demo_feedforward_pd.sh](/home/lyapunov/bone_drill/bone_ws/scripts/run_j14_demo_feedforward_pd.sh)。

### 3.3 Feedforward + First-order LADRC

在同一 Base feedforward 上叠加三轴 **First-order LADRC residual compensation**，使用观测器进行残差状态与扰动估计。LADRC 处理残差补偿，不替代 Base geometric feedforward。

入口：[run_j14_demo_feedforward_ladrc.sh](/home/lyapunov/bone_drill/bone_ws/scripts/run_j14_demo_feedforward_ladrc.sh)。实际选用的残差记录为 `u_combined_actual_rad/deg`；`pid_total_residual_rad/deg` 是并行 Classical 残差，不能解释为 ADRC 输出。

共同生命周期为 runtime / feedback ready → startup `OPERATING_HOME` 反馈确认 → 双 IMU ready → stationary reference capture / preflight → ACTIVE。无故障 ACTIVE 中正常操作员退出会停止姿态输出，沿既有执行链路受控返回 HOME，反馈确认后执行六轴 Pause 和资源清理。故障退出跳过 HOME，执行 Pause / 清理；Demo 不自动进入 `STORAGE_PARK`。这不保证任意初始位置均能进入 ACTIVE。

操作说明见 [J14 Demo Entrypoints](/home/lyapunov/bone_drill/docs/j14/J14_DEMO_ENTRYPOINTS.md)，生命周期依据见 [HOME shutdown report](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_DEMO_F1_HOME_SHUTDOWN.md)。

## 4. PID / ADRC Representative Comparison

### Inputs and analysis window

真实硬件输入为 [pid/run_002](/home/lyapunov/bone_drill/results/j14/controller_compare/pid/run_002) 与 [adrc/run_002](/home/lyapunov/bone_drill/results/j14/controller_compare/adrc/run_002)，分别保存 `telemetry.csv`、`metadata.json` 和会话结束记录。PID 指 Feedforward + Classical P/PD（Ki = 0），ADRC 指 Feedforward + First-order LADRC residual compensation。

PID 完整 CSV 为 1,359 行，其中 ACTIVE 1,215 行；ADRC 完整 CSV 为 1,215 行，其中 ACTIVE 1,163 行。已有完整性检查记录必需字段无缺失/非有限值，时间戳递增，`cycle_index` 连续；这仅证明保存的控制 tick 快照连续，不证明原始 IMU 消息无丢包。

主要比较沿用既有 **ACTIVE common window：0–23.239906447 s**，各自首个 ACTIVE tick 为 t = 0，两组各取 **1,163 个原生样本**，不重新对齐扰动、不插值。startup HOME、reference capture 和 shutdown HOME 不参与指标。较长 PID 的 52 行尾段保留在完整曲线和补充统计中，不进入主要指标。

`timestamp_ns` 是主机 `time.perf_counter_ns` 控制 tick 时间，不是传感器采样时刻或 UTC。RPY 均相对各次捕获参考；两次参考不同，不据此建立外部绝对姿态精度。详细窗口与数据检查见 [Round 2 summary](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/summary.md)。

### Base disturbance statistics

字段为 `base_rel_roll_deg`、`base_rel_pitch_deg`、`base_rel_yaw_deg`。以下单位均为 deg；range = max − min，peak absolute = max(abs(x))。

| Axis | PID RMS | ADRC RMS | PID range | ADRC range | PID peak absolute | ADRC peak absolute |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Roll | 10.757911 | 13.314377 | 17.909294 | 19.139041 | 15.207097 | 17.661705 |
| Pitch | 8.098322 | 7.115071 | 18.878725 | 16.943024 | 17.489456 | 16.054062 |
| Yaw | 2.058904 | 3.287575 | 7.229765 | 10.425585 | 4.109166 | 5.567742 |

Base Roll / Pitch / Yaw RMS 的 ADRC 相对 PID 变化分别为 **+23.76%、−12.14%、+59.68%**。人工输入在幅度、波形与时序上仍不一致，尤其 Roll / Yaw，不能称为严格相同扰动。

### Platform attitude residual

字段为 `platform_rel_roll_deg`、`platform_rel_pitch_deg`、`platform_rel_yaw_deg`，表示平台相对捕获零参考的欧拉角残余，不是 SO(3) 误差向量分量。以下单位均为 deg。

| Axis | PID RMS | ADRC RMS | PID max absolute | ADRC max absolute |
| --- | ---: | ---: | ---: | ---: |
| Roll | 0.981966 | 1.601807 | 3.810738 | 8.441471 |
| Pitch | 1.591881 | 1.734782 | 5.371234 | 4.980688 |
| Yaw | 1.670401 | 3.689602 | 7.347981 | 13.201495 |

表格描述这两次会话的测量结果，不隔离算法效应与输入差异。误差 min/max、完整 ACTIVE 补充统计和原始异常峰值保留在 [summary.csv](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/summary.csv) 与详细报告中。

### Normalized disturbance rejection ratio

轴向描述指标：`rho_axis = RMS(platform_rel_axis_deg) / RMS(base_rel_axis_deg)`，即 **Platform residual / Base disturbance**，无量纲。RMS 使用原始样本的 `sqrt(mean(x^2))`，不去均值；该比值是两个 RMS 的比，不是逐样本比值的平均。

| Axis | PID / Classical P/PD | ADRC / First-order LADRC |
| --- | ---: | ---: |
| Roll | 0.091279 | 0.120307 |
| Pitch | 0.196569 | 0.243818 |
| Yaw | 0.811306 | 1.122287 |

已有 `platform_so3_error_deg` 字段的归一化指数分别为 PID **0.184177**、ADRC **0.283289**。分子直接使用该 SO(3) 旋转角的 RMS；分母使用逐样本 `sqrt(roll^2 + pitch^2 + yaw^2)` 的 Base 欧拉角幅值 RMS，没有补造 Platform SO(3) 数据，也没有将分母替换成 Base SO(3) 旋转角。这是描述性指数，不是 SO(3) 到 SO(3) 的传递增益。

归一化只能描述本窗口中单位输入扰动 RMS 对应的残余姿态 RMS，没有消除频率、时序、跨轴耦合、参考、瞬态、非线性或限幅差异。ADRC Yaw 的比值大于 1 也不足以单独证明扰动放大或不稳定。**这些结果仍是 normalized representative indicator，不能替代严格相同扰动输入下的 controller benchmark，也不支持 PID 或 ADRC superiority 的结论。**

指标定义、未舍入数值及独立复算记录见 [normalized summary](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/normalized_disturbance_summary.md) 和 [normalized CSV](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/normalized_disturbance_summary.csv)。本次从原始 CSV 独立复算核对摘要数值；图表均来自已有离线分析，没有重新生成或修改分析产物。

## 5. Validation Scope and Limitations

已完成的范围包括：双 IMU 姿态观测及残差反馈闭环、六轴执行链路、三种主动稳定模式、指定会话中的 runtime qualification / controlled shutdown，以及第二轮真实硬件 representative comparison。纯 Feedforward 模式仅观测 Platform；上述闭环结论对应带 residual feedback 的模式。

资格与性能结论限定于保存的配置、运行窗口和硬件条件：

- **人工扰动重复性有限：** Base 输入不能完全复现，统计归一化也不构成严格性能排名。
- **全工作空间绝对精度未专项建立：** IMU 相对姿态保持和局部控制反馈不等于外部参考下的绝对精度；全工作空间精度与重复性也未完成表征。
- **自动恢复 / resume 未 qualified：** 不宣称全自动故障恢复控制。
- **理论稳定性未证明：** 硬件会话成功、观测器有界或有限窗口误差不是理论稳定性证明。
- **可靠性不等于零异常：** 保留静态调度长尾、shutdown exception、Base IMU USB 断连及启动层安全拒绝记录；后续 PASS 不证明这些故障类别已彻底消除。USB 事件的物理 / 电气触发因素仍未确认。
- **记录覆盖有边界：** telemetry 保存已发出的控制 tick 快照；fault tick 可能在 append 前返回，完整 IMU receive/header 序列并未全部记录。连续 CSV 或 `safe_shutdown=true` 不能替代原始故障与退出日志。

冻结 tag、当前工作树、已 source 的 ROS workspace 和实际执行路径应分别确认。本次只更新总结文档，不修改冻结代码、控制参数、原始实验数据或既有分析输出，不运行真机、不提交 Git。

## 6. Evidence Index

### Code and Demo entrypoints

| 类别 | 入口 |
| --- | --- |
| Production Parent | [production_driver.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/production_driver.py)、[j14_process_driver.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_driver.py) |
| UART Child / IPC | [j14_process_uart.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_process_uart.py)、[j14_uart_child_main.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_uart_child_main.py) |
| Feedback Bridge | [j14_feedback_bridge.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_feedback_bridge.py)、[bridge main](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j14_feedback_bridge_main.py)、[compatibility relay](/home/lyapunov/bone_drill/bone_ws/scripts/j14_a2_q3_feedback_relay.py) |
| Base feedforward / residual algorithms | [j11_base_feedforward_runner.py](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_base_feedforward_runner.py)、[j11_m2_pid_residual_runner.py](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j11_m2_pid_residual_runner.py)、[j13_ladrc_shadow_runner.py](/home/lyapunov/bone_drill/bone_ws/src/h30_imu/scripts/j13_ladrc_shadow_runner.py) |
| Demo lifecycle / recording wrappers | [j14_demo_launcher.py](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_launcher.py)、[j14_demo_controller.py](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_controller.py)、[j14_demo_home.py](/home/lyapunov/bone_drill/bone_ws/scripts/j14_demo_home.py) |
| Demo commands | [Feedforward](/home/lyapunov/bone_drill/bone_ws/scripts/run_j14_demo_feedforward.sh)、[Classical P/PD](/home/lyapunov/bone_drill/bone_ws/scripts/run_j14_demo_feedforward_pd.sh)、[First-order LADRC](/home/lyapunov/bone_drill/bone_ws/scripts/run_j14_demo_feedforward_ladrc.sh) |
| Offline analysis | [analyze_pid_adrc_compare.py](/home/lyapunov/bone_drill/bone_ws/scripts/analyze_pid_adrc_compare.py)；归一化补充结果以已保存的报告与 CSV 为索引，不宣称该脚本已集成全部补充分析。 |

### Design, qualification and handoff documents

- Runtime 操作与接口：[J14_PRODUCTION_DRIVER.md](/home/lyapunov/bone_drill/docs/j14/J14_PRODUCTION_DRIVER.md)。
- 架构与实现：[P1 design audit](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A2_P1_DESIGN_AUDIT.md)、[P1 productionization](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A2_P1_PRODUCTIONIZATION.md)、[three-process handshake](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A1_13_THREE_PROCESS_COORDINATED_HANDSHAKE.md)。
- 通信定位：[bounded UART write](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A1_UART_TX_BOUNDED_WRITE.md)、[UART workload isolation](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A1_8_SERVO_UART_WORKLOAD_ISOLATION.md)、[Q1F diagnostic isolation](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A2_Q1F_TX_QUEUE_DIAGNOSTICS_ISOLATION.md)。
- ACTIVE 路径：[Q3J Parent starvation fix](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A2_Q3J_PARENT_ACTIVE_COMMAND_STARVATION_FIX.md)。
- Demo 设计与使用：[source audit](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A_DEMO_SOURCE_AUDIT.md)、[F1 design audit](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A_DEMO_F1_DESIGN_AUDIT.md)、[HOME shutdown](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_DEMO_F1_HOME_SHUTDOWN.md)、[Demo Entrypoints](/home/lyapunov/bone_drill/docs/j14/J14_DEMO_ENTRYPOINTS.md)。
- 阶段交接：[J14_FINAL_HANDOFF.md](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_FINAL_HANDOFF.md)、[J14_TO_NEXT_STAGE_HANDOFF.md](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md)。这些文件保留其历史版本边界；本文件不改写其交接时 HEAD 或后续计划。
- 残差控制前序依据：[J13_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/stages/J13/J13_FINAL_SUMMARY.md)、[J13_TO_J14_HANDOFF.md](/home/lyapunov/bone_drill/agent_docs/stages/J13/J13_TO_J14_HANDOFF.md)。

### Results and figures

| Evidence | 路径与用途 |
| --- | --- |
| Static 600 s qualification | [Q2-R3 directory](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s)，包含 metadata、Child cycles、Bridge pipeline、Pause audit 与详细分析。 |
| Active runtime verification | [Q3-R1D Classical](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s)、[Q3-R2 LADRC](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s)；会话级资格证据，不能与 Round 2 的窗口指标混用。 |
| 已归档的三模式 Demo | [Feedforward final](/home/lyapunov/bone_drill/results/j14/demo/feedforward_20261010_183023_090291/demo_final.json)、[Classical final](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261010_183135_756834/demo_final.json)、[LADRC final](/home/lyapunov/bone_drill/results/j14/demo/feedforward_ladrc_20261010_183241_244023/demo_final.json)。 |
| 本地后续三模式复验 | [Feedforward final](/home/lyapunov/bone_drill/results/j14/demo/feedforward_20261011_155457_215290/demo_final.json)、[Classical final](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_155532_657398/demo_final.json)、[LADRC final](/home/lyapunov/bone_drill/results/j14/demo/feedforward_ladrc_20261011_155611_493332/demo_final.json)。 |
| 保留的失败记录 | [Base IMU freshness STOP](/home/lyapunov/bone_drill/results/j14/demo/feedforward_ladrc_20261010_180702_906278/demo_final.json)、[startup rejection](/home/lyapunov/bone_drill/results/j14/demo/feedforward_20261011_153957_162728/demo_final.json)。 |
| Round 2 PID raw data | [telemetry.csv](/home/lyapunov/bone_drill/results/j14/controller_compare/pid/run_002/telemetry.csv)、[metadata.json](/home/lyapunov/bone_drill/results/j14/controller_compare/pid/run_002/metadata.json)。 |
| Round 2 ADRC raw data | [telemetry.csv](/home/lyapunov/bone_drill/results/j14/controller_compare/adrc/run_002/telemetry.csv)、[metadata.json](/home/lyapunov/bone_drill/results/j14/controller_compare/adrc/run_002/metadata.json)。 |
| Round 2 main analysis | [summary.md](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/summary.md)、[summary.csv](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/summary.csv)。 |
| Round 2 normalized analysis | [normalized_disturbance_summary.md](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/normalized_disturbance_summary.md)、[normalized_disturbance_summary.csv](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/normalized_disturbance_summary.csv)。 |

图表入口：

- [fig1_base_disturbance.png](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/fig1_base_disturbance.png)：两组完整 ACTIVE Base 原始曲线，标明公共窗口及 PID 尾段。
- [fig2_platform_error.png](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/fig2_platform_error.png)：两组完整 ACTIVE Platform 残余曲线，保留原始峰值。
- [fig3_error_rms_compare.png](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/fig3_error_rms_compare.png)：公共窗口 Roll / Pitch / Yaw RMS。
- [fig4_normalized_disturbance_rejection.png](/home/lyapunov/bone_drill/results/j14/controller_compare/comparison/real_run_002/fig4_normalized_disturbance_rejection.png)：轴向 normalized residual ratio。

上述后续复验、controller_compare 结果及离线脚本在本次整理前仍为本地 untracked；recording wrapper 也有未提交修改。文档链接提供本地证据入口，不表示这些内容已随 tag 归档。

### Tags and commits

| Reference | Commit | 范围 |
| --- | --- | --- |
| `v1.0-j13-ladrc-attitude-residual-control-qualified` | `89d38b3ac2bbb63234aa56f8ecd87178010dde76` | J13 LADRC 姿态残差控制资格基线。 |
| `v1.0-j14-a-three-process-runtime-reliability-qualified` | `ce48ee97f5c827a12591d43a82d32ce97fd4689d` | J14-A 三进程 runtime reliability 冻结基线。 |
| `5c2b3d1` | `5c2b3d1` | Demo 与操作文档收口。 |
| `ba14b79` | `ba14b79` | Final handoff 文档提交；本次整理前 HEAD。 |

博客素材按问题组织：runtime ownership / communication reliability 使用第 2 节与 Q2/Q3 证据；主动稳定方法及生命周期使用第 3 节与 Demo final records；实验图表使用第 4 节与 Round 2 数据；所有性能、精度和恢复表述附带第 5 节限制。不得把代表性记录改写为全工作空间精度、理论稳定性或控制器全局优越性结论。
