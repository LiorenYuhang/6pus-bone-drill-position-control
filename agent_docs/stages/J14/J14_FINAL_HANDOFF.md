# J14-A Final Handoff

**Status:** CLOSED / FROZEN — runtime reliability baseline and stabilization demo handoff.

**Handoff date:** 2026-10-11.

**Frozen tag:** `v1.0-j14-a-three-process-runtime-reliability-qualified` → `ce48ee97f5c827a12591d43a82d32ce97fd4689d`.

本文件整理已完成能力、冻结边界、证据入口和已知限制。阶段细节保留在 [J14_FINAL_SUMMARY.md](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_FINAL_SUMMARY.md)；本交接以原始记录约束结论，不替代实验日志或控制器性能报告。

## 1. Stage Overview

J14-A 处理 ROS 2 + UART 机器人系统长期运行中的可靠性问题：communication blocking、timeout、feedback age、startup coordination、executor starvation 和 controlled shutdown。

已建立 Parent / UART Child / Feedback Bridge 三进程运行基础，并完成静态资格验证、Classical P/PD 与 LADRC ACTIVE 会话及三个姿态稳定 Demo 的实机验证。资格结论适用于保存的配置、运行窗口和硬件条件；不表示硬实时保证、所有故障已消除或任意时长运行均已验证。

## 2. Completed Capabilities

### Runtime Architecture

| Process | Ownership and responsibility |
| --- | --- |
| Parent | 接收 ROS 目标，按 latest-target 语义向 Child 交付命令，负责进程协调与 heartbeat/lease 更新；production Parent 使用 `SingleThreadedExecutor`，不直接访问电机 UART。 |
| UART Child | 电机 UART 的唯一所有者，执行 50 Hz 通信与控制写入、反馈采集、安全检查和六轴 Pause；Parent lease 失效时可在 Child 本地停止。 |
| Feedback Bridge | Child 反馈 IPC 的唯一消费者，将反馈发布到 ROS；不访问 UART、不产生运动命令。反馈流与 Parent 命令订阅解耦。 |

兼容 Relay 仅转换既有反馈话题，帮助冻结的 J11/J13 controller 与当前 runtime 对接；不增加 UART 或运动命令所有者。Base / Platform IMU 由各自独立进程和串口提供数据。

实现与接口入口：[production driver 操作说明](/home/lyapunov/bone_drill/docs/j14/J14_PRODUCTION_DRIVER.md)、[production_driver.py](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/production_driver.py)。

### Communication Reliability

- 完成 UART blocking / TX-RX 长尾定位与 bounded-write 处理；不对 delivery uncertain 的运动写入进行 blind retry。
- 分离通信关键路径与诊断路径，修复已观察到的诊断处理长尾；production 保留有界近期记录及累计健康信息，qualification 保留更完整的时序证据。
- 完成 Child / Bridge / Parent readiness 到 `SERVO_START` 的协调 handshake，保障反馈链路启动次序。
- 修复已观察到的 Parent executor / IPC starvation 问题，并通过后续 ACTIVE 软件与硬件会话验证。底层 GIL、DDS 和 OS 调度的所有细分机制并未全部独立确认。
- 保留 feedback age、lease、安全边界及故障停止语义。通信恢复软件曾有专项验证，但当前三进程 production 的自动 recovery / resume 未 qualified，不能用软件测试替代完整硬件恢复证明。

诊断与修复依据：[Q1F diagnostic isolation](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A2_Q1F_TX_QUEUE_DIAGNOSTICS_ISOLATION.md)、[Q3J Parent runtime fix](/home/lyapunov/bone_drill/agent_docs/stages/J14/J14_A2_Q3J_PARENT_ACTIVE_COMMAND_STARVATION_FIX.md)。

### Stabilization Demo

三个入口共享 Base attitude 的 SO(3) 逆旋转前馈；差别在 residual controller。

| Demo | Entrypoint | Residual control |
| --- | --- | --- |
| Base SO(3) Feedforward | [run_j14_demo_feedforward.sh](/home/lyapunov/bone_drill/bone_ws/scripts/run_j14_demo_feedforward.sh) | 无残差控制；Platform IMU 用于观测。 |
| Feedforward + Classical P/PD | [run_j14_demo_feedforward_pd.sh](/home/lyapunov/bone_drill/bone_ws/scripts/run_j14_demo_feedforward_pd.sh) | Platform 姿态误差的 Classical P/PD 残差补偿。 |
| Feedforward + First-order LADRC | [run_j14_demo_feedforward_ladrc.sh](/home/lyapunov/bone_drill/bone_ws/scripts/run_j14_demo_feedforward_ladrc.sh) | 三轴 First-order LADRC 残差补偿。 |

共同生命周期：runtime / feedback ready → 有效反馈及安全检查 → Startup `OPERATING_HOME` 确认 → Dual IMU ready → stationary reference / preflight → ACTIVE。

无故障 ACTIVE 状态下正常 `Ctrl+C`：停止姿态输出 → 受控返回 `OPERATING_HOME` → 实际反馈确认 → 六轴 Pause → 释放串口与清理进程。故障退出跳过 HOME，执行 Pause 与清理。`OPERATING_HOME` 与 `STORAGE_PARK` 保持独立，Demo 不自动 PARK；HOME 经原 Cartesian → IK → trajectory shaper → Parent → Child 路径执行。

当前操作入口：[J14 Demo 使用说明](/home/lyapunov/bone_drill/docs/j14/J14_DEMO_ENTRYPOINTS.md)。

## 3. Validation Evidence

以下记录回答不同工程问题，**不是统一 benchmark**，不能据此直接排名三个 controller 的扰动抑制性能。ACTIVE 会话的名义时长包含启动 / reference 阶段，不等于相同长度的 ACTIVE 数据窗口。

| Evidence | Established scope | Primary entry |
| --- | --- | --- |
| Q2-R3, 600 s static qualification | 静态三进程通信、反馈连续性、安全停止及资源释放的资格验证；保留发生过的调度长尾和 deadline reset，不宣称零抖动。 | [run metadata](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/run_metadata.json)、[原始 Child cycles](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/child_cycles.jsonl)、[详细检查](/home/lyapunov/bone_drill/results/j14/j14_a2_q2_r3_static_600s/q2_detailed_analysis.json) |
| Q3-R1D Classical P/PD active session | 真实硬件 ACTIVE 链路 PASS；不扩展为全工作空间精度结论。 | [analysis](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/q3_analysis.json)、[原始 controller telemetry](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r1d_classical_60s/j14_a2_q3_r1_classical_telemetry_powered.csv) |
| Q3-R2 LADRC active session | 三轴 First-order LADRC 真实硬件 ACTIVE 链路 PASS；不构成理论稳定性证明。 | [analysis](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/q3_analysis.json)、[原始 controller telemetry](/home/lyapunov/bone_drill/results/j14/j14_a2_q3_r2_ladrc_60s/j14_a2_q3_r1_ladrc_telemetry_powered.csv) |
| Feedforward Demo hardware verification | ACTIVE、正常退出 HOME、Pause 和资源清理 PASS。 | [2026-10-10 final record](/home/lyapunov/bone_drill/results/j14/demo/feedforward_20261010_183023_090291/demo_final.json) |
| Classical P/PD Demo hardware verification | 同一 Demo 生命周期的真实硬件验证 PASS。 | [2026-10-10 final record](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261010_183135_756834/demo_final.json) |
| LADRC Demo hardware verification | 同一 Demo 生命周期的真实硬件验证 PASS。 | [2026-10-10 final record](/home/lyapunov/bone_drill/results/j14/demo/feedforward_ladrc_20261010_183241_244023/demo_final.json) |

上述 Demo 记录包含 `hardware_mode=active`、ACTIVE、HOME 确认、Pause ACK、serial release 和 residual process 结果，支持操作员硬件确认及生命周期验证。完整故障周期的 IMU receive/header 序列并非所有 Demo 都已保存。

2026-10-11 后续三模式复验也记录了 PASS：[Feedforward](/home/lyapunov/bone_drill/results/j14/demo/feedforward_20261011_155457_215290/demo_final.json)、[Classical](/home/lyapunov/bone_drill/results/j14/demo/feedforward_pd_20261011_155532_657398/demo_final.json)、[LADRC](/home/lyapunov/bone_drill/results/j14/demo/feedforward_ladrc_20261011_155611_493332/demo_final.json)。交接时这些续测目录仍为本地 Git untracked 数据，不属于冻结 tag 的已归档证据。

## 4. Frozen Boundary

冻结对象包括 production driver、UART protocol、Child / Bridge ownership、safety limits、production launch、既有 YAML 与 controller / geometry 基线；冻结 tag 不移动、不改写。

核心不变量保持：Parent lease 1000 ms、Child 50 Hz、1500 mA 电流保护、80-step reconnect 边界、既有位置 / software travel limits、UART timeout / drain 语义、六轴 Pause、trajectory shaping、startup handshake 与 latest-target semantics。IMU watchdog 门限保留原 controller 定义，不因一次 Demo 故障放宽。

交接时 HEAD 为 `5c2b3d1`，包含冻结 tag 之后的 Demo wrapper / 文档演进；production 源码、launch 和 YAML 相对 tag 未变化。**冻结 tag、当前 working tree、已 source 的 ROS workspace 和实际执行路径必须分别确认**，不能用 tag 存在代替 runtime 一致性检查。

允许更新文档、演进 Demo wrapper / stage-specific lifecycle 与对应软件测试；不得由此暗中改动 production、算法、参数、接口或安全边界。影响实际生命周期的 wrapper 改动仍需对应验证，既有实验与失败记录保持只读。改变冻结对象应作为新的、单独验证的版本处理，不能继续沿用本阶段 PASS 结论。

## 5. Known Limitations

- **No theoretical stability proof**：硬件会话成功和观测器数值有界不等于闭环理论稳定性证明。
- **No absolute accuracy validation**：IMU 姿态观测与局部控制反馈不构成外部参考下的绝对精度验证。
- **No full workspace precision characterization**：未完成全工作空间精度与重复性表征。
- **Automatic recovery / resume not qualified**：完整通信故障恢复、重新锚定和自动恢复运动未获得当前 production 的硬件资格结论。
- **Disturbance comparison not fully standardized**：扰动、payload、姿态、运行窗口和评价指标尚未全部统一，不能宣称 LADRC 优于 Classical 或 Feedforward。
- **External USB reliability remains a follow-up**：曾发生 Base IMU USB 断连并触发 freshness STOP，见 [F3 failure record](/home/lyapunov/bone_drill/results/j14/demo/feedforward_ladrc_20261010_180702_906278/demo_final.json)。USB 断连有内核证据，物理 / 电气触发因素尚未确认；后续 PASS 不证明该类故障已彻底消除。
- **Startup qualification has a boundary**：不能保证任意初始位置均能进入 ACTIVE。2026-10-11 曾在 controller 启动前因负 step target 被安全边界拒绝，见 [startup rejection](/home/lyapunov/bone_drill/results/j14/demo/feedforward_20261011_153957_162728/demo_final.json)；该记录是启动层拒绝，不是 ACTIVE controller 算法失败。
- **Diagnostic retention is limited**：production / Demo 使用有界记录；个别失败缺少精确 fault-tick age 和完整 IMU 序列。摘要 PASS 或 `safe_shutdown=true` 不能替代原始退出日志检查；F3 历史清理日志曾记录 launch 内部 SIGTERM 升级。

## 6. Next Stage Handoff

Future work 保留三个方向：

- **Fair controller comparison**：在可比较的扰动、payload、reference、数据窗口和指标下比较三个控制器。
- **Structure optimization**：以测量证据评估机构、传感器安装和连接拓扑的优化。
- **Further validation**：补充控制效果、运行可靠性和精度证据，独立确认尚未 qualified 的能力。

以上仅交接方向，不定义具体 J15 方案、参数或实验安排。J14-A 提供已验证的 runtime 与 Demo 基础；后续研究需保持冻结边界，并为新结论提供相应证据。
