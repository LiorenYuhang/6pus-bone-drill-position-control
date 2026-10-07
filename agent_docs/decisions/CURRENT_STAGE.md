# 6PUS 当前开发阶段状态卡 (Current Stage State Sheet)

**最后更新时间**: 2026-10-07

**当前状态**: **J14 ACTIVE (Long-Duration Runtime & Communication Reliability)**

**前序阶段闭环**: **J13 CLOSED / ENGINEERING QUALIFIED**

---

## 1. 阶段导航 (Stage Status)

* **当前活动阶段**: **J14 (Long-Duration Runtime & Communication Reliability) = ACTIVE**
  - **核心定位**: 解决真实 6PUS 并联平台在长时间连续运行中的底层通信与调度可靠性瓶颈；
  - **攻关重点**: Communication Blocking、UART Long-Tail、Scheduler Long-Tail、Feedback Age Accumulation 与 Runtime Stall；
  - **安全守则**: 严格保持 80-step recovery reject、283.3 ms warning、1000 ms hard stop 与 1500 mA 安全红线不变；
  - **第一阶段最终资格目标**: 真实 6PUS 硬件平台连续 Active 运行约 10 分钟（~30,000 周期 @ 50 Hz，0 通信退出 / 0 hard stop / 0 motor fault / 0 持续饥饿）；
  - **第二阶段预留方向**: 10 分钟长时稳定性通过后，回到偏心负载工况，开展末端人工瞬态扰动下的 Classical vs 3-axis LADRC 对比；
  - **J14-Q1 支线任务**: ROS 2 QoS Deadline / Liveliness 故障检测独立表征（非阻塞，~0.5 day，仅做检测/时间戳/统计，严禁联动 motor STOP）；
  - **阶段交接规范**: [`agent_docs/J13_TO_J14_HANDOFF.md`](../J13_TO_J14_HANDOFF.md)

* **已闭环资格认证阶段**: **J13 (First-Order Linear ADRC Residual Control) = CLOSED / ENGINEERING QUALIFIED**
  - **核心定位**: First-Order Linear ADRC / LADRC 动平台姿态残差主动抗扰控制实机实现与资格认证；
  - **J13-1A ~ 1C**: MATLAB 仿真审计、实机 Plant 辨识、离散 LADRC 数学认证与回放，**PASS**；
  - **J13-2A1 ~ 2A2**: 影子模式实机遥测闭环（4工况/8762周期/0故障），**PASS / QUALIFIED**；
  - **J13-2B1**: Roll 单轴 LADRC 物理接管静态验收与重复性补充，**PASS / QUALIFIED**；
  - **J13-E0**: 伺服调度器断流专项诊断（确认存在底层 UART blocking 与调度饥饿现象；实验性 driver 修复未合入 main，production driver 严格保持 J12 冻结基线零变更，以 PASS_WITH_RUNTIME_WARNINGS 闭环并移交 J14 专班解决）；
  - **J13-2C1 (3-axis Active 10s Qualification)**: 空载 10s 全三轴物理接管测试（500 周期 / 0 停机 / 0 故障 / 观测器稳定），**PASS / QUALIFIED**；
  - **J13 Payload A/B (偏心负载工程验证)**: Classical (`j13_3axis_payload_classical_10s`) vs 3-axis LADRC (`j13_3axis_payload_ladrc_10s`)，两组均完成 500 周期全流程闭环；
  - **权威总结**: [`agent_docs/J13_FINAL_SUMMARY.md`](../J13_FINAL_SUMMARY.md)
  - **Windows 交接包**: [`results/j13/J13_payload_AB_win_package_20261007.zip`](../../results/j13/J13_payload_AB_win_package_20261007.zip)

* **已完成冻结阶段**: **J12 (Classical Dynamic Performance Characterization) = CLOSED / FROZEN** (`v1.0-j12-classical-dynamic-performance-frozen`)
  - **核心定位**: Classical Feedforward + P/PD Dynamic Performance Characterization（经典姿态稳定控制性能与物理负载摸底）；
  - **J12-1 / J12-2 / J12-3**: 大角度静态摸底、组合姿态筛选、手持高动态表征（有效角速度峰值 $92.33^\circ/\text{s}$，电流峰值 468 mA，瓶颈确认为残差饱和与执行器速度滞后）；
  - **生产安全门限**: 主机 `MAX_CURRENT_MA = 1500`（对齐执行器 0x20 硬件过流保护）；
  - **权威总结**: [`agent_docs/J12_FINAL_SUMMARY.md`](../J12_FINAL_SUMMARY.md)

* **前序冻结基准**: **J11 (Feedforward + Classical P/PD Feedback) = FROZEN** (`v1.0-j11-attitude-stabilization-frozen`)
  - **最终参数**: `Kp=[0.05, 0.05, 0.025]`, `Kd=[0, 0.0015, 0.0015] s`, `Ki=[0, 0, 0]`, `fc=5 Hz`, 残差限幅 $\pm 0.5^\circ$；
  - **权威入口**: [`agent_docs/j11/J11_FINAL_SUMMARY.md`](../j11/J11_FINAL_SUMMARY.md)

* **前序物理基线**: J1~J10 全面冻结（J1 串口协议、J2 解析逆解、J3 调度覆盖、J4 轨迹平滑、J5 纳秒时间轴与数值正解、J6 机械总装、J7 50Hz 伺服、J8 笛卡尔运动、J9 双 IMU、J10 6DOF 工作空间）。

---

## 2. 权威依据文档 (Authoritative References)

1. **J13 最终总结报告**: [`agent_docs/J13_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/J13_FINAL_SUMMARY.md)
2. **J13 至 J14 交接规范**: [`agent_docs/J13_TO_J14_HANDOFF.md`](file:///home/lyapunov/bone_drill/agent_docs/J13_TO_J14_HANDOFF.md)
3. **J12 最终总结报告**: [`agent_docs/J12_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/J12_FINAL_SUMMARY.md)
4. **J11 最终总结报告**: [`agent_docs/j11/J11_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/j11/J11_FINAL_SUMMARY.md)
5. **J10 最终总结报告**: [`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/stages/J10/J10_FINAL_SUMMARY.md)
6. **J8 最终硬件准入报告**: [`agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md)
