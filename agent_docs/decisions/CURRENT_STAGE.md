# 6PUS 当前开发阶段状态卡 (Current Stage State Sheet)

**最后更新时间**: 2026-10-10
**当前状态**: **J14-A (Long-Duration Runtime & Communication Reliability) = CLOSED / FROZEN**
**当前开发任务 (Current Development)**: **NONE**
**下一阶段 (Next Stage)**: **NOT YET OPENED**

---

## 1. 阶段状态概览 (Stage Status)

* **已闭环冻结阶段**: **J14-A (Long-Duration Runtime & Communication Reliability) = CLOSED / FROZEN**
  - **Release Tag**: `v1.0-j14-a-three-process-runtime-reliability-qualified`
  - **生产架构 (Production Architecture)**:
    - **Parent Node**: `SingleThreadedExecutor` inline event execution (消除了多线程 wait-set scanning 导致的 IPC 租约饥饿);
    - **UART Child**: 独立子进程，RS485 串口物理独占 owner;
    - **Feedback Bridge**: 独立子进程，底层反馈 IPC 独占 consumer;
    - **Compatibility Relay**: 纯外部 ROS 适配器，零直接串口或指令仲裁权限。
  - **真实硬件资格认证 (Real-Hardware Qualification)**:
    - **Q3-R1D (Classical P/PD Active 60s)**: **PASS** (3529 driver cycles, 2945 ACTIVE cycles @ 50 Hz, 0 hard stop, 0 fault, max IMU age < 57 ms);
    - **Q3-R2 (3-axis LADRC Active 60s)**: **PASS** (3539 driver cycles, 2942 ACTIVE cycles @ 50 Hz, 0 hard stop, 0 fault, max IMU age < 66 ms, LESO 状态全程有界稳定);
    - **三个官方 Demo (Feedforward / PD / LADRC)**: **HUMAN HARDWARE CONFIRMATION = PASS** (全生命周期：Startup HOME → Dual-IMU preflight → ACTIVE → Ctrl+C → Shutdown HOME → Pause 6/6 → 串口干净释放 → 0 残留进程)。
  - **安全红线冻结 (Frozen Safety Invariants)**:
    - 1000 ms Parent 租约保护
    - 50 Hz 伺服主频 ($20\text{ ms}$)
    - 1500 mA 执行器过流保护
    - 80-step / axis 重连跳变抑制
    - 100 ms 双 IMU Age Watchdog
    - 6 轴硬件 Pause (ACK 6/6 确认)
  - **权威总结**: [`agent_docs/stages/J14/J14_FINAL_SUMMARY.md`](../stages/J14/J14_FINAL_SUMMARY.md)
  - **交接规范**: [`agent_docs/stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md`](../stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md)

---

## 2. 权威依据文档 (Authoritative References)

1. **J14 最终总结报告**: [`agent_docs/stages/J14/J14_FINAL_SUMMARY.md`](../stages/J14/J14_FINAL_SUMMARY.md)
2. **J14 跨阶段交接规范**: [`agent_docs/stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md`](../stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md)
3. **J14 生产驱动架构说明**: [`docs/J14_PRODUCTION_DRIVER.md`](../../docs/J14_PRODUCTION_DRIVER.md)
4. **J14 Demo 操作手册**: [`docs/J14_DEMO_ENTRYPOINTS.md`](../../docs/J14_DEMO_ENTRYPOINTS.md)
5. **J13 最终总结报告**: [`agent_docs/stages/J13/J13_FINAL_SUMMARY.md`](../stages/J13/J13_FINAL_SUMMARY.md)
6. **J12 最终总结报告**: [`agent_docs/stages/J12/J12_FINAL_SUMMARY.md`](../stages/J12/J12_FINAL_SUMMARY.md)
7. **J11 最终总结报告**: [`agent_docs/stages/J11/J11_FINAL_SUMMARY.md`](../stages/J11/J11_FINAL_SUMMARY.md)
