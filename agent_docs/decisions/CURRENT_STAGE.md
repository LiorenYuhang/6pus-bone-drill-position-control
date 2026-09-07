# 6PUS 当前开发阶段状态卡 (Current Stage State Sheet)

**最后更新时间**: 2026-09-07  
**当前冻结状态**: $\mathbf{J9\ IMU\ Integration\ \&\ Measurement\ Qualification\ COMPLETE}$  
*(官方 Release Tag: `v1.0-j8-active-servo-hardware-qualified`)*

---

## 1. 阶段导航 (Stage Status)

* **当前推进阶段**: **$\mathbf{J9\ IMU\ Integration\ \&\ Measurement\ Qualification\ COMPLETE}$；下一阶段 (J10) 为 Task-Space Operational Safe Workspace Measurement**
* **已正式 PASS & 冻结的 Gates (Frozen Milestones)**:
  * **J1 (单总线半双工 921600 8N1 底层驱动协议)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J2 (六轴同步通信、0x29 伴随反馈与基准性能)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J3 (IK / 执行器闭环就位验证)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J4 (连续笛卡尔轨迹控制管线与安全策略)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J5 (装配前软件与传感器基线)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J6 (总装后现场调试与 HOME/PARK 定位基线)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J7 (六轴 50Hz Servo 伺服运动控制实机基线)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J8 (连续笛卡尔伺服、Design-B 0x29 伴随反馈与全自由度实机验证 -- 当前版本)**: $\mathbf{PASS\ /\ FROZEN}$
    - **系统架构**: `Cartesian Target (50Hz) -> Analytical IK -> Trajectory Shaper -> J7 Design-B Servo -> 0x29 StatusFeedback -> TimedJointFeedback -> FixedLag (tau=160ms) -> LM Analytical Jacobian FK (50Hz)`；
    - **Design-B 伴随反馈**: ACTIVE 模式以 50Hz 0x29 目标写入伴随返回的 `StatusFeedback` 作为运行期主反馈源，常规阻塞式 0x30 轮询完全移出 50Hz 关键控制路径；
    - **两级安全看门狗**:
      - Tier 1: 执行周期内 6/6 轴反馈完整性监测，连续 $N=4$ 周期缺轴触发通信安全保护；
      - Tier 2: 263.324 ms 调度/截止期非实时操作系统安全上限（非反馈超期时间），容忍标准 Linux 调度抖动；
    - **集约化 PARK/HOME 契约**:
      - `PARK_CMD = 0 step` ($25.000\text{ mm}$)，`HOME_CMD = 1000 step` ($50.000\text{ mm}$)；
      - 引入暂定 PARK 测量带 $[-5, +5]\text{ steps}$，多样本稳定性校验通过后判定 `VALID_PARK`，规划种子设为 `[0]*6`，指令空间严格 $\ge 0$；
      - 遥测保留原始未篡改编码器读数，严格拒绝混合/未分类负向异常状态；
    - **三轮自治硬件准入**: 连续 3 轮完整 PARK $\to$ HOME $\to$ 20s 保持 $\to$ PARK 循环 100% PASS（56,754 / 56,754 帧反馈，0 故障，0 超时，0 停机）；
    - **最终 9 阶段实机视频演示考核**: 137.128s，6898 周期，实测持续刷新率 50.30 Hz，41,388 / 41,388 响应（100.0% 完整率），0 丢帧，0 故障，0 STOP，各轴单向与 6 自由度复合运动全部平滑就位；
    - **全工作空间回归测试**: 349 项单元与集成测试 **100% PASS**。

---

## 2. 权威依据文档 (Authoritative References)

1. **J8 最终硬件准入报告**: [`agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md)
2. **J9 收口文档**: [`J9_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/J9_FINAL_SUMMARY.md) / [`J9_TO_J10_HANDOFF.md`](file:///home/lyapunov/bone_drill/J9_TO_J10_HANDOFF.md)
3. **唯一日常实机操作指南**: [`6PUS_SERVO_OPERATION_GUIDE.md`](file:///home/lyapunov/bone_drill/6PUS_SERVO_OPERATION_GUIDE.md)
4. **ROS 2 架构与教学教程**: [`6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](file:///home/lyapunov/bone_drill/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)
5. **J7 基线设计与决策**: [`agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md)
