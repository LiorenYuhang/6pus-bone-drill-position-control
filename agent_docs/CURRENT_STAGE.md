# 6PUS 当前开发阶段状态卡 (Current Stage State Sheet)

**最后更新时间**: 2026-08-21
**当前状态**: $\mathbf{J5\ PRE\text{-}ASSEMBLY\ BASELINE\ FROZEN}$

---

## 1. 阶段导航 (Stage Status)

* **当前推进阶段**: **$\mathbf{J5\ 开发正式收尾并全面冻结}$；下一阶段为 $\mathbf{J6\ POST\text{-}ASSEMBLY\ COMMISSIONING}$（机械总装与现场调试）**
* **已正式 PASS & 冻结的 Gates (Frozen Milestones)**:
  * **J1 (单总线半双工 921600 8N1 底层驱动协议)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J2 (六轴同步通信、0x29 伴随反馈与基准性能)**: $\mathbf{PASS\ /\ FROZEN}$（10Hz 轮询，18.18ms 串行通信耗时，15.18ms 扫描跨度，0 写入 READ_ONLY 审计）
  * **J3 (IK / 执行器闭环就位验证)**: $\mathbf{PASS\ /\ FROZEN}$（解析逆运动学、S 曲线规划、独立台架电缸 PARK `[0]*6` $\leftrightarrow$ HOME `[1000]*6` 往返就位精度 $\le 2\text{ steps}$）
  * **J4 (连续笛卡尔轨迹控制管线与安全策略)**: $\mathbf{PASS\ /\ FROZEN}$
    - **状态定义**：J4 控制算法与安全策略保持冻结；UART transport 层应用 batch-atomicity hotfix（6 轴 Batch 全局锁 `with self._bus_lock_ctx:` 与 Active 模式轮询抑制）并完成实机连续动态回归；
    - **J4 控制行为实机回归快照**：连续 Heave 动态轨迹下 6 轴协调良好、Zero Safety Violations、Zero Faults，独立台架回归平稳；
  * **J5-A (J5 接口与数据流准备)**: $\mathbf{PASS\ /\ FROZEN}$（`pus6_msgs` 自定义消息包、`6pus_geometry.yaml` 唯一权威几何参数源）
  * **J5-B1 (双 IMU 软件时间同步与时钟频偏消除)**: $\mathbf{PASS\ /\ FROZEN}$ (Base: $-14.63\text{ PPM}$, Plat: $-17.56\text{ PPM}$)
  * **J5-B2 (最终双 IMU 外参与 lever arm)**: $\mathbf{DEFERRED\ \to\ J6}$（等待最终机构安装后执行）
  * **J5-B3 (双 IMU 实时相对姿态估计核心算法与实时链路)**: $\mathbf{PASS\ /\ FROZEN}$
    - 0x51 硬件采样时间戳已完全启用（双路 100% 具备，步长恒定 5000us）；
    - USB 拓扑兼容性恢复已完成：Base H30 Class `0x04` 输出掩码恢复为 `0x80D8` 并固化，双路 `0x51/0x52` 重新验证为 100%，详见 [`agent_docs/J5_USB_TOPOLOGY_RECOVERY.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_USB_TOPOLOGY_RECOVERY.md)；
    - 算法闭环：Software Sync $\to$ Session-Start 5% Trimmed Mean 自动零偏（Auto Bias COMPLETE，1200 样本，6.0s） $\to$ 初始 $\mathbf{R}_{SB,SP}$ $\to$ 双边李群 $\text{SO}(3)$ 姿态积分（122Hz 实时输出）；
    - 静态性能：漂移率降至 **$0.363^\circ/\text{min}$**；同板大动态共模抑制验证 ($80.8^\circ/\text{s}$ 大翻转) 动态 $\text{SO}(3)$ RMS $0.2767^\circ$，归零残差 $0.3899^\circ$；
  * **J5-B4 (平动/加速度融合估计)**: $\mathbf{DEFERRED\ \to\ J6/J7}$（待机械 FK 与双 IMU 最终外参建立后开展）
  * **J5-C1 (多源统一时间轴、Timed Telemetry 接口与 Batch 原子性)**: $\mathbf{PASS\ /\ FROZEN}$
    - 建立以 Host `CLOCK_MONOTONIC` (ns) 为内部基准的统一时间轴；
    - 落地 `TimedJointFeedback` (`per_axis_rx_done_monotonic_ns`) 与 `TimedJointCommand` (`per_axis_tx_start/done_monotonic_ns`)；
    - 序列语义：主动写周期 100% 具备 Command $\leftrightarrow$ Feedback 1-to-1 同号配对，静止/恢复/读周期自然产生 Feedback-only 观测，零乱序零丢失；
    - 实证连续 Batch Overlap = $0\ /\ 392$ ($100.00\%$ 原子性无交错)；
    - 确立 Fixed-Lag 逐轴滑动历史重建规范（六轴全部 Bracketed $\to$ Publish，任一轴未 Bracketed $\to$ WAIT，严格 0 外推）；
  * **J5-C2-SW (6PUS 正运动学 FK 数学实现与 IK→FK 一致性验证)**: $\mathbf{PASS\ /\ FROZEN}$
    - 基于真实硬件 IK 几何（$c=40\text{mm}, d=27.5\text{mm}, e=84\text{mm}, \lambda=30^\circ, \mu=60^\circ$）完成解析雅可比 Levenberg-Marquardt 正运动学求解器；
    - 覆盖安全测试包络 1000 个随机样本：**收敛率 100.00%，平移数值一致性残差均值 $5.83\times 10^{-9}\text{ mm}$，$\text{SO}(3)$ 姿态残差均值 $1.21\times 10^{-7\circ}$，几何残差均值 $1.58\times 10^{-11}\text{ m}$，平均迭代 $4.93$ 次**；
    - 实现 `FixedLagJointSynchronizer` 零外推多轴时钟对齐重建模块（已加 `threading.RLock` 线程安全锁）；
  * **J5-C2-RT-SW (FK 实时解算节点)**: $\mathbf{PASS\ /\ FROZEN}$
    - FK 节点必须使用独立 `SingleThreadedExecutor`（硬约束）；
    - 真实硬件 15s 联调实测：Timed 消息单调连续下发，FixedLag 达成 97.07% SYNCED（微观通信抖动下严格零外推触发 WAIT_FUTURE_SAMPLE），FK 解算 90.6Hz，残差 $< 10^{-11}\text{ m}$，干净退出无崩溃；
  * **测试基线**:
    - **当前 ROS 2 工作空间核心测试集**：`pytest bone_ws/src -v` $\to$ **162 passed**（耗时 39.85s，100% 通过）；
    - **全仓库历史全量用例收集**：`pytest /home/lyapunov/bone_drill --collect-only` $\to$ **468 collected**（包含 162 项工作空间用例 + 306 项历史独立硬件台架用例）。
* **当前物理阻塞条件与延期项 (Deferred to J6)**:
  * 6PUS 完整机构尚未组装，不能进行真实机械 FK 精度或实际 $T_{BP}$ 验证。
  * 两个 H30 IMU 仍固定在同一块测试板，不能进行最终安装外参、lever arm 或 Dual-IMU/FK 融合验证。
  * **J5-C2-HW 实机 FK/IMU 联调**: $\mathbf{DEFERRED\ \to\ J6}$（待 6PUS 机构完整装配后按 Phase 7 执行）。
* **下一阶段入口 (Next Stage Entry Point)**:
  * **J6 POST-ASSEMBLY COMMISSIONING（机械总装与现场调试）**：
    严格依据 [`agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md) 从 Phase 0（机械静态检查）逐步推进至 Phase 8。

---

## 2. 权威依据文档 (Authoritative References)

1. **J5 最终冻结清单**: [`agent_docs/J5_FROZEN_MANIFEST.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_FROZEN_MANIFEST.md)
2. **装配前总验收报告**: [`agent_docs/PRE_ASSEMBLY_DOCUMENT_AND_HARDWARE_ACCEPTANCE.md`](file:///home/lyapunov/bone_drill/agent_docs/PRE_ASSEMBLY_DOCUMENT_AND_HARDWARE_ACCEPTANCE.md)
3. **装配前基线文档**: [`agent_docs/J5_PRE_ASSEMBLY_BASELINE.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_PRE_ASSEMBLY_BASELINE.md)
4. **装配调试规程**: [`agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md)
5. **长期主教学教程**: [`6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](file:///home/lyapunov/bone_drill/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)
6. **长期主实机指南**: [`6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`](file:///home/lyapunov/bone_drill/6PUS_REAL_HARDWARE_OPERATION_GUIDE.md)
7. **J5-B3 总结与冻结文档**: [`agent_docs/J5_B3_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_B3_FINAL_SUMMARY.md)
8. **J4 总结与冻结文档**: [`agent_docs/J4_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/J4_FINAL_SUMMARY.md)
 9. **J5 USB 拓扑恢复报告**: [`agent_docs/J5_USB_TOPOLOGY_RECOVERY.md`](file:///home/lyapunov/bone_drill/agent_docs/J5_USB_TOPOLOGY_RECOVERY.md)
