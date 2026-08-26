# 6PUS 当前开发阶段状态卡 (Current Stage State Sheet)

**最后更新时间**: 2026-08-26  
**当前冻结状态**: $\mathbf{J7\ SIX\text{-}AXIS\ 50Hz\ SERVO\ HARDWARE\ BASELINE\ FROZEN\ v1.0}$  
*(官方 Release Tag: `v1.0-j7-servo-hardware-frozen`)*

---

## 1. 阶段导航 (Stage Status)

* **当前推进阶段**: **$\mathbf{J7\ 伺服运动控制基线正式收尾并全面冻结}$；下一阶段为 Dual-IMU / FK 融合滤波与高频主动抑振**
* **已正式 PASS & 冻结的 Gates (Frozen Milestones)**:
  * **J1 (单总线半双工 921600 8N1 底层驱动协议)**: $\mathbf{PASS\ /\ FROZEN}$
  * **J2 (六轴同步通信、0x29 伴随反馈与基准性能)**: $\mathbf{PASS\ /\ FROZEN}$（10Hz 轮询，18.18ms 串行通信耗时，15.18ms 扫描跨度，0 写入 READ_ONLY 审计）
  * **J3 (IK / 执行器闭环就位验证)**: $\mathbf{PASS\ /\ FROZEN}$（解析逆运动学、S 曲线规划、独立台架电缸 PARK `[0]*6` $\leftrightarrow$ HOME `[1000]*6` 往返就位精度 $\le 2\text{ steps}$）
  * **J4 (连续笛卡尔轨迹控制管线与安全策略)**: $\mathbf{PASS\ /\ FROZEN}$（J4 控制算法与安全策略保持冻结；UART transport 层应用 batch-atomicity hotfix 6 轴 Batch 全局锁 `with self._bus_lock_ctx:` 与 Active 模式轮询抑制）
  * **J5 (装配前软件与传感器基线)**: $\mathbf{PASS\ /\ FROZEN}$（统一时间轴 Host `CLOCK_MONOTONIC`、Fixed-Lag 严格零外推状态重构、解析雅可比 LM 数值正解、双 H30 硬件微秒时间戳与自动零偏估计）
  * **J6 (总装后现场调试与 HOME/PARK 定位基线)**: $\mathbf{PASS\ /\ FROZEN}$
    - 定位模式（mode=0, 10Hz, speed=50）六轴独立 HOME (`[1000]*6`) / PARK (`[0]*6`) 启动与收拢调试，通过连续 10 循环实机测试（详见 [`agent_docs/j6/J6_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/j6/J6_FINAL_SUMMARY.md)）；
    - J6 定位模式作为系统底层 commissioning、故障排查与可靠 fallback 基线。
  * **J7 (六轴 50Hz Servo 伺服运动控制实机基线 -- 当前版本)**: $\mathbf{PASS\ /\ FROZEN}$
    - 50Hz 连续 0x29 目标流（周期 $20.00\,\text{ms} \pm 0.10\,\text{ms}$，单总线纯写批次耗时均值 $3.38\,\text{ms}$，最大 $\le 6.83\,\text{ms}$）；
    - 非阻塞轮询反馈调度器（每周期 1 轴 0x30 状态查询，单轴采样率 $\approx 8.33\,\text{Hz}$），运行期取消 0x25 模式回读，以避免错误 mode 门禁与额外 UART/RX residual 交互；
    - PARK $\leftrightarrow$ HOME 全行程（1000 步 / 25.0mm）五次多项式 S 曲线实机验证（2326 周期，46.50s，Fault=0，Timeout=0，0x1A 干净停机）；
    - A/B 定量对比证实：目标刷新率提升 5 倍，全行程 RMS 跟踪误差降低 61.6%（$7.53 \to 2.89\,\text{步}$），峰值冲击电流降低 46.9%（$145 \to 77\,\text{mA}$），实机视觉运动显著平滑；
    - 工作空间核心功能包全量单元测试 **304 项 100% PASS**。

---

## 2. 权威依据文档 (Authoritative References)

1. **统一文档地图**: [`agent_docs/README.md`](file:///home/lyapunov/bone_drill/agent_docs/README.md)
2. **唯一实机操作指南**: [`6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`](file:///home/lyapunov/bone_drill/6PUS_REAL_HARDWARE_OPERATION_GUIDE.md)
3. **ROS 2 架构与教学教程**: [`6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](file:///home/lyapunov/bone_drill/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)
4. **J7 决策与时序审计**: [`agent_docs/decisions/J7_C2_UART_TIMING_BUDGET_AUDIT.md`](file:///home/lyapunov/bone_drill/agent_docs/decisions/J7_C2_UART_TIMING_BUDGET_AUDIT.md)
5. **J6 最终调试总结**: [`agent_docs/j6/J6_FINAL_SUMMARY.md`](file:///home/lyapunov/bone_drill/agent_docs/j6/J6_FINAL_SUMMARY.md)
6. **J5 最终冻结清单**: [`agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md`](file:///home/lyapunov/bone_drill/agent_docs/frozen/J5_PRE_ASSEMBLY_FROZEN_v1.0.md)
