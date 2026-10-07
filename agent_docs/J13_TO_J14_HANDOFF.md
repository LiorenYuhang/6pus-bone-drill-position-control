# J13 至 J14 阶段交接规范 (J13 to J14 Handoff)
# First-Order 3-Axis LADRC to Long-Duration Runtime & Communication Reliability

- **交接发起方**: **J13 (First-Order Linear ADRC Attitude Residual Control)** — **CLOSED / ENGINEERING QUALIFIED**
- **交接接收方**: **J14 (Long-Duration Runtime & Communication Reliability)** — **NEXT ACTIVE STAGE**
- **交接日期**: 2026-10-07
- **交接基准文档**: [`agent_docs/J13_FINAL_SUMMARY.md`](./J13_FINAL_SUMMARY.md)

---

## 1. J14 继承的 J13 冻结资产与算法状态 (Frozen J13 Assets)

### 1.1 冻结控制架构与参数基准
进入 J14 后，控制算法层全面冻结，**严禁随意改动控制律代码或重新调参**：
1. **Classical Baseline**:
   - $\text{Base SO(3) Feedforward} + \text{Roll P} + \text{Pitch PD} + \text{Yaw PD}$
   - Roll: $K_p = 0.05,\; K_d = 0.0$
   - Pitch: $K_p = 0.05,\; K_d = 0.0015\,\text{s}$
   - Yaw: $K_p = 0.025,\; K_d = 0.0015\,\text{s}$
   - 残差限幅: $\pm 0.5^\circ$
2. **3-axis LADRC**:
   - $\text{Base SO(3) Feedforward} + \text{3-axis LADRC residual control}$
   - Nominal Plant: $\dot{e} = f - b_0 u$
   - LESO: 闭式零阶保持（ZOH）离散化二阶扩张状态观测器
   - Control Law: $u_{\text{raw}} = \frac{\omega_c z_1 + z_2}{b_0},\; u_{\text{actual}} = \text{clip}(u_{\text{raw}}, \pm 0.5^\circ)$
   - Roll: $b_0 = 20.0\,\text{s}^{-1},\; \omega_c = 0.8\,\text{rad/s},\; \omega_o = 2.5\,\text{rad/s}$
   - Pitch: $b_0 = 10.0\,\text{s}^{-1},\; \omega_c = 1.0\,\text{rad/s},\; \omega_o = 4.0\,\text{rad/s}$
   - Yaw: $b_0 = 6.0\,\text{s}^{-1},\; \omega_c = 0.25\,\text{rad/s},\; \omega_o = 1.2\,\text{rad/s}$
   - 残差限幅: 三轴严格统一 $\pm 0.5^\circ$

### 1.2 关键概念与边界约束
- **$z_2$ 术语规范**: 统一称为**等效集总扰动项（Equivalent Lumped Disturbance）**，量纲为 $\text{deg/s}$ 或 $\text{rad/s}$，严禁表述为真实物理力矩；
- **偏心负载规范**: 统一称为**同一未知质量、同一固定位置的偏心重物（same fixed eccentric payload condition）**；
- **对比严谨性**: 严禁在未经严格同源激励对齐的前提下宣称“LADRC 全面优于 Classical”。

---

## 2. J14 阶段第一核心目标：长时运行与通信可靠性

### 2.1 问题来源与工程痛点
J13 的 10s 短时运行证实了控制算法与观测器的稳定性，但底层驱动、串口通信与调度系统表现出显著的长尾延时（Long-Tail Latency）：
- **调度抖动 (Scheduler Gap)**：实测峰值达到 $274.9 \sim 370.6\,\text{ms}$；
- **串口排空延时 (UART Drain Duration)**：实测峰值达到 $182.6 \sim 363.1\,\text{ms}$；
- **反馈数据龄期 (Feedback Age)**：实测峰值达到 $269.9 \sim 352.4\,\text{ms}$；
- **偶发长尾停机**：在更长时段运行中曾出现过因串口读写阻塞或调度饥饿接近/超过 1.0s 而触发的系统停机。

### 2.2 J14 第一攻关目标 (Primary Objective)
**彻底解决真实 6PUS 并联平台在长时间连续运行中的底层通信与调度可靠性问题**，重点攻关：
1. **Communication Blocking**（驱动层串口读写同步阻塞消除）；
2. **UART Long-Tail**（串口发送与排空时延的长尾压制）；
3. **Scheduler Long-Tail**（Python 运行时、GIL 与操作系统线程调度长尾抖动消除）；
4. **Feedback Age Accumulation**（数据反馈积压与陈旧度控制）；
5. **Runtime Stall**（偶发运行断流与饥饿消除）。

### 2.3 严格安全守则：严禁放宽安全门限 (Strict Safety Policy)
**J14 的优化目标是解决导致时延过大的底层根因，绝不是通过放宽安全门限来掩盖问题。**  
以下安全边界必须严格保持生效，除非后续有独立且充分的工程证据证明需修改设计：
- **80-step recovery reject**（恢复目标偏差超过 80 步拒绝执行）；
- **283.3 ms warning**（时效告警门限）；
- **1000 ms hard stop**（硬性停机红线）；
- **1500 mA 相电流上限**（`MAX_CURRENT_MA = 1500`）。

### 2.4 J14 第一阶段最终资格目标 (10-Minute Qualification Target)
**真实 6PUS 硬件平台连续 Active 运行约 10 分钟（~30,000 伺服周期 @ 50 Hz）**。
- **验收合格标准**：
  1. 全程不因通信阻塞或串口超时退出；
  2. 0 次触发 1000 ms Hard STOP；
  3. 0 次触发电机故障（Motor Fault）；
  4. 不出现持续性的反馈饥饿（Persistent Feedback Starvation）；
  5. 不出现不可恢复的调度/UART 断流故障。

---

## 3. J14 第二阶段预留实验方向：末端瞬态扰动响应对比

在 J14 第一阶段 10 分钟长时运行可靠性通过硬件资格认证之后，系统将具备稳定承载长时间连续对比实验的能力。此时方可开展第二阶段预留控制对比实验：

### 3.1 预留实验设计原则
- **工况承接**：回到 J13 已建立的偏心负载安装条件（保持同一固定偏心重物）；
- **扰动形式**：在偏心重物基础上，对动平台末端施加典型人工瞬态外力冲击（如轻微触碰、短暂指推、小幅阶跃式瞬态外力扰动）；
- **对比对象**：在严格一致的基底与测试规程下，对比 **Classical Baseline vs 3-axis LADRC**。

### 3.2 拟观测与评估的核心指标
1. **峰值姿态误差 (Peak Attitude Error)**：瞬态外力冲击瞬间的动态偏转极值；
2. **恢复调节时间 (Recovery Time)**：外力撤除后姿态残差恢复至稳态误差带所需时间；
3. **稳态残差 (Residual Error)**：静态抗重力与外力持续分量的抵消能力；
4. **控制输出与饱和度 (Control Effort & Saturation)**：瞬态冲击期间控制量响应刚度与撞击 $\pm 0.5^\circ$ 限幅的比例；
5. **观测器动态重构性能 (LESO $z_2$ Response)**：扩张状态观测器捕获瞬态扰动的上升沿与衰减特性。

> [!NOTE]
> 第二阶段实验方向目前仅作为 J14 的前瞻性路线储备。在 J14 第一阶段 10 分钟长时稳定性考核未通过之前，**严禁提前开展第二阶段实验或进行任何硬件采集**。

---

## 4. J14 非阻塞支线任务：J14-Q1 ROS 2 QoS 故障检测特性表征

- **任务标识**: **J14-Q1 — ROS 2 QoS Deadline / Liveliness Fault Detection Characterization**
- **任务定位**: 在 J14 runtime 攻关期间独立开展的非阻塞故障检测表征子项（预计工期约 0.5 day，绝对不阻塞 J14 第一主线 10 分钟长时运行达标）。
- **研究目标**: 定量表征在不同故障注入场景下，下游/监控 ROS 2 节点基于 QoS Deadline 与 Liveliness 机制检测到异常的响应时延与可靠性，涵盖四类场景：
  1. Publisher 停止发布数据但进程仍存活；
  2. Publisher 进程被异常 kill；
  3. 通过 `SIGSTOP` 信号冻结/暂停 Publisher 进程；
  4. Publisher 回调或发布流程出现人为长延迟。
- **记录指标**:
  - 故障注入时间戳 (Fault injection timestamp)；
  - Deadline missed 时间戳；
  - Liveliness lost 时间戳；
  - 故障检测时延 (Detection latency)；
  - 误报/虚警计数 (False-positive count)。
- **严格安全红线与实施边界**:
  - 第一版功能边界严格限定为：**检测、打标时间戳、持久化日志与统计指标（detect, timestamp, log, statistics）**；
  - **严禁直接由 QoS 故障触发电机 STOP 动作（严格禁止 QoS fault $\to$ motor STOP）**；
  - **严禁修改生产环境底层安全状态机（Production Safety State Machine）**。
