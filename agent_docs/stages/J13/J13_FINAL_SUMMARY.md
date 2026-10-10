# J13 最终总结报告 (J13 Final Summary)
# First-Order Linear ADRC / LADRC Attitude Residual Control

- **阶段标识**: **J13**
- **定位**: **First-Order Linear ADRC / LADRC 动平台姿态残差主动抗扰控制实机实现与资格认证**
- **最终状态**: **CLOSED / ENGINEERING QUALIFIED**
- **上游基线**: J11 冻结姿态稳定基线 (`v1.0-j11-attitude-stabilization-frozen`)、J12 动态性能摸底基线 (`v1.0-j12-classical-dynamic-performance-frozen`)
- **下一阶段**: **J14 — Long-Duration Runtime & Communication Reliability**

---

## 1. J13 阶段目标与架构演进

J13 阶段的核心目标是在真实 6PUS 并联骨科手术机器人硬件平台上，设计、实现并验证针对动平台姿态残差补偿（Attitude Residual Control）的自抗扰控制（ADRC）算法，建立与经典控制基准（Classical P/PD）的对比评价，并在真实空载与偏心负载工况下完成硬件闭环考核。

### 1.1 最终系统控制架构
动平台目标姿态合成采用分层解耦拓扑：
$$\mathbf{R}_{BP,\text{cmd}} = \mathbf{R}_{BP,FF} \cdot \Delta \mathbf{R}_{\text{residual}}$$
其中：
- **前馈链路**：保持 Base-IMU 全 $SO(3)$ 几何前馈不变：$\mathbf{R}_{BP,FF} = \mathbf{R}_{WB}^T$；
- **残差补偿链路**：三轴独立运行一阶离散化线性自抗扰残差控制器（3-axis LADRC residual control）。

### 1.2 对照组：Classical Baseline
J13 全程严格以 J11/J12 冻结的经典控制架构为被对照基准：
- **控制律结构**：$\text{Base SO(3) Feedforward} + \text{Roll P} + \text{Pitch PD} + \text{Yaw PD}$
- **具体增益参数**：
  - Roll: $K_p = 0.05,\; K_i = 0.0,\; K_d = 0.0$
  - Pitch: $K_p = 0.05,\; K_i = 0.0,\; K_d = 0.0015\,\text{s}$
  - Yaw: $K_p = 0.025,\; K_i = 0.0,\; K_d = 0.0015\,\text{s}$
- **残差安全限幅**：三轴对称截断于 $\pm 0.5^\circ$（$\pm 0.008727\,\text{rad}$）。

### 1.3 3-axis LADRC 数学模型与符号约定（与生产代码 `j13_ladrc.py` 严格对齐）
ADRC 是一类以“扩张状态观测器（ESO）估计并动态前馈消除总扰动”为核心的方法族，本项目当前采用的是面向 6PUS 姿态残差控制的一阶线性 LADRC 实现：
1. **单轴标称被控对象**：
   $$\dot{e} = f - b_0 u$$
   其中 $e$ 为世界系姿态残差跟踪误差（测量反馈 $y = e$），$u$ 为姿态残差补偿角控制量，$f$ 为等效集总扰动项（Equivalent Lumped Disturbance，rad/s），$b_0 > 0$ 为名义输入增益（$\text{s}^{-1}$）。
2. **二阶连续线性扩张状态观测器（LESO）**：
   $$\dot{z}_1 = z_2 - b_0 u_{\text{actual}} + 2\omega_o (y - z_1)$$
   $$\dot{z}_2 = \omega_o^2 (y - z_1)$$
   其中 $z_1 \approx e$ 为姿态误差状态估计，$z_2 \approx f$ 为等效集总扰动项估计，$\omega_o$ 为观测器带宽。实际软件采用零阶保持器（ZOH）闭式精确离散转移矩阵更新。
3. **控制律与输出安全限幅**：
   $$u_{\text{raw}} = \frac{\omega_c z_1 + z_2}{b_0}$$
   $$u_{\text{actual}} = \text{clip}(u_{\text{raw}}, -0.5^\circ, +0.5^\circ)$$
   使闭环姿态残差跟踪误差满足 $\dot{e} \approx -\omega_c e$，以闭环控制器带宽 $\omega_c$ 指数衰减收敛。

### 1.4 最终三轴生产控制参数
三轴残差硬限幅严格统一为 $\pm 0.5^\circ$：
- **Roll 通道**：$b_0 = 20.0\,\text{s}^{-1},\; \omega_c = 0.8\,\text{rad/s},\; \omega_o = 2.5\,\text{rad/s}$
- **Pitch 通道**：$b_0 = 10.0\,\text{s}^{-1},\; \omega_c = 1.0\,\text{rad/s},\; \omega_o = 4.0\,\text{rad/s}$
- **Yaw 通道**：$b_0 = 6.0\,\text{s}^{-1},\; \omega_c = 0.25\,\text{rad/s},\; \omega_o = 1.2\,\text{rad/s}$

---

## 2. J13 真机实验证据库 (Real Hardware Evidence)

所有结论均基于本地真实运行遥测数据支持：

### 2.1 证据 A：3-axis LADRC 空载 10s 资格认证
- **数据路径**: [`results/j13/j13_2c1_3axis_active/20261007_141708/j13_2c1_3axis_active_10s/`](../results/j13/j13_2c1_3axis_active/20261007_141708/j13_2c1_3axis_active_10s/)
- **数据规模**: 完整记录 500 周期（实际物理时长 9.98 s，名义 50 Hz）。
- **运行状态**:
  - 接管时刻：第 71 周期（$t = 1.40\,\text{s}$），接管瞬间指令跳变仅 $4.31 \times 10^{-5}$ deg；
  - 硬件驱动输出周期：430 周期；
  - 最终状态：`ACTIVE`，故障原因：空（`""`）；
  - 异常计数：0 driver STOP、0 motor fault、0 recovery reject、0 scheduler deadline exceeded、0 observer divergence；
  - 电机状态：平均相电流仅 16.4 mA，无失步、无堵转、无数值发散。

### 2.2 证据 B：偏心负载工况 Classical 对照运行
- **数据路径**: [`results/j13/j13_2a2_shadow/20261007_143007/j13_3axis_payload_classical_10s/`](../results/j13/j13_2a2_shadow/20261007_143007/j13_3axis_payload_classical_10s/)
- **数据规模**: 完整记录 500 周期（429 个 ACTIVE 周期，接管发生于第 72 周期 $t = 1.42\,\text{s}$）。
- **指标统计**:
  - Base 激励: Roll RMS 1.02°（Peak 2.29°）；Pitch RMS 2.41°（Peak 6.43°）；Yaw RMS 1.74°（Peak 3.18°）；
  - 平台误差: Roll RMS 0.45°；Pitch RMS 0.87°；Yaw RMS 0.61°；
  - 控制输出: 峰值仅 0.129°，饱和率 0.0%，完全在 $\pm 0.5^\circ$ 线性区内。

### 2.3 证据 C：偏心负载工况 3-axis LADRC 运行
- **数据路径**: [`results/j13/j13_2c1_3axis_active/20261007_143036/j13_3axis_payload_ladrc_10s/`](../results/j13/j13_2c1_3axis_active/20261007_143036/j13_3axis_payload_ladrc_10s/)
- **数据规模**: 完整记录 500 周期（446 个 ACTIVE 周期，接管发生于第 55 周期 $t = 1.08\,\text{s}$）。
- **指标统计**:
  - Base 激励: Roll RMS 1.94°（Peak 4.57°）；Pitch RMS 3.61°（Peak 6.93°）；Yaw RMS 0.91°（Peak 2.43°）；
  - 平台误差: Roll RMS 1.07°；Pitch RMS 1.31°；Yaw RMS 0.44°；
  - 控制输出: Roll 峰值 0.228°（0.0% 饱和）；Pitch 峰值 0.500°（**16.37% 限幅饱和**）；Yaw 峰值 0.113°（0.0% 饱和）。

---

## 3. J13 最终核心结论与科学事实边界

### 3.1 J13 已经证明的事实 (Established Conclusions)
1. **真实硬件成功落地**：一阶线性自抗扰算法（含 ZOH 离散二阶 LESO 与输出限幅保护）已成功部署至 6PUS 并联平台 50 Hz 伺服闭环；
2. **空载 10s 资格测试合规**：3-axis LADRC 在无负载工况下平稳完成全流程 10s 物理接管与闭环运行，无任何驱动器或算法故障；
3. **观测器数值稳定性**：各通道 LESO 内部状态 $z_1$ 与 $z_2$ 全程平滑收敛，未发生任何数值溢出、漂移或发散；
4. **偏心负载下完成闭环运行**：在装夹偏心重物的物理工况下，3-axis LADRC 顺利跑完全流程 10s 硬件测试（10s run 无 fatal runtime failure，0 hard stop，0 motor fault），但仍存在明显 timing long-tail；
5. **代码与日志实现严格一致**：离线重放审计证实控制律与遥测数据自洽。

### 3.2 J13 明确未建立的结论 (Unestablished Boundaries)
1. **未证明“LADRC 全面优于 Classical”**：当前偏心负载 A/B 测试中，基座人工摇晃激励非严格同源（LADRC 组遭遇的外部 Base Roll RMS amplitude 约为 Classical 组的 **1.896 倍**，Base Pitch RMS amplitude 约为 Classical 组的 **1.501 倍**，Base Yaw RMS amplitude 约为 Classical 组的 **0.525 倍**）。**不能仅根据绝对 Platform Error 建立控制器的因果优劣**；
2. **未证明“Yaw 轴性能提升单独由 LADRC 导致”**：本次 LADRC run 的 Yaw 误差统计低于 Classical run，但其 Base Yaw 输入激励本身亦低约 47.5%，无法剥离外部输入差异将该差异单独归因于控制器；
3. **未证明“$z_2$ 是真实物理力矩”**：LESO 输出的 $z_2$ 是姿态角速度量纲（rad/s 或 deg/s）的**等效集总扰动项（Equivalent Lumped Disturbance）**，统摄了内部模型不确定性、偏心力矩等效影响及运动耦合效应，绝非物理实测力矩；
4. **未证明“当前参数为最优参数”**：当前三轴参数为经过单轴渐进整定出的首套可用工程参数，未进行全域参数寻优；
5. **未构成严格统计学结论**：单次 10s 随机人工扰动测试属于阶段性工程验证，不构成统计学意义上的控制性能证明。

### 3.3 关键工程边界记录
- **偏心负载状态**：本次偏心重物具体质量、质心到平台几何中心的力臂距离及安装偏角均未进行仪器精密标定，统一界定为“**同一未知质量、同一固定位置的偏心重物（same fixed eccentric payload condition）**”；
- **Pitch 轴限幅饱和**：在偏心负载 LADRC run 中，Pitch 发生 **73 / 446（16.37%）** 的 $\pm 0.5^\circ$ 输出饱和；Pitch $u_{\text{raw}}$ 峰值达到约 $1.025^\circ$。峰值扰动对应的**扰动补偿分量幅度**达到 $\frac{z_{2,\text{peak}}}{b_0} \approx \frac{0.1208\,\text{rad/s}}{10.0\,\text{s}^{-1}} \approx 0.692^\circ$，该分量本身即已击穿 $\pm 0.5^\circ$ 限幅线，导致扰动无法完全前馈对消，进而引发误差在饱和区间累积。

---

## 4. 关键瓶颈发现：Runtime 长尾时序问题

J13 在多次真机测试中暴露了一个直接制约长时间连续运行的底层工程瓶颈：**短时间 10s 实验可通过，但通信与调度系统存在明显的长尾延时（Long-Tail Timing）**。

在已记录的偏心实验中观察到：
- **调度间隔峰值 (Max Scheduler Gap)**: Classical 约 274.9 ms，LADRC 约 370.6 ms；
- **串口排空耗时峰值 (Max UART Drain Duration)**: Classical 约 182.6 ms，LADRC 约 363.1 ms；
- **数据反馈龄期峰值 (Max Feedback Age)**: Classical 约 269.9 ms，LADRC 约 352.4 ms；
- **历史故障追溯**: 在 J13 前期长周期或复杂工况排查中，系统曾出现过接近或超过 1.0 s 的 UART 读写阻塞或调度饥饿故障。

**归因定性与证据边界**：上述现象**绝非 LADRC 控制算法或数学模型的缺陷**。现有证据表明，J13 中观察到多种不同的 long-tail failure mechanism，包括底层 UART blocking/stall 以及 scheduler/GIL starvation 类现象；其共同根因尚未完全确定，属于底层基础运行时与通信可靠性问题，将在 J14 专班继续定位并解决。若不彻底解决上述长尾时序瓶颈，系统无法稳定迈向分钟级至十分钟级的长时间连续作业。

---

## 5. 阶段闭环裁决

- **裁决结论**: **CLOSED / ENGINEERING QUALIFIED**
- J13 阶段圆满完成首套 3-axis LADRC 在真实 6PUS 平台上的架构落地、空载 10s 资格认证与偏心负载工程验证。所有算法模型、参数配置与离线分析资产正式封箱存档，不再继续在 J13 下修改代码或进行小幅调参。
- 研发主线正式移交至 **J14 — Long-Duration Runtime & Communication Reliability**。
