# J12 最终总结报告 (J12 Final Summary)
# Classical Feedforward + P/PD Dynamic Performance Characterization

- **阶段标识**: **J12**
- **定位**: **Classical Feedforward + P/PD Dynamic Performance Characterization (经典前馈+P/PD动态性能与物理负载摸底)**
- **状态**: **CLOSED / FROZEN**
- **冻结 Tag**: `v1.0-j12-classical-dynamic-performance-frozen`
- **上游基线**: J11 冻结姿态稳定基线 (`v1.0-j11-attitude-stabilization-frozen`)
- **下一阶段**: J13 ADRC 姿态主动抗扰控制 (`ADRC Attitude Disturbance Rejection`)

---

## 1. 阶段目标与执行路线

J11 阶段已经闭环并冻结了经典姿态防抖架构（Base-IMU 全 $SO(3)$ 旋转前馈 + Platform-IMU 任务空间残差 P/PD 反馈，参数 $K_p = [0.05, 0.05, 0.025]$，$K_d = [0, 0.0015, 0.0015]\text{ s}$，$f_c = 5.0\text{ Hz}$，三轴残差限幅 $\pm 0.5^\circ$）。

J12 阶段的核心目标**不是重新调参，也不是开发新控制律（ADRC 明确属于 J13）**，而是在真实物理平台上对经典架构进行系统性的**极限负载边界摸底与动态性能表征**，具体划分为三大子模块：
1. **J12-1**: Large-Angle / Configuration-Dependent Load Characterization（大角度静态/准静态负载边界摸底）
2. **J12-2**: Combined Quasi-Static Attitude Screening（代表性三轴组合姿态准静态负载筛选）
3. **J12-3**: Handheld Dynamic Performance Characterization（手持快速/高动态性能与电机电流边界摸底）

---

## 2. J12-1: 大角度静态负载边界摸底结论

- **核心实验**:
  - `J12-1A` (`20261005_104403`): Base Roll $\approx -25.6^\circ$ 静态台阶测试，六轴跟踪正常，稳态电流均值 $< 15\text{ mA}$（峰值 $\le 69\text{ mA}$），无非线性升流，**PASS**。
  - `J12-1B` (`20261005_110720`): Base Roll $+32.3^\circ$ 存在 3.8s 稳定台阶，六轴跟踪与电流完全安全；但在负向大角度（约 $-38^\circ \sim -45^\circ$）时触发高负载与跟踪劣化。
- **权威科学表述规范**:
  - 在 $-38^\circ \sim -45^\circ$ 深倾角区域观察到的高电流与跟踪误差恶化，正式统一定义为：  
    **`configuration-dependent mechanical / kinematic loading with actuator tracking deterioration`**  
    （构型相关的机械/运动学重载伴随执行器跟踪性能恶化）。
  - **严格禁止**将其不严谨地表述为“confirmed singularity”（已确认奇异点）、“confirmed dead point”（已确认机构死点）或“460~477 steps physical stroke limit”（物理行程极限）。机构未触碰硬限位，主要源于大倾角下机构雅可比条件数劣化与连杆几何受力剧增。
- **工程结论**: 确立正常控制安全姿态边界为 Base $SO(3) \le 30^\circ$，后续动态实验避开负向深倾角陷阱。

---

## 3. J12-2: 三轴组合姿态准静态筛选结论

- **核心实验**: `20261005_122117`（主数据 `j12_2_combined_screening_feedforward_powered.csv`，3584 周期，0 故障退出，归位 HOME 最大偏差 3 steps）。
- **筛选工况**:
  - Case A: Roll $\approx +15^\circ$, Pitch $\approx +10^\circ$, Yaw $\approx +5^\circ$
  - Case B: Roll $\approx +15^\circ$, Pitch $\approx -10^\circ$, Yaw $\approx -5^\circ$
  - Case C: Roll $\approx -15^\circ$, Pitch $\approx +10^\circ$, Yaw $\approx -5^\circ$
  - Case D: Roll $\approx -15^\circ$, Pitch $\approx -10^\circ$, Yaw $\approx +5^\circ$
- **权威科学表述规范**:
  - 实验证明上述**代表性三轴组合姿态均安全通过（PASS）**，稳态电流保持在 $20 \sim 150\text{ mA}$，无异常非线性升流。
  - **严格限制结论外推**：仅证明所测代表性离散组合工况安全可用，**不可宣称完整连续姿态工作域已全部认证**。

---

## 4. J12-3: 手持高动态性能表征结论

- **最终闭环 Run**: [`results/j12/j12_3_dynamic_performance/20261005_152615/`](../results/j12/j12_3_dynamic_performance/20261005_152615/)
- **数据规模**: 完整记录 2078 周期（2007 个 ACTIVE 闭环伺服周期，有效时长 40.14 s）。
- **实测关键指标**:
  - **Base SO(3) 姿态**: 最大值 **`21.51°`**，均值 `9.37°`（三轴范围：Roll `[-10.63°, +18.29°]`, Pitch `[-19.43°, +18.16°]`, Yaw `[-19.15°, +13.20°]`）。
  - **Base 空间角速度 $|\omega|_B$**: 实测**有效物理角速度峰值（observed effective peak angular rate）为 `92.33°/s`**，P95 为 **`46.16°/s`**，P99 为 `71.93°/s`，均值为 `16.72°/s`。  
    *(注：严谨定义为手持测试下观察到的有效角速度峰值，不得称为系统的绝对闭环带宽或物理极限)*。
  - **三档动态阶梯验证**:
    - *Slow 段 (t=1.7~15.6s)*: $|\omega|_B$ 均值 10.65°/s，峰值 34.65°/s，平台误差均值 3.66°，峰值 10.44°，最大电流 224 mA；
    - *Medium 段 (t=15.8~30.3s)*: $|\omega|_B$ 均值 14.41°/s，峰值 51.76°/s，平台误差均值 10.64°，峰值 23.13°，最大电流 468 mA；
    - *Fast 段 (t=30.5~38.4s)*: $|\omega|_B$ 均值 28.37°/s，峰值 92.33°/s，平台误差均值 10.75°，峰值 29.13°，最大电流 188 mA。
  - **Platform SO(3) 误差**: 全 Run RMS = **`10.43°`**，P95 = **`22.47°`**，Max = **`29.13°`**。在 $|\omega|_B > 20^\circ/\text{s}$ 时误差出现明显跃升。
  - **最大跟踪误差**: 全系统极值为 **`163 steps` (4.08 mm)**（位于 ID2 与 ID5）。各轴最大误差：ID1=139, ID2=163, ID3=157, ID4=135, ID5=163, ID6=154 steps。
  - **最大实测电流与安全门限**:
    - 全 Run 实际最大电流为 **`468 mA`**（发生于 ID6，t = 25.060s，当时 Base SO(3)=12.01°，$|\omega|_B=25.27^\circ/\text{s}$，处于 Medium 快速换向工况）。
    - **分级阈值统计**: `>500 mA`: **0 次 (0.00%)**；`>800 mA`: 0 次；`>1000 mA`: 0 次；`>1200 mA`: 0 次；`>1400 mA`: 0 次。
- **最终停机原因 (STOP Reason)**:
  - 触发信息: `Timeout recovery target rejected: max jump 81 steps > limit 80 steps. Target generator must re-anchor before resuming.`
  - 性质认定: 属于**主机通信看门狗 Slew-rate 跳变防护门禁**。**绝非 Overcurrent（过流），绝非 Actuator Internal Fault（执行器故障），绝非 Mechanical Jam（机构卡死）**。停机前 2007 周期数据全部为有效科研数据。
- **高动态瓶颈归因**:
  - 当前架构在高动态下**绝不是 Current Limited**（峰值电流 468 mA，稳态温和，远离 1500 mA 电机硬件保护线）；
  - 核心瓶颈是 **`Control Residual Saturation + Actuator Tracking Limitation`** 共同作用：
    1. *控制端*: 三轴 PID 残差被截断于 $\pm 0.5^\circ$，在大扰动下 100% 饱和，无法继续输出纠偏量；
    2. *执行端*: 电机受步频速度斜率限制，换向滞后达 163 steps（4.08 mm）。

---

## 5. 生产电流门限正式修改 (MAX_CURRENT_MA = 1500)

1. **修改决策**:
   - 将主机端生产安全常数正式统一修改为：  
     `MAX_CURRENT_MA: int = 1500`（修改文件: [`pus6_motor_driver/safety.py`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py)）
2. **决策依据**:
   - 六轴执行器内部硬件过流寄存器 `0x20` 均已实测确认固定为 1500 mA；
   - 彻底废除历史 500 mA 宿主机保守硬熔断门限，将主机软件保护与执行器内部硬件保护严格对齐于 1500 mA；
   - 不修改执行器内部 `0x20` 寄存器。
3. **保持极简设计**:
   - 维持 `current_mA > 1500 -> STOP`，不增加 600/700/800 mA 告警、不增加防抖滤波、不增加 $I^2t$ 热模型、不增加复杂状态机。
4. **清理临时机制**:
   - 完全移除并停止依赖 `PUS6_HOST_MAX_CURRENT_MA` 环境变量；生产代码默认即为 1500 mA。

---

## 6. J12 结项结论 (Final Verdict)

- **J12 阶段**: **CLOSED / FROZEN**
- **冻结 Tag**: `v1.0-j12-classical-dynamic-performance-frozen`
- **成果交付**:
  - 确立了 6PUS 机器人在大角度（J12-1）、组合姿态（J12-2）和高速手持动态（J12-3）下的完整物理特性与限制边界；
  - 统一了生产端 1500 mA 电流保护；
  - 为 J13 全面开展 ADRC 自抗扰控制提供了无可置疑的对比 Benchmark 与物理边界数据。
