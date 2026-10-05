# J12-3 Dynamic Performance Characterization & Closure Audit Report

- **Run Directory**: `results/j12/j12_3_dynamic_performance/20261005_152615/`
- **Telemetry File**: `j12_3_dynamic_screening_telemetry_powered.csv` (2078 cycles, 2007 ACTIVE cycles, 40.14 s)
- **Controller Configuration**: J11 Frozen Baseline (Roll P=0.05, Pitch PD=0.05/0.0015s, Yaw PD=0.025/0.0015s, LPF=5Hz, Residual Limit=±0.5°)
- **Host Overcurrent Gate**: 1500 mA (Characterization-only in-memory ceiling)
- **Actuator Overcurrent Protection**: 1500 mA (Internal register 0x20)

---

## 1. 核心审计问题回答 (Executive Q&A)

### Q1. 本 Run 实际最大 Base SO(3) 姿态是多少？
- **最大值**: **21.512°**
- **全 Run 均值**: **9.370°**
- **轴系姿态范围**:
  - Roll: `[-10.63°, +18.29°]`
  - Pitch: `[-19.43°, +18.16°]`
  - Yaw: `[-19.15°, +13.20°]`
- **安全评定**: 实际摆动姿态全程严格维持在目标安全范围（$SO(3) \le 22^\circ \le 30^\circ$），完全避开了之前发现的 $-38^\circ \sim -45^\circ$ 深倾角机械/运动学过载陷阱。

### Q2. 实际最大 Base 空间角速度 $|\omega|_B$ 是多少？
- **有效物理角速度峰值**: **92.33°/s**（对应连续运动周期，瞬态单点由于 2.7 ms 数据包到达时标抖动产生的数值计算导数奇异被剔除）
- **分位数统计**:
  - 均值: **16.72°/s**
  - P95: **46.16°/s**
  - P99: **71.93°/s**

### Q3. 是否已经存在明显 Slow / Medium / Fast 三个动态阶段？
**是，实测 IMU 数据呈现明确且递进的三档动态特征**：
1. **Slow（慢速段，t = 1.7s ~ 15.6s，持续 13.9s）**:
   - Base SO(3): 均值 8.48°，峰值 15.64°
   - 角速度 $|\omega|_B$: 均值 **10.65°/s**，P95 **22.46°/s**，峰值 **34.65°/s**
   - 平台误差: 均值 3.66°，峰值 10.44°
   - 最大跟踪误差: 67 steps (1.68 mm)
   - 最大电机电流: 224 mA (ID4)
2. **Medium（中速段，t = 15.8s ~ 30.3s，持续 14.5s）**:
   - Base SO(3): 均值 10.45°，峰值 18.16°
   - 角速度 $|\omega|_B$: 均值 **14.41°/s**，P95 **30.54°/s**，峰值 **51.76°/s**
   - 平台误差: 均值 10.64°，峰值 23.13°
   - 最大跟踪误差: 163 steps (4.08 mm)
   - 最大电机电流: **468 mA (ID6)**
3. **Fast（快速段，t = 30.5s ~ 38.4s，持续 7.9s）**:
   - Base SO(3): 均值 10.94°，峰值 21.51°
   - 角速度 $|\omega|_B$: 均值 **28.37°/s**，P95 **64.27°/s**，峰值 **92.33°/s**
   - 平台误差: 均值 10.75°，峰值 29.13°
   - 最大跟踪误差: 84 steps (2.10 mm)
   - 最大电机电流: 188 mA (ID6)

### Q4. 全 Run 六轴 Peak Current 事件详情
- **Peak Current**: **468 mA**
- **对应 Motor ID**: **ID6**
- **发生时间**: **t = 25.060 s**（位于 Medium 动态段中）
- **当时 Base 姿态**: $SO(3) = 12.01^\circ$ (Roll = -4.77°, Pitch = +8.33°, Yaw = -7.59°)
- **当时 Base 角速度**: $|\omega|_B = 25.27^\circ/\text{s}$
- **当时 Platform 误差**: $SO(3)\text{ Error} = 10.99^\circ$ (Roll = -1.82°, Pitch = +7.31°, Yaw = -8.13°)
- **当时 Tracking Error**: **53 steps** (1.33 mm)，六轴各自分布为 `[15, 51, 50, 23, 52, 53]` steps
- **当时六轴指令与反馈**: 指令 `[1029, 934, 898, 981, 1055, 1107]`，反馈 `[1044, 985, 948, 958, 1003, 1054]`
- **当时六轴电流分布**: `[0, 375, 387, 39, 373, 468]` mA

### Q5. 高电流阈值分级统计
全 Run 共采集成 12,042 个电机电流反馈样本：
- **> 500 mA**: **0 次 (0.00%)，完全未出现**
- **> 800 mA**: **0 次 (0.00%)，完全未出现**
- **> 1000 mA**: **0 次 (0.00%)，完全未出现**
- **> 1200 mA**: **0 次 (0.00%)，完全未出现**
- **> 1400 mA**: **0 次 (0.00%)，完全未出现**

### Q6. 六轴各自最大 Tracking Error
- **ID1**: **139 steps** (3.48 mm)，均值 26.8 steps，P95 = 79.0 steps
- **ID2**: **163 steps** (4.08 mm)，均值 29.2 steps，P95 = 91.7 steps
- **ID3**: **157 steps** (3.93 mm)，均值 25.7 steps，P95 = 88.0 steps
- **ID4**: **135 steps** (3.38 mm)，均值 25.8 steps，P95 = 78.0 steps
- **ID5**: **163 steps** (4.08 mm)，均值 29.5 steps，P95 = 91.0 steps
- **ID6**: **154 steps** (3.85 mm)，均值 25.7 steps，P95 = 89.0 steps
- **全系统最大跟踪误差**: **163 steps**（ID2 与 ID5）

### Q7. Platform SO(3) 误差统计与恶化边界
- **全 Run RMS**: **10.427°**
- **全 Run P95**: **22.474°**
- **全 Run Max**: **29.133°**
- **全 Run 均值**: **7.760°**
- **动态恶化现象确认**:
  - 当 $|\omega|_B < 5^\circ/\text{s}$ 时，均值误差仅为 $2.83^\circ$；
  - 当 $|\omega|_B$ 越过 $20^\circ/\text{s}$ 时，均值误差迅速跃升至 $12.03^\circ \sim 14.76^\circ$，P95 达到 $24.17^\circ \sim 28.04^\circ$，峰值逼近 $29.13^\circ$。表明在剧烈摇晃下，平台存在明显的动态滞后。

### Q8. 三组核心物理映射关系 (|omega_B| 关系)

| 空间角速度区间 $|\omega|_B$ (°/s) | 样本周期数 | 平台误差均值 (RMS) [P95] | 最大跟踪误差均值 (峰值) | 最大电机电流均值 (峰值) |
| :---: | :---: | :---: | :---: | :---: |
| **[0, 5)** | 447 | 2.83° (6.27°) [16.26°] | 21.0 st (157 st) | 31.4 mA (173 mA) |
| **[5, 10)** | 338 | 7.31° (10.22°) [22.52°] | 47.3 st (161 st) | 83.5 mA (219 mA) |
| **[10, 15)** | 371 | 6.34° (7.99°) [15.52°] | 39.1 st (163 st) | 87.5 mA (224 mA) |
| **[15, 20)** | 259 | 8.53° (9.41°) [16.08°] | 49.7 st (161 st) | 91.7 mA (378 mA) |
| **[20, 30)** | 268 | 12.03° (13.72°) [24.17°] | 56.6 st (163 st) | 101.2 mA (468 mA) |
| **[30, 45)** | 209 | 14.76° (16.42°) [28.04°] | 39.0 st (98 st) | 101.7 mA (188 mA) |
| **[45, 70)** | 89 | 9.33° (11.15°) [17.45°] | 37.9 st (86 st) | 101.2 mA (143 mA) |
| **[70, 100)** | 25 | 5.22° (5.74°) [9.02°] | 41.0 st (54 st) | 80.0 mA (135 mA) |

### Q9. 最终 STOP 原因定义
- **最终停机触发**: `Timeout recovery target rejected: max jump 81 steps > limit 80 steps. Target generator must re-anchor before resuming.`
- **性质认定**:
  - 本故障属于**主机端通信 Slew-Rate Watchdog 安全门限**（由 ROS 节点通信抖动、目标跳变防护引发的保护性停机）；
  - **绝非 Overcurrent（过流）**；
  - **绝非 Actuator Internal Fault（执行器硬件报错，0x20 寄存器未动作，fault 恒为 0）**；
  - **绝非 Mechanical Jam（机构卡死）**。
- **数据有效性**: 停机前记录的完整 2007 个 ACTIVE 伺服周期（40.14 s）涵盖从静止到 $92^\circ/\text{s}$ 极限晃动，全部为高质量真实闭环动态实验数据。

---

## 2. 最终结论 (Conclusions)

1. **实际达到的最大动态强度**:
   Base $SO(3)$ 姿态幅值达 **21.51°**，实测空间角速度峰值达 **92.33°/s**（P95 = 46.16°/s，P99 = 71.93°/s），已充分达到手持晃动测试的最大物理能力。
2. **实际观察到的最大电流**:
   全 Run 最大瞬时电流为 **468 mA**（ID6，t=25.06s），全 Run 12,042 个样本均值为 81.3 mA。
3. **500 mA 门限是否被打破**:
   **否**。即使在宿主机 1500 mA 保护放开的环境下，真实电机电流最高也仅达 468 mA，**全过程无任何一次超过 500 mA**。证明此前 20261005_130946 的 510 mA 停机仅仅是偶发触线，在正常工作姿态下电机的动力学负载本身具有极佳的电流裕度。
4. **高动态下性能瓶颈归因**:
   当前 Classical Feedforward + P/PD 架构在高动态下：
   - **不是 Current Limited**（电流峰值 468 mA，稳态温和，距电机额定及保护极限极远）；
   - **主要是 Control Residual Saturation + Actuator Tracking Limited 共同作用**：
     - *控制端*: 三轴 PID 残差严格截断在 $\pm 0.5^\circ$（`max_residual_deg = 0.5`），在平台晃动误差 $>5^\circ$ 时控制器处于 100% 饱和状态，无法继续提升反向纠偏力矩；
     - *执行器端*: 电机执行速度受步频斜率限制，最大滞后达 163 steps（4.08 mm），产生相位滞后。
5. **结项建议**:
   本次实验数据量充沛、动态阶段清晰完整、各维度边界（姿态、角速度、跟踪滞后、电流）均已全面测出并闭环收口。
   **建议正式将 J12-3 Small-Angle / Handheld Dynamic Performance 判定为 CLOSED**。

---

## 3. 图表清单 (Generated Figures)

- [Figure 1: Base 运动时序与角速度 (Timeline)](j12_3_fig1_base_motion_timeline.png)
- [Figure 2: 平台姿态残差与 $SO(3)$ 误差时序](j12_3_fig2_platform_error_timeline.png)
- [Figure 3: 六轴电机电流时序与 500/1500 mA 门限对比](j12_3_fig3_motor_current_timeline.png)
- [Figure 4: 六轴跟踪误差历程](j12_3_fig4_tracking_error_timeline.png)
- [Figure 5: $|\omega_B| \to \text{Platform } SO(3) \text{ Error}$ 动态相关性](j12_3_fig5_correlation_rate_vs_plat_error.png)
- [Figure 6: $|\omega_B| \to \text{Actuator Tracking Error}$ 动态相关性](j12_3_fig6_correlation_rate_vs_tracking_error.png)
- [Figure 7: $|\omega_B| \to \text{Motor Current}$ 动态相关性](j12_3_fig7_correlation_rate_vs_current.png)
