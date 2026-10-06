# J13-2B1: Roll 单轴 LADRC 物理接管静态验收总结 (Roll Single-Axis LADRC Active Takeover — Static Qualification)

- **阶段代号**: **J13-2B1**
- **前序冻结基线**: J11 (`v1.0-j11-attitude-stabilization-frozen`), J12 (`v1.0-j12-classical-dynamic-performance-frozen`), J13-2A (`d8a769c`)
- **阶段状态**: **QUALIFIED / CLOSED**
- **真实硬件数据集路径**: `/home/lyapunov/bone_drill/results/j13/j13_2b1_roll_active/20261006_104124/`
  - 15s 静态运行: `static_15s/j13_2b1_static_15s_roll_active_telemetry_powered.csv`
  - 30s 静态运行: `static_30s/j13_2b1_static_30s_roll_active_telemetry_powered.csv`

---

## 1. 阶段目标与物理控制权定义

J13-2B1 是自抗扰控制（ADRC）在并联机构上**首次获得物理控制权**的里程碑阶段。
为保证绝对安全，本阶段执行极度受限的单轴静态验收契约：

1. **控制权受限接管**: 仅 Roll 轴姿态残差切换为 LADRC 闭环指令（`roll_active` 模式），Pitch 和 Yaw 轴 100% 保持 Classical P/PD 物理控制；
2. **闭环外环架构冻结**:
   - Base-IMU 全 $SO(3)$ 旋转矩阵前馈 $\mathbf{R}_{BP\_FF} = \mathbf{R}_{WB}^T$ 100% 保持；
   - 任务空间姿态误差定义、逆运动学（IK）、轨迹整形器（Trajectory Shaper）及底层 J8 伺服驱动 100% 保持；
3. **LESO 闭环输入信号严格切换**:
   - Roll 轴 LESO 观测器输入切换为 Roll 轴实际施加的 LADRC 前一拍控制量：$u_{plant\_roll, k-1} = u_{ladrc\_roll\_actual, k-1}$；
   - Pitch / Yaw 轴 LESO 输入继续使用 Classical 前一拍控制量：$u_{plant\_pitch, k-1} = u_{classical\_pitch\_actual, k-1}$，$u_{plant\_yaw, k-1} = u_{classical\_yaw\_actual, k-1}$；
4. **参数严格冻结**:
   - Roll/Pitch: $b_0 = 10.0\text{ s}^{-1}, \omega_c = 1.0\text{ rad/s}, \omega_o = 4.0\text{ rad/s}, u_{limit} = \pm 0.5^\circ$；
   - Yaw: $b_0 = 3.0\text{ s}^{-1}, \omega_c = 0.35\text{ rad/s}, \omega_o = 1.8\text{ rad/s}, u_{limit} = \pm 0.5^\circ$；
   - 控制频率: $50\text{ Hz}$ ($T_s = 0.02\text{ s}$)。

---

## 2. 软件实现与单元测试 (25/25 PASS)

在 `src/h30_imu/scripts/j13_ladrc_shadow_runner.py` 与 `j13_ladrc_staged_launcher.py` 中实现了：
1. **多模式支持**: `--mode shadow | roll_active`；
2. **无扰切换门禁 (Bumpless Transfer Gate)**: 在进入 `ACTIVE` 状态的第一拍，测量 $|u_{ladrc\_roll} - u_{classical\_roll}|$。若跳变超过安全阈值（$0.05^\circ$），立即安全停机；
3. **故障安全降级机制**: 若 Roll LADRC 出现非有限值（NaN/Inf）或异常，立即安全降级至 Classical Roll 并触发安全停机；
4. **单元测试**: 25 组单元测试（包含 5 组针对 `roll_active` 路由、输入切换、无扰门禁、降级隔离的针对性测试）全部通过。

---

## 3. 实机测试执行与数据分析

在真实物理样机上，先执行 5s Dry-run 预检通过，随后在 `OPERATING_HOME` 执行了 15s 与 30s 两次带电真实运动测试（`POWERED REAL EXECUTION`）：

### 3.1 核心遥测指标汇总对比

| 指标项目 | J13-2A2 静态影子基线<br>(Classical 全权控制) | J13-2B1 15s 静态接管<br>(Roll LADRC Active) | J13-2B1 30s 静态接管<br>(Roll LADRC Active) | 评定与结论 |
| :--- | :---: | :---: | :---: | :--- |
| **有效闭环周期 (ACTIVE)** | 1,445 cycles (28.9s) | 696 cycles (13.92s) | 1,446 cycles (28.92s) | 完整执行，无提前中断 |
| **硬件写入次数 (Write Count)** | 1,445 | 696 | 1,446 | 100% 闭环下发 |
| **接管瞬间跳变 (Command Jump)** | N/A (未接管) | **0.000016°** ($2.81\times 10^{-7}$ rad) | **0.000045°** ($7.79\times 10^{-7}$ rad) | **完全无扰平滑接管** ($\ll 0.05^\circ$) |
| **Roll 姿态误差 Peak** | 0.0209° | 0.0334° | **0.0202°** | 30s 低于基线，15s 高于基线 |
| **Roll 姿态误差 Mean** | 0.0103° | 0.0166° | **0.0067°** | 30s 低于基线，15s 高于基线 |
| **Roll 姿态误差 RMS** | 0.0113° | 0.0182° | **0.0084°** | 30s 运行低，15s 运行高；缺少 matched trials，不作统计性性能提升断言 |
| **Roll 姿态误差 P95** | 0.0176° | 0.0285° | **0.0163°** | 30s 低于基线，15s 高于基线 |
| **Pitch 姿态误差 RMS** | 0.0078° | 0.0105° | 0.0178° | 维持在正常传感器本底噪声带 |
| **Yaw 姿态误差 RMS** | 0.0102° | 0.0126° | 0.0057° | 维持在正常传感器本底噪声带 |
| **LESO $z_1-e_{roll}$ 观测 RMSE** | 0.0010° | 0.0025° | 0.0014° | 状态估计高度收敛 |
| **LESO $z_2$ 等效总扰动均值** | +0.0052°/s | -0.1638°/s | +0.0881°/s | 平滑有界，无发散或高频抖振 (Residual Equivalent Lumped Disturbance) |
| **LESO $z_2$ 等效总扰动极值** | [-0.013, +0.019]°/s | [-0.294, +0.000]°/s | [-0.026, +0.284]°/s | 有界估计物理稳态扰动 |
| **六轴电机峰值电流** | 10 mA | **38 mA** | **54 mA** | 远低于 1500 mA 安全硬门限 |
| **六轴电机均值电流** | 0.0 mA | 0.13 mA | 0.08 mA | 机构近乎零功耗稳态平衡 |
| **最大关节跟踪误差** | 1 step | **2 steps** (0.05 mm) | **2 steps** (0.05 mm) | 伺服跟踪高保真 |
| **控制量饱和率 (Saturation)** | 0.0% | **0.0%** | **0.0%** | $|u_{roll}| < 0.035^\circ \ll 0.5^\circ$ |
| **硬件故障码 (Fault Codes)** | 0 | **0** | **0** | 零过流、零失联、零硬件错误 |

---

## 4. 关键科学与工程结论

1. **接管瞬态特性 (Bumpless Takeover)**:
   - 接管瞬间第一拍的命令跳变仅为 $0.000016^\circ \sim 0.000045^\circ$，完全处于 IMU 量化噪声量级以内；
   - 平台姿态无任何冲击跳变，电机电流无脉冲突刺，实现了完美的无扰物理接管。
2. **稳态精度表现**:
   - Roll LADRC 已完成稳定无扰物理接管。30 s Active run 的 Roll RMS 低于所引用 Classical static baseline，但 15 s run 高于该 baseline。由于目前缺少 matched repeated trials，暂不把差异解释为 statistically qualified LADRC performance improvement。
   - 证实了一阶 LADRC 与二阶 LESO 在物理闭环中能够稳定运行，未引入不稳定动态或发散。
3. **LESO 扰动观测特性**:
   - $z_1$ 紧密跟踪真实误差 $e_{roll}$，二者差值 RMSE 仅 $0.0014^\circ$；
   - $z_2$（等效总扰动，单位 $\text{rad/s}$ 或 $^\circ/\text{s}$）在静态下平滑收敛于 $[-0.03^\circ/\text{s}, +0.28^\circ/\text{s}]$ 区间，未出现任何数值振荡或高频抖颤。
4. **轴间解耦与健康安全性**:
   - Pitch 和 Yaw 轴 100% 保持 Classical P/PD 控制，误差保持在 $0.005^\circ \sim 0.018^\circ$ 正常范围内，未观察到导致系统失稳或异常的 cross-axis degradation；
   - 电机峰值电流仅 54 mA，关节跟踪误差 $\le 2$ steps，机构保持极高健康余量。

---

## 5. 准入与停步判定

- **J13-2B1 Roll Single-Axis LADRC Active Takeover — Static Qualification = PASS**
- **当前步骤结束，严格停步**: 不自动进入 J13-2B2（gentle disturbance），等待下一步明确指令。
