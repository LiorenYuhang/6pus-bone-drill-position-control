# J13 动平台偏心负载工况对比分析报告 (Classical vs 3-axis LADRC)

## 1. 实验概述与数据边界

本报告对 6PUS 并联骨科手术机器人平台在偏心负载工况下的两组 10 s 真机试验数据进行技术审计与客观对比分析。

### 1.1 实验对象界定
- **A 组（Classical Controller）**：  
  控制器架构：$\text{Base SO(3) Feedforward} + \text{Roll P} + \text{Pitch PD} + \text{Yaw PD}$  
  原始运行标识：`j13_3axis_payload_classical_10s`（采集时间：2026-10-07 14:30:07）
- **B 组（3-axis LADRC）**：  
  控制器架构：$\text{Base SO(3) Feedforward} + \text{3-axis LADRC Residual Control}$  
  原始运行标识：`j13_3axis_payload_ladrc_10s`（采集时间：2026-10-07 14:30:36）

### 1.2 实验物理边界与约束（重要事实声明）
1. **偏心负载状态**：  
   本次实验在动平台上安装了偏心重物，但**该重物的具体质量未称重测量、重心到动平台几何中心的实际力臂距离未精确测量、安装方向未进行仪器标定**。两次实验在物理装夹未改动的情况下连续进行，属于**“同一未知质量、同一固定位置的偏心重物（same fixed eccentric payload condition）”**。
2. **基座激励特性**：  
   基座（Base）扰动来自人工施加的外力摇晃，两次实验的激励**属于工况相似但波形非严格对齐、幅度具有显著随机差异的外力输入**。
3. **等效集总扰动项（Equivalent Lumped Disturbance, $z_2$）**：  
   LESO 输出的扩张状态 $z_2$ 为**等效集总扰动项**（工程量纲为 $\text{deg/s}$ 或 $\text{rad/s}$），包含了未建模并联机构动力学、运动学耦合、偏心重力效应及外界晃动等效影响的总和，**严禁将其解读为真实力矩、真实扰动力矩、物理扭矩或扰动变化率**。
4. **对比评价边界**：  
   鉴于基座输入激励强度的客观差异，**本报告不作“LADRC 全面优于 Classical”之推论**，仅忠实记录各轴测得数据、控制量行为与工程状态。

---

## 2. 实验配置与控制器参数

### 2.1 基础实验信息
- **采样频率**：标称 50 Hz（名义周期 20.0 ms）
- **总采样数**：500 周期（实际物理时长 9.98 s）
- **Classical 组接管时刻**：第 72 周期（$t = 1.42\,\text{s}$），包含 429 个 active 控制输出周期；接管瞬间指令跳变 $< 5.54 \times 10^{-5}$ deg；
- **3-axis LADRC 组接管时刻**：第 55 周期（$t = 1.08\,\text{s}$），包含 446 个 active 控制输出周期；接管瞬间指令跳变 $< 3.18 \times 10^{-6}$ deg；
- **故障统计**：两组实验均实现 0 driver STOP、0 motor fault、0 recovery reject、0 scheduler deadline exceeded，全流程平稳完成。

### 2.2 控制器参数表
详细 CSV 见 `../03_tables/controller_parameters.csv`。

| 控制器组别 | 通道 | 控制器类型 | 关键增益参数 | 前馈配置 | 残差限幅 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Classical** | Roll | P | $K_p = 0.05$ | Base SO(3) 启用 | $\pm 0.5^\circ$ |
| **Classical** | Pitch | PD | $K_p = 0.05,\; K_d = 0.0015$ | Base SO(3) 启用 | $\pm 0.5^\circ$ |
| **Classical** | Yaw | PD | $K_p = 0.025,\; K_d = 0.0015$ | Base SO(3) 启用 | $\pm 0.5^\circ$ |
| **3-axis LADRC** | Roll | 一阶线性 ADRC | $b_0 = 20.0,\; \omega_c = 0.8\,\text{rad/s},\; \omega_o = 2.5\,\text{rad/s}$ | Base SO(3) 启用 | $\pm 0.5^\circ$ |
| **3-axis LADRC** | Pitch | 一阶线性 ADRC | $b_0 = 10.0,\; \omega_c = 1.0\,\text{rad/s},\; \omega_o = 4.0\,\text{rad/s}$ | Base SO(3) 启用 | $\pm 0.5^\circ$ |
| **3-axis LADRC** | Yaw | 一阶线性 ADRC | $b_0 = 6.0,\; \omega_c = 0.25\,\text{rad/s},\; \omega_o = 1.2\,\text{rad/s}$ | Base SO(3) 启用 | $\pm 0.5^\circ$ |

---

## 3. 统计指标与对比分析

### 3.1 Base 输入激励可比性
详细 CSV 见 `../03_tables/base_excitation_metrics.csv`；对应图件：`../02_figures/fig01_base_roll_comparison.png` 至 `fig03_base_yaw_comparison.png`。

| 激励通道 | Classical 组 (A) | 3-axis LADRC 组 (B) | 差异比例 (ΔRMS) | 客观记录 |
| :--- | :--- | :--- | :--- | :--- |
| **Base Roll** | RMS = 1.02°, P95 = 2.07°, Peak = 2.29° | RMS = 1.94°, P95 = 4.06°, Peak = 4.57° | **+90.2%** | Base Roll RMS amplitude 约为 Classical 组的 1.9 倍 |
| **Base Pitch**| RMS = 2.41°, P95 = 5.72°, Peak = 6.43° | RMS = 3.61°, P95 = 6.63°, Peak = 6.93° | **+49.8%** | Base Pitch RMS amplitude 约为 Classical 组的 1.5 倍 |
| **Base Yaw**  | RMS = 1.74°, P95 = 3.14°, Peak = 3.18° | RMS = 0.91°, P95 = 2.21°, Peak = 2.43° | **-47.7%** | Base Yaw RMS amplitude 约为 Classical 组的 0.52 倍 |

### 3.2 动平台姿态跟踪误差对比（ACTIVE 阶段）
详细 CSV 见 `../03_tables/platform_error_metrics.csv`；对应图件：`../02_figures/fig04_roll_error_comparison.png` 至 `fig06_yaw_error_comparison.png`。

| 误差通道 | Classical 组 | 3-axis LADRC 组 | ΔRMS | 数据观察与事实归因 |
| :--- | :--- | :--- | :--- | :--- |
| **Roll Error** | RMS = 0.45°, MAE = 0.32°, P95 = 0.90°, Peak = 1.17° | RMS = 1.07°, MAE = 0.77°, P95 = 2.14°, Peak = 3.16° | +140.4% | LADRC 组 Base Roll RMS amplitude 约为 Classical 的 1.9 倍，且控制量未饱和 |
| **Pitch Error**| RMS = 0.87°, MAE = 0.59°, P95 = 1.88°, Peak = 2.39° | RMS = 1.31°, MAE = 0.86°, P95 = 3.05°, Peak = 4.19° | +50.8% | **Pitch LADRC 出现 16.4% 的 $\pm 0.5^\circ$ 输出饱和削顶**，致使扰动未完全消除 |
| **Yaw Error**  | RMS = 0.61°, MAE = 0.33°, P95 = 1.46°, Peak = 2.76° | RMS = 0.44°, MAE = 0.29°, P95 = 1.01°, Peak = 1.36° | **-27.6%** | **本次 LADRC run 的 Yaw error RMS/P95/Peak 均低于 Classical run，但 LADRC run 的 Base Yaw excitation 同时明显更低，因此当前非同源人工扰动数据不能将该差异单独归因于控制器。** |

### 3.3 控制输出与观测器行为
详细 CSV 见 `../03_tables/control_output_metrics.csv` 与 `ladrc_observer_metrics.csv`；对应图件：`../02_figures/fig07_classical_control_output.png` 至 `fig09_ladrc_z2_states.png`。

- **Classical 组控制输出**：三轴残差补偿输出峰值仅 $0.129^\circ$，均处于 $\pm 0.5^\circ$ 限幅以内，饱和率为 0.0%；
- **LADRC 组控制输出**：
  - Roll：RMS = $0.093^\circ$，Peak = $0.228^\circ$，饱和率 = 0.0%；
  - Pitch：RMS = $0.277^\circ$，Peak = **$0.500^\circ$**，**饱和率 = 16.4%**；
  - Yaw：RMS = $0.038^\circ$，Peak = $0.113^\circ$，饱和率 = 0.0%；
- **LESO 观测器状态**：
  - Roll：$z_1$ Peak = $2.131^\circ$；$z_2$ Peak = $3.141^\circ/\text{s}$（$0.0548\,\text{rad/s}$）；
  - Pitch：$z_1$ Peak = $3.333^\circ$；$z_2$ Peak = **$6.920^\circ/\text{s}$（$0.1208\,\text{rad/s}$）**；
  - Yaw：$z_1$ Peak = $0.730^\circ$；$z_2$ Peak = $0.494^\circ/\text{s}$（$0.0086\,\text{rad/s}$）。

---

## 4. 偏心负载专项现象观察

### 4.1 初始静态阶段观察（$t = 0 \sim 1.5\,\text{s}$，见 `fig10_initial_static_bias_zoom.png`）
在基座尚未发生动态摇晃的初始静态区间：
- **Roll 轴**：误差均值为 $+0.020^\circ \sim +0.023^\circ$（峰值约 $+0.059^\circ$）；
- **Pitch 轴**：误差均值为 $+0.001^\circ \sim +0.005^\circ$（峰值 $< 0.010^\circ$）；
- **Yaw 轴**：误差均值为 $+0.004^\circ \sim +0.011^\circ$（峰值 $< 0.024^\circ$）。
- *物理观察*：静态重力下垂主要集中反映在 Roll 轴，呈现微小的初始角度偏差，接管过程两组控制器均平稳过渡。

### 4.2 动态扰动阶段与限幅饱和现象
- **Pitch 轴限幅饱和机制**：  
  Pitch 通道的 LESO $z_2$ 峰值达到 $6.92^\circ/\text{s}$（$0.1208\,\text{rad/s}$）。在名义增益 $b_0 = 10.0$ 下，峰值扰动对应的**扰动补偿分量幅度（disturbance-compensation component magnitude）**达到：
  $$\frac{z_{2,\text{peak}}}{b_0} = \frac{0.1208\,\text{rad/s}}{10.0\,\text{s}^{-1}} = 0.01208\,\text{rad} \approx 0.692^\circ$$
  单仅此扰动补偿分量即已超过 $\pm 0.50^\circ$ 的工程残差限幅；当结合状态误差反馈项计算完整的 $u_{\text{raw}} = \frac{\omega_c z_1 + z_2}{b_0}$ 时，控制器在多处波峰发生削顶饱和（饱和率 16.4%），扰动未获完全对消，导致姿态误差在饱和区间累积。
- **Yaw 轴线性区表现**：  
  Yaw 轴未发生任何限幅饱和，LESO $z_2$ 维持在 $0.5^\circ/\text{s}$ 以内的温和区间。本次 LADRC run 的 Yaw 误差统计更低，但其 Base Yaw 输入激励本身亦较平缓，无法将该表现单独归因于控制器。

---

## 5. 系统底层健康度总结

详细 CSV 见 `../03_tables/runtime_health_metrics.csv`。
- **电机跟踪误差**：Classical 最大 38 steps，LADRC 最大 80 steps（当前生产代码未配置动态步数安全上限，处于安全观测范围内）；
- **电机相电流**：Classical 均值 18.5 mA（峰值 143 mA），LADRC 均值 22.7 mA（峰值 555 mA），均远低于驱动器 1500 mA 硬件保护阈值（`pus6_motor_driver/safety.py: MAX_CURRENT_MA = 1500`）；
- **通信与调度**：数据最大龄期及调度间隔均在 380 ms 以内，UART TX 与 Drain 均在单周期内受控闭环，系统全时无死锁、无超时报错、无驱动器 STOP 触发。
