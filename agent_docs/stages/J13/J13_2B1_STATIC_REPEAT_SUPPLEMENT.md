# J13-2B1 静态重复性补充分析报告 (J13-2B1 Static Repeat Supplement)

- **阶段归属**: **J13-2B1 静态验收补充证据 (Static Repeat Supplement)**
- **前序冻结基线**: J11 (`v1.0-j11-attitude-stabilization-frozen`), J12 (`v1.0-j12-classical-dynamic-performance-frozen`), J13-2B1 (`05fd7b3`)
- **阶段状态**: **COMPLETED**
- **真实硬件数据集路径**: `/home/lyapunov/bone_drill/results/j13/j13_2b2_roll_active_gentle/20261006_114437/`
  > **数据溯源与物理事实勘误声明**:
  > 尽管存储目录与 Stage 名称中包含 `gentle`，但在真实物理实验过程中，**操作人员全程未对 Base（定平台）施加任何人为晃动或物理位移激励**。
  > 真实物理环境全程为 **BASE STATIC**。Base IMU 记录到的微小角位移（RMS 0.007° ~ 0.021°）与角速度（RMS 0.034°/s ~ 0.036°/s）全部属于传感器零漂、环境微振动、导线张力等静态微扰动本底，**绝非受控的人工基座扰动**。
  > 因此，本批次数据集正式重新分类为 **J13-2B1 Static Repeat Supplement**，严禁解释为动态抗扰数据。

---

## 1. 数据集重分类映射

| 原始执行标记 | 原始目录路径 | 正式物理分类 | 控制权配置 | 有效闭环周期 / 时长 | 硬件故障码 |
| :--- | :--- | :--- | :--- | :---: | :---: |
| **Run A** | `classical_reference/` | **Classical Static Repeat** | Roll/Pitch/Yaw 均为 Classical P/PD | 1,679 周期 (33.58s) | 0 |
| **Run B** | `ladrc_trial_1/` | **Roll LADRC Active Static Repeat 1** | Roll LADRC Active, Pitch/Yaw Classical | 1,696 周期 (33.92s) | 0 |
| **Run C** | `ladrc_trial_2/` | **Roll LADRC Active Static Repeat 2** | Roll LADRC Active, Pitch/Yaw Classical | 1,663 周期 (33.26s) | 0 |

---

## 2. 静态重复性核心遥测指标汇总

统计分析结果归档于 [`results/j13/j13_2b2_roll_active_gentle/20261006_114437/analysis/j13_2b2_comparative_metrics.json`](file:///home/lyapunov/bone_drill/results/j13/j13_2b2_roll_active_gentle/20261006_114437/analysis/j13_2b2_comparative_metrics.json)：

| 统计指标分类 | 指标项目 | Run A: Classical Static Repeat | Run B: LADRC Static Repeat 1 | Run C: LADRC Static Repeat 2 | 物理特性与评估说明 |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **基座静态本底**<br>(Base Static Baseline) | **Base Roll RMS** | **0.0071°** | **0.0080°** | **0.0208°** | 静态传感器温漂与微扰动本底 |
| | Base Roll P95 | 0.0150° | 0.0146° | 0.0288° | 静态微漂移量级一致 |
| | Base Roll Peak | 0.0213° | 0.0193° | 0.0340° | 极微小静态极值 |
| | Base Roll Rate RMS | 0.0342°/s | 0.0360°/s | 0.0352°/s | 陀螺仪量化与热本底噪声 |
| | Base Roll Rate P95 | 0.0694°/s | 0.0683°/s | 0.0695°/s | 三组陀螺仪本底完全重合 |
| | Base SO(3) 综合幅值 RMS | 0.0223° | 0.0120° | 0.0281° | 无人为激励 |
| **平台 Roll 跟踪误差**<br>(Platform Roll Error) | **Roll Error RMS** | **0.0056°** | **0.0061°** | **0.0114°** | **多次静态运行均稳定有界** |
| | **Roll Error P95** | 0.0117° | 0.0132° | 0.0186° | 均在 $0.01^\circ \sim 0.02^\circ$ 范围内 |
| | **Roll Error Peak** | 0.0164° | 0.0200° | 0.0208° | 最大峰值均 $\le 0.021^\circ$ |
| | **Roll Error MAE** | 0.0045° | 0.0046° | 0.0103° | Repeat 1 与 Classical 几乎完全相同 |
| **平台全自由度误差**<br>(Platform Total SO(3))| **Total SO(3) RMS** | **0.0135°** | **0.0133°** | **0.0453°** | 全局静态姿态保持稳定 |
| | Total SO(3) P95 | 0.0203° | 0.0218° | 0.0641° | 正常静态离散带 |
| | Total SO(3) Peak | 0.0247° | 0.0271° | 0.0679° | 安全有界 |
| **交叉轴性能** | Pitch 误差 RMS | 0.0079° | 0.0064° | 0.0172° | 处于正常传感器噪声水平 |
| | Yaw 误差 RMS | 0.0093° | 0.0100° | 0.0402° | 保持 Classical 控制，无异常干扰 |
| **LESO 状态观测特性** | $z_1 - e_{roll}$ RMSE | 0.0011° | 0.0012° | 0.0018° | 状态估计高度收敛 |
| | $z_2$ 等效总扰动均值 | +0.0006°/s | +0.0446°/s | +0.1078°/s | Equivalent Lumped Disturbance |
| | $z_2$ 等效总扰动极值 | [-0.015, +0.015]°/s | [-0.043, +0.186]°/s | [-0.117, +0.404]°/s | 平滑有界，无振荡 |
| | $z_2$ 变化率标准差 ($\sigma_{\Delta z2}$)| 0.00039°/s | 0.00038°/s | 0.00057°/s | 极度平滑，无高频抖颤 |
| **控制指令与饱和度** | 接管第一拍跳变 | 0.000019° | 0.000022° | 0.000024° | 完全无扰平滑接管 ($\ll 0.05^\circ$) |
| | Roll 控制指令 RMS | 0.0003° | 0.0089° | 0.0200° | 平稳输出 |
| | Roll 控制指令 Peak | 0.0008° | 0.0190° | 0.0417° | 远低于 $\pm 0.5^\circ$ 限幅 |
| | **控制量饱和率 (Saturation)** | **0.0%** | **0.0%** | **0.0%** | **零饱和** |
| **机构伺服与硬件健康** | 最大关节跟踪误差 | 2 steps (0.05mm) | 2 steps (0.05mm) | 3 steps (0.075mm) | 伺服跟踪高保真 (P95=2 steps) |
| | 六轴电机峰值电流 | 0 mA | 0 mA | 38 mA | 远低于 1500 mA 安全硬门限 |
| | 控制周期 $dt$ 均值 / 最大值 | 20.00ms / 37.68ms | 20.00ms / 37.11ms | 20.00ms / 27.89ms | 确定性 50Hz 伺服周期 |
| | **硬件故障码 (Fault Codes)** | **0** | **0** | **0** | **零故障，系统 100% 健康安全** |

---

## 3. 科学解释与工程定性结论

1. **静态重复性定性 (Static Repeatability Evaluation)**:
   - Roll LADRC 在连续两次独立的物理接管运行（Repeat 1 与 Repeat 2，累积 3,359 闭环伺服周期，67.2 秒物理运行）中，展现出良好的数值稳定性、状态收敛性与硬件安全性。
   - 姿态跟踪误差 RMS 维持在 $0.0061^\circ \sim 0.0114^\circ$，峰值误差稳定在 $0.0200^\circ \sim 0.0208^\circ$。
2. **客观性能评估 (不支持“LADRC 静态显著优于 Classical”)**:
   - Classical Static Repeat 的 Roll 误差 RMS 为 $0.0056^\circ$；
   - LADRC Static Repeat 1 为 $0.0061^\circ$（与 Classical 相当，略高 $0.0005^\circ$）；
   - LADRC Static Repeat 2 为 $0.0114^\circ$（高于 Classical 约 $0.0058^\circ$）；
   - **科学定论**: 当前数据表明，**Roll LADRC Active 在多次静态运行中保持稳定、有界、安全，但当前静态重复数据不支持“LADRC 静态精度显著优于 Classical”这一结论**。
3. **保持基线与参数冻结**:
   - 鉴于静态误差差异处于微度量级（$< 0.01^\circ$），且机构未发生任何发散或振荡，**严格不重新调参**，保持 $b_0=10.0	ext{ s}^{-1}, \omega_c=1.0	ext{ rad/s}, \omega_o=4.0	ext{ rad/s}$ 冻结不变；
   - 本静态重复数据作为 J13-2B1 的充分补充支撑证据，**不否定 J13-2B1 PASS 结论**。

---

## 4. 阶段状态修正

- **J13-2B1: Roll Single-Axis LADRC Active Static Qualification = PASS**
- **J13-2B1 Static Repeat Supplement = COMPLETED**
- **J13-2B2: Roll LADRC Active Gentle Base-Disturbance Qualification = NOT YET EXECUTED (PENDING REAL BASE EXCITATION)**
  - 原因：尚未对基座施加真实的人工 Roll 扰动。待操作人员到位并实际施加定平台扰动后，再行正式执行。
- **当前动作**: 已停止所有硬件运行，严禁自动执行 Pitch Active 或新测试。
