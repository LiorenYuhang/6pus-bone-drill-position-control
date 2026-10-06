# J13-2A: LADRC 影子模式实机资格认证总结 (LADRC Shadow Mode Real-Hardware Qualification)

- **阶段代号**: **J13-2A (J13-2A1 + J13-2A2)**
- **前序冻结基线**: J11 (`v1.0-j11-attitude-stabilization-frozen`), J12 (`v1.0-j12-classical-dynamic-performance-frozen`)
- **阶段状态**: **QUALIFIED / CLOSED**
- **Canonical 数据集路径**: `/home/lyapunov/bone_drill/results/j13/j13_2a2_shadow/20261005_205126/`

---

## 1. 阶段目标与执行边界

J13-2A 作为 J13 自抗扰控制（ADRC）首次进入真实 ROS 2 工程环境的阶段，其唯一核心目标为：
**在 50Hz 实时控制循环中，以影子模式（Shadow Mode）并行运行 3 轴离散线性自抗扰控制器（LADRC + LESO），严禁对真实机械机构施加任何物理控制权。**

### 严格边界与不变式
1. **真实控制权保持不变**: 真实六自由度并联机构的姿态残差命令 100% 由冻结的 J11/J12 经典 P/PD 控制器生成；
2. **前馈算子与运动学不变**: Base-IMU 全 $SO(3)$ 旋转矩阵前馈 $\mathbf{R}_{BP\_FF} = \Delta \mathbf{R}_B^T$、误差定义、逆运动学（IK）、轨迹整形器（Trajectory Shaper）及底层 J8 伺服驱动完全继承；
3. **零硬件写入泄漏**: LADRC 仅读取真实状态与前一拍被控对象实际输入，内部运行二阶 LESO 与一阶控制律，其输出仅写入遥测数据流，严禁送入合成姿态或底层电机；
4. **冻结基准零修改**: J8、J11、J12 所有历史冻结文件保持 0 行代码修改。

---

## 2. 核心技术实现 (J13-2A1)

在 `bone_ws/src/h30_imu/` 中新增独立模块，并通过单元测试（20/20 PASS）：

1. **离散 LADRC 核心 (`scripts/j13_ladrc.py`)**:
   - **离散二阶 LESO**: 基于零阶保持（Exact ZOH）显式矩阵离散化，观测状态 $z_1 \approx e$（跟踪误差）与 $z_2$（等效总扰动 Equivalent Lumped Disturbance，单位 $\text{rad/s}$）；
   - **控制律**: 比例反馈与总扰动补偿 $u_0 = k_p (r - z_1) - \frac{z_2}{b_0}$；
   - **安全限幅**: 严格限制于 $\pm 0.5^\circ$（与经典基线一致）；
   - **输入映射**: 支持显式传入前一拍真实被控对象输入 $u_{plant, k-1}$，确保影子模式下观测器闭环与实际物理激励一致。
2. **影子模式调度器 (`scripts/j13_ladrc_shadow_runner.py`)**:
   - 继承 `j9_1_3dof_orientation_servo.py` 完整生产架构与两级时效安全策略；
   - 50Hz 同步主循环中：在计算完经典 P/PD 残差并发送给 IK 之后，将同一状态观测与残差输入传入 3 轴独立 LADRC；
   - 完整记录状态遥测：`ladrc_u_roll/pitch/yaw`, `ladrc_z1_*`, `ladrc_z2_*`, `diff_u_*`, `sign_agreement_*` 等。
3. **参数配置与测试 (`config/j13_ladrc_params.yaml`, `test/`)**:
   - 冻结初值：Roll/Pitch 通道 $b_0 = 10.0\text{ s}^{-1}, \omega_c = 1.0\text{ rad/s}, \omega_o = 4.0\text{ rad/s}$；Yaw 通道 $b_0 = 3.0\text{ s}^{-1}, \omega_c = 0.35\text{ rad/s}, \omega_o = 1.8\text{ rad/s}$。

---

## 3. 真实硬件实测证据 (J13-2A2)

实机运行于 Ubuntu 24.04 NUC + 真实 6PUS 样机 + 双 H30 IMU，完成了 4 组真实工况实机闭环遥测（Preflight Check 1~10 均 100% 通过）：

| 工况名称 | 数据集相对子目录 | 有效闭环周期 (ACTIVE) | 运行物理时长 | 峰值电流 | 最大关节误差 | 饱和率 |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **Static** (静态基线) | `static/` | 1,445 cycles | 28.90 s | 10 mA | 1 step | 0.0% |
| **Roll/Pitch Gentle** | `roll_pitch_gentle/` | 2,195 cycles | 43.90 s | 61 mA | 3 steps | 0.0% |
| **Yaw Gentle** | `yaw_gentle/` | 2,180 cycles | 43.60 s | 35 mA | 2 steps | 0.0% |
| **3DOF Gentle** | `three_axis_gentle/` | 2,942 cycles | 58.84 s | 48 mA | 2 steps | 0.0% |
| **合计** | - | **8,762 cycles** | **175.24 s** | **61 mA** | **3 steps** | **0.0%** |

### 遥测分析关键发现
1. **无硬件或通信故障**: 8,762 个周期内，底层过流 = 0 次，通信超时 = 0 次，硬件故障码 = 0，电机实际电流最高仅 61 mA（远低于 1500 mA 安全硬门限）；
2. **状态观测器完全收敛**:
   - $z_1$ 严格收敛并跟踪误差 $e_{R\_W}$，静态工况三轴 RMSE 仅约 $0.0010^\circ$；
   - $z_2$（等效总扰动）保持有界平滑，全动态过程静态峰值 $<0.02^\circ/\text{s}$，动态峰值 $<0.04^\circ/\text{s}$，无任何数值抖振（chatter）或高频震荡；
3. **零控制权泄漏**: 全程由 Classical P/PD 控制物理平台，LADRC 命令仅记录于 CSV 遥测，未对执行器造成任何扰动。

---

## 4. 表述边界与科学定义矫正 (Statement Qualifications)

根据审查意见，本总结及后续报告对以下关键概念实施严格的科学表述校正：

1. **符号契约一致性**:
   - **错误表述**: “符号 100% 同向绝对证明了符号契约正确”；
   - **规范表述**: **“在测试的真实 shadow 样本内未观察到反向，LADRC shadow 指令与 Classical 保持了符合预期的一致补偿方向。”**
2. **离散化模型滞后**:
   - **错误表述**: “Exact ZOH 彻底消除了真实系统滞后”；
   - **规范表述**: **“Exact ZOH 避免了 Forward-Euler 的额外数值离散误差，但真实物理链路滞后仍然存在。”**
3. **扰动观测物理语义**:
   - **错误表述**: 将 $z_2$ 称为实际外力矩或真实扭矩；
   - **规范表述**: **$z_2$ 严格定义为“等效总扰动（Equivalent Lumped Disturbance）”，物理量纲为 $\text{rad/s}$，代表进入一阶误差导数通道的所有模型未建模动态与外扰总和。**
4. **控制量幅值比与参数调整判定**:
   - **修正认知**: 前序报告中“因 $u_{shadow}$ 为 Classical 的 2.5~3.5 倍，故需调小增益后再接入”之推论不成立。Shadow 模式下的实际物理轨迹完全由 Classical 控制器主导，LESO 观测到的并非 LADRC 闭环自身稳态残差，而是包含了 Classical 固有稳态偏差的总等效扰动。由于静态和温和扰动下 LADRC 残差绝对值 $< 0.009^\circ \ll 0.5^\circ$ 限幅门限，当前的保守参数（Roll/Pitch $b_0=10.0, \omega_c=1.0, \omega_o=4.0$）保持冻结，直接作为 J13-2B1 的接管基准。

---

## 5. 准入结论

J13-2A1 与 J13-2A2 满足全部既定质量与安全性指标：
- **J13-2A LADRC Shadow Mode Real-Hardware Qualification = PASS**
- 准入下一步：**J13-2B1: Roll Single-Axis LADRC Active Takeover — Static Qualification**
