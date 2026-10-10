# J12 至 J13 阶段交接规范 (J12 to J13 Handoff)
# Classical Baseline Characterization to ADRC Attitude Disturbance Rejection

- **交接发起方**: **J12 (Classical Dynamic Performance Characterization)** — **CLOSED / FROZEN**
- **交接接收方**: **J13 (ADRC Attitude Disturbance Rejection)** — **NEXT ACTIVE STAGE**
- **交接日期**: 2026-10-05
- **冻结 Tag**: `v1.0-j12-classical-dynamic-performance-frozen`

---

## 1. J13 继承的物理与算法资产 (Inherited Assets)

### 1.1 J11 冻结经典控制基准 (Frozen Classical Baseline)
J13 开展 ADRC 对比实验时，必须严格以 J11 冻结参数作为“被对照组（Classical Benchmark）”，严禁私自修改经典基线参数：
- **比例增益 (P)**: $K_p = [0.05, 0.05, 0.025]$（Roll, Pitch, Yaw）
- **积分增益 (I)**: $K_i = [0.0, 0.0, 0.0]$
- **微分增益 (D)**: $K_d = [0.0, 0.0015, 0.0015]\text{ s}$（Roll 零微分，Pitch/Yaw 微分使能）
- **微分低通滤波**: $f_c = 5.0\text{ Hz}$（一阶巴特沃斯低通）
- **残差安全限幅**: 三轴各自截断于 $\pm 0.5^\circ$
- **姿态前馈算子**: 完整 $SO(3)$ 旋转矩阵前馈 $\mathbf{R}_{BP\_FF} = \Delta \mathbf{R}_B^T$
- **总合成命令**: $\mathbf{R}_{BP\_cmd} = \mathbf{R}_{BP\_FF} \cdot \Delta \mathbf{R}_{PID\_B}$

### 1.2 J12 物理表征数据与瓶颈规律 (Dynamic Characterization Evidence)
J13 无需盲目试错，可直接使用 J12 实测物理规律：
- **安全姿态范围**: Base $SO(3) \le 22^\circ \sim 30^\circ$（避开负向 Roll $-38^\circ \sim -45^\circ$ 高负载陷阱）；
- **动态速度跨度**: 实测观察到的手持有效角速度峰值 $|\omega|_B = 92.33^\circ/\text{s}$（P95 = $46.16^\circ/\text{s}$，均值 $16.72^\circ/\text{s}$）；
- **电机电流特性**: 动态晃动下六轴电流极值为 **468 mA**（均值约 81 mA），全 Run 12,042 个样本 0 次超过 500 mA，证明**机构在正常工作姿态下绝非 Current Limited**；
- **核心控制瓶颈**: 经典控制下的主要误差源为 **`Control Residual Saturation (±0.5° 饱和)`** 与 **`Actuator Tracking Limitation (执行器受速度斜率限制，最大滞后 163 steps / 4.08 mm)`**。

### 1.3 生产底层安全环境 (Production Safety Invariants)
- **宿主机过流硬门限**: `MAX_CURRENT_MA = 1500`（代码默认固定，不依赖环境变量）；
- **执行器内部过流保护**: 寄存器 `0x20` 维持 1500 mA 不变；
- **两级时效看门狗**: 283.3 ms 告警 / 1.0 s 硬停机策略保持生效。

---

## 2. J13 下一阶段核心攻关任务 (Core Research Focus)

J13 正式进入博士课题高级主动控制主线：**ADRC 姿态抗扰与振动抑制 (ADRC Attitude Disturbance Rejection)**。

1. **自抗扰控制器 (ADRC) 算法实现与仿真**:
   - 建立任务空间姿态非线性通道模型；
   - 设计扩张状态观测器 (Extended State Observer, ESO)，对动基座扰动与模型不确定性进行实时总扰动估计；
   - 设计非线性/线性反馈控制律 (NLSEF / LSEF) 进行总扰动实时前馈补偿。
2. **算法迁移与实机闭环**:
   - 将 ADRC 控制器接入 6PUS 50 Hz 伺服主线程（替换原 P/PD 模块，保留安全限幅与底层驱动架构）。
3. **同工况严格受控对比 (Matched-Condition Comparison)**:
   - 在相同 Base 扰动工况下（慢速、中速、快速），定量对比：
     - ADRC vs J11/J12 Classical P/PD 的平台姿态残差（RMS, P95, Peak）；
     - ADRC 对扰动频率的相移与抑制带宽；
     - 执行器步频需求与跟踪滞后对比。

---

## 3. J13 严格负面清单 (Strict Prohibitions - What NOT to Do)

为了高度聚焦控制核心，**严禁在 J13 重新重复 J12 已完成的物理摸底工作**：
- **严禁重新进行大角度静态摸底**（J12-1 已闭环）；
- **严禁重新进行 PARK/HOME 电流与速度摸底实验**（J12-3C 已证明空载最大速度仅 184 mA）；
- **严禁重新进行单轴或多轴工作空间筛选**（J10/J12-2 已闭环）；
- **严禁重新优化 J7/J8/J11 底层通信与调度代码**（底层已全面封顶）。
