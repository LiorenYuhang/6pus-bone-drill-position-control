# 6PUS 骨科并联机器人防抖控制与感知框架

> [English](README.md) | [中文]
> **面向 6-PUS 并联骨科手术机器人的 ROS 2 实时感知与主动防抖控制系统**

---

## 1. 项目简介 (Project Overview)

**6PUS-Anti-Vibration** 是专为 6-PUS（移动副-万向节-球副）Stewart 并联机构研发的开源高精度实时感知与运动控制框架。项目面向骨科手术机器人领域的高精细操作，旨在通过实时位姿解算、加加速度连续轨迹整形与高频伺服通信，实现平台在动基座扰动条件下的主动抗扰与高带宽振动抑制。

### 核心阶段划分：
- **J1–J10（已完成并冻结）**：构建了坚实的真实物理硬件运动与感知基线，涵盖多电机底层串口通信、微秒级闭式解析逆解、三阶加加速度连续轨迹平滑整形、50Hz 连续流式伺服驱动、双 H30 IMU 微秒级时钟对齐解算、数值正运动学重构，以及在真实物理平台上完成的 6 自由度单轴安全运行包络边界全域测定。
- **J11-M1（已完成并冻结）**：完成 Base-IMU 全 SO(3) 前馈姿态保持实机闭环、专用驱动层运行资格认证、两级安全策略 (Two-Tier Safety Policy)、300s 静态长周期达标以及真机低速动基座扰动防抖补偿验证（官方 Release Tag: `v1.0-j11-m1-feedforward-hardware-qualified`）。
- **J11（已完成并冻结）**：最终保留 Base-IMU SO(3) Feedforward + Platform-IMU Classical P/PD Attitude Feedback。Roll / Pitch / Yaw 参数：`Kp = [0.05, 0.05, 0.025]`，`Kd = [0, 0.0015, 0.0015] s`，`Ki = 0`；低通滤波截止频率：`fc = 5 Hz`（Roll=P，Pitch=PD，Yaw=PD）。Demo A 与 Demo B 实机验证通过（官方 Release Tag: `v1.0-j11-attitude-stabilization-frozen`）。
- **J12（已完成并冻结）**：经典前馈+P/PD动态性能与物理负载摸底（Classical Feedforward + P/PD Dynamic Performance Characterization）。全面完成大角度静态负载摸底（J12-1）、3自由度组合姿态准静态筛选（J12-2）以及手持高动态性能表征（J12-3，实测有效峰值角速度 `92.33°/s`）。实测电机最大电流 `468 mA`。主机端生产安全电流保护统一升级为 `MAX_CURRENT_MA = 1500`（官方 Release Tag: `v1.0-j12-classical-dynamic-performance-frozen`）。
- **J13（已完成并工程资格认证）**：真实 6PUS 并联平台一阶线性自抗扰动平台姿态残差控制（First-Order Linear ADRC / LADRC Residual Control）。完整闭环实机架构（Base $SO(3)$ 几何前馈 + 三轴独立一阶线性 ADRC，含 ZOH 离散二阶扩张状态观测器 LESO 与 $\pm 0.5^\circ$ 残差安全限幅）。完成空载 10s 物理接管资格认证（0 故障）；完成偏心负载工况下对比 Classical P/PD 基线实机测试。识别出底层通信与调度长尾延时为系统迈向长时间连续运行的核心瓶颈。阶段状态：CLOSED / ENGINEERING QUALIFIED。

---

## 2. 当前工程开发状态 (Current Development Status)

6PUS 并联平台的研发严格划分为两大核心板块：

```text
┌─────────────────────────────────────────────────────────────────────────┐
│              第一部分：物理实机运动与感知基础 (J1–J10)                  │
│                        [状态：已全面完成并冻结]                         │
│  • J1: RS485 UART 底层通信基线 (921600 8N1，六电机只读协议在线)         │
│  • J2: 6PUS 解析几何逆运动学 (闭式代数解，微秒级计算延迟)               │
│  • J3: 无锁非阻塞调度与 Latest-Value-Wins (LVW) 原子覆盖机制            │
│  • J4: 有状态加加速度限制轨迹整形器 (Stateful Jerk Shaper, C² 连续)     │
│  • J5: 纳秒单调时间轴重构与基于解析雅可比矩阵的 LM 高频数值正解         │
│  • J6: 机械整机总装调试与集约化 HOME/PARK 启动状态分类器                │
│  • J7: 50Hz 物理实机伺服驱动与 Design-B 0x29 伴随反馈机制               │
│  • J8: 9 阶段笛卡尔连续实机视频演示验收 (100% 反馈完整率，0 故障)       │
│  • J9: 双 Yesense H30 IMU 微秒级硬件时间对齐与 SO(3) 相对姿态积分       │
│  • J10: 6 自由度 HOME 基准单轴实机安全运行包络边界全域实测              │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│               第二部分：主动姿态稳定与防抖研究 (J11+)                   │
│                        [状态：前沿研究路线图]                           │
│  • J11-M1: 动基座全 SO(3) 前馈姿态保持与高可靠驱动运行资格认证          │
│            [状态：已全面完成并冻结 — Release Tag: v1.0-j11-m1]          │
│  • J11: Base-IMU SO(3) 前馈 + Platform-IMU P/PD 姿态反馈稳定控制         │
│         [状态：已全面完成并冻结 — Release Tag: v1.0-j11]                 │
│  • J12: 经典前馈+P/PD动态性能与物理负载摸底                              │
│         [状态：已全面完成并冻结 — Release Tag: v1.0-j12]                 │
│  • J13: 一阶线性自抗扰动平台姿态残差控制 (First-Order LADRC)              │
│         [状态：已闭环资格认证 — CLOSED / ENGINEERING QUALIFIED]          │
│  • 当前活动主线：J14 — 长时运行与通信可靠性攻关 (Runtime Reliability)    │
│    - 彻底消除 UART 同步阻塞与调度长尾抖动 (Long-Tail Latency)            │
│    - 严格保持两级安全策略 (80步恢复拒绝 / 283.3ms告警 / 1000ms硬停机)     │
│    - 资格验收目标：真实 6PUS 连续 10 分钟 50Hz Active 伺服闭环全通       │
│    - 后续研究方向：偏心负载工况下末端人工瞬态扰动抑制特性对比            │
└─────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **真实性与验证边界**：J1 至 J10、J11 与 J12 已全面冻结；J13 已正式按工程资格达标闭环。所有指标均基于真实物理机载遥测数据支持。在偏心负载 A/B 测试中，基座人工晃动输入非严格一致，未作 LADRC 全面优于经典控制的主观断言。

### J10 单自由度实机安全运行包络边界（以 HOME 为基准）

| 自由度 | 实测安全运行范围 | 边界制约机理 |
| :--- | :--- | :--- |
| **X** | `[-45.75, +45.75] mm` | 极限行程包络 (ID2 / ID5 电缸限位) |
| **Y** | `[-45.50, +49.00] mm` | 极限行程包络 (ID1 / ID4 电缸限位) |
| **Z** | `[-25.00, +25.00] mm` | 50.0 mm 总对称共模垂向行程 |
| **Roll** | `[-35.0, +37.5] deg` | 机械球铰极限角约束 |
| **Pitch** | `[-30.0, +35.0] deg` | 电缸单向行程达到极限 |
| **Yaw** | `[-90.0, +90.0] deg` | 完整测试旋转角包络 |

*(注：上述数据均为在 `OPERATING_HOME = [1000]*6` 步基准下独立单轴运动实测结果，详见 [`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](./agent_docs/stages/J10/J10_FINAL_SUMMARY.md))*。

### 真机定位与姿态跟踪性能（HOME 附近）

以下结果表示 HOME 附近、小范围、单轴条件下的 **command-to-feedback / FK / IMU 跟踪表现**，不等同于全工作空间绝对定位精度、外部计量系统标定精度或重复定位精度。

| 测试 | 指令/条件 | 实测响应 | 统计口径 | 工程解释 |
| --- | --- | --- | --- | --- |
| 执行器位置分辨率 | 配置映射：40 steps/mm | 0.025 mm/step | 由 `steps_per_meter` / `meters_per_step` 得到的关节指令量化 | 执行器/关节分辨率，不是 TCP 笛卡尔定位精度 |
| X 平移 | +0.5 / -0.5 mm | FK 局部位移 +0.3311 / -0.3991 mm；指令-响应绝对差值 0.1689 / 0.1009 mm | action 阶段最后 100 个 FK 样本均值减去 pre-HOME 阶段最后 100 个 FK 样本均值 | 两次实验中的局部、模型依赖 command-to-FK 跟踪 |
| Y 平移 | +0.5 / -0.5 mm | FK 局部位移 +0.5184 / -0.4957 mm；绝对差值 0.0184 / 0.0043 mm | 同一 last-100-sample action-minus-pre-HOME FK 均值口径 | 两次实验中的局部、模型依赖 command-to-FK 跟踪 |
| Z 平移 | +0.5 / -0.5 mm | FK 局部位移 +0.4769 / -0.5018 mm；绝对差值 0.0231 / 0.0018 mm | 同一 last-100-sample action-minus-pre-HOME FK 均值口径 | 局部、模型依赖跟踪；+Z 实验同时存在跨轴耦合 |
| Roll | +0.5 / -0.5 deg | Platform-IMU 均值 +0.4486 / -0.5083 deg；指令减响应的有符号均值误差 +0.0514 / +0.0083 deg | action 阶段后半段的算术均值 | HOME 附近小角度跟踪；现有实验具有方向差异 |
| Pitch | +0.5 / -0.5 deg | Platform-IMU 均值 +0.4995 / -0.5213 deg；有符号均值误差 +0.0005 / +0.0213 deg | 同一 action 阶段后半段均值口径 | HOME 附近小角度跟踪；现有实验具有方向差异 |
| Yaw | +2.0 deg | Platform-IMU 均值 +1.4825 deg；有符号均值误差 +0.5175 deg | 同一 action 阶段后半段均值口径 | 明显弱于 Roll/Pitch，不能据此给出统一姿态精度声明 |
| 全工作空间绝对定位精度 | 尚未专项外部标定 | 未知 | 未测量 | 当前未完成资格认证 |
| 重复定位精度 | 尚未做系统同点重复统计 | 未知 | 未测量 | 当前未完成资格认证 |

上述具有轴向与方向差异的结果采用两组明确定义且各自内部一致的统计口径：J9.1 平移口径与 J9.1 姿态口径。它们支持 HOME 附近的局部 command-to-feedback 表征，但不支持归纳为单一的笛卡尔或姿态精度规格。整机全工作空间绝对精度和重复定位精度尚需通过专项外部计量与同点重复统计进一步确定。详见[`完整证据总结`](./agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md)。

### J11 — 双 IMU 三轴姿态稳定（阶段成果）

**目标**：当 Base（基座）发生姿态运动时，利用双 IMU + 6PUS 主动补偿，使 Platform（动平台）尽量保持初始三轴姿态。

**方案**：

- **M1（纯前馈，已冻结）**：Base-IMU full SO(3) Feedforward；Platform IMU 仅用于 Observer / Validation。
- **M2（前馈 + 反馈，最终保留）**：Base-IMU SO(3) Feedforward + Platform-IMU Classical P/PD Attitude Feedback；Roll = P，Pitch = PD，Yaw = PD。

**最终参数**：

```text
Kp = [0.05, 0.05, 0.025]
Ki = [0.0, 0.0, 0.0]
Kd = [0.0, 0.0015, 0.0015] s
Derivative LPF fc = 5 Hz
Residual limit = ±0.5° / axis
```

#### 评价指标

- **Peak** $= \max_k |\theta_k|$：整个分析窗口内最大的绝对姿态偏差，回答「最坏时刻平台最大偏了多少」，反映最坏瞬时响应。
- **RMS** $= \sqrt{\frac{1}{N}\sum_k \theta_k^2}$：整个分析时间段内姿态误差的均方根，回答「整个过程中总体抖动有多大」，反映整体持续误差水平。
- **G_Peak** $=$ Platform Peak / Base Peak：Platform 最大姿态响应相对 Base 最大姿态扰动的比例。
- **G_RMS** $=$ Platform RMS / Base RMS：Platform 整体姿态误差相对 Base 整体姿态扰动的比例。

G 越小，表示 Platform 对 Base 扰动产生的姿态响应越小，即姿态扰动抑制越强。

**Roll / Pitch / Yaw** 分别用于观察三个姿态分量；**SO(3) attitude error** 描述当前三维姿态相对目标姿态的整体旋转误差。因此 **SO(3) G_Peak** 是整体三维姿态的最坏归一化响应，**SO(3) G_RMS** 是整个动态过程中整体三维姿态的归一化总体误差水平。

#### 最终结果

| 指标 | M1 纯前馈 | M2 前馈 + P/PD | 代表性下降 |
|---|---:|---:|---:|
| Roll G_RMS | 0.4071 | 0.2662 | 34.6% |
| Pitch G_RMS | 0.4153 | 0.2607 | 37.2% |
| Yaw G_RMS | 0.2912 | 0.2297 | 21.1% |
| SO(3) G_Peak | 0.7353 | 0.3063 | 58.3% |
| SO(3) G_RMS | 0.3713 | 0.2575 | 30.6% |

其中 `Reduction = (1 - G_M2 / G_M1) × 100%`。完整 8 项（含 Roll/Pitch/Yaw 的 G_Peak）与逐项计算口径见 [`docs/j11/J11_FINAL_CONTROL_COMPARISON.md`](./docs/j11/J11_FINAL_CONTROL_COMPARISON.md)。

> 上述百分比表示两组代表性真机实验中观测到的归一化姿态响应指标下降。由于两次人工 Base 扰动轨迹并非严格一致，该结果用于硬件归一化表征，而不解释为严格同输入条件下的传递函数增益变化。

#### 数据来源

- M1 参考数据集：`results/j11/runtime_validation/20260918_112354/J11_DISTURB_QUAL_feedforward_powered.csv`
- M2 参考数据集：`results/j11/m2_pid/20260920_095850/j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_telemetry_powered.csv`

后续图表重绘以及 Peak / RMS / G_Peak / G_RMS 重算，均可直接以上述原始 CSV 为数据来源。

### J11 演示

从刚开机的新 Terminal 中运行时，无需提前手工 source ROS 2 或 workspace。两个 wrapper 会自行加载 ROS 2 Jazzy 与 `bone_ws/install/setup.bash`；已经手工 source 的开发 Terminal 也同样支持。

纯前馈：

```bash
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j11_demo_feedforward.sh
```

前馈 + P/PD：

```bash
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j11_demo_feedforward_pd.sh
```

演示中只按一次 `Ctrl+C`，wrapper 会受控结束并返回 `OPERATING_HOME`。开发或排错时仍可显式 source `/opt/ros/jazzy/setup.bash` 与 `/home/lyapunov/bone_drill/bone_ws/install/setup.bash`，但这不是最终 Demo 的强制要求。

**J11 状态**：已完成并冻结（`v1.0-j11-attitude-stabilization-frozen`）—— Demo A（Feedforward）PASS；Demo B（Feedforward + P/PD）PASS ON REPEAT。

### J12 经典前馈+P/PD动态性能与物理负载摸底（阶段成果）

- **阶段状态**：**已完成并冻结（COMPLETED & FROZEN）**（官方 Release Tag: `v1.0-j12-classical-dynamic-performance-frozen`）。
- **实验范围**：大角度静态负载边界摸底（J12-1）、代表性三轴组合姿态准静态筛选（J12-2）以及手持快速高动态性能表征（J12-3）。
- **关键结论**：
  - 在手持大动态晃动下验证了经典前馈+P/PD抗扰性能，观测到的实测有效物理角速度峰值达 `92.33°/s`（P95 = `46.16°/s`）。
  - 全动态测试中电机电流温和，实测峰值仅 `468 mA`（无任何样本超过 500 mA）。明确高动态主要瓶颈为控制残差饱和（±0.5°）与执行器跟踪滞后，**绝非电流受限（Not Current Limited）**。
  - 主机端生产过流保护正式与执行器硬件内部保护对齐统一为 `MAX_CURRENT_MA = 1500`。
  - 详细总结：[`agent_docs/stages/J12/J12_FINAL_SUMMARY.md`](./agent_docs/stages/J12/J12_FINAL_SUMMARY.md)；交接文档：[`agent_docs/stages/J12/J12_TO_J13_HANDOFF.md`](./agent_docs/stages/J12/J12_TO_J13_HANDOFF.md)。

### J13 一阶线性 ADRC 姿态残差控制与偏心负载真机验证（阶段结果）

- **阶段状态**：**已闭环工程资格认证（CLOSED / ENGINEERING QUALIFIED）**（官方 Release Tag: `v1.0-j13-ladrc-attitude-residual-control-qualified`，最终 Commit: `89d38b3ac2bbb63234aa56f8ecd87178010dde76`）。
- **控制架构**：
  - **3-axis LADRC**：Base $SO(3)$ 几何前馈 + 三轴独立一阶线性 ADRC 残差控制（Base $SO(3)$ Feedforward + 3-axis First-Order Linear ADRC / LADRC Residual Control，采用严格 ZOH 离散二阶扩张状态观测器 LESO）。
  - **Classical 基线**：Base $SO(3)$ 几何前馈 + Roll P + Pitch PD + Yaw PD（Base $SO(3)$ Feedforward + Roll P + Pitch PD + Yaw PD）。
- **最终 LADRC 参数**：

| 轴向 | $b_0$ | $\omega_c$ | $\omega_o$ | 残差限幅 |
| :--- | :---: | :---: | :---: | :---: |
| **Roll** | 20.0 | 0.8 rad/s | 2.5 rad/s | $\pm 0.5^\circ$ |
| **Pitch** | 10.0 | 1.0 rad/s | 4.0 rad/s | $\pm 0.5^\circ$ |
| **Yaw** | 6.0 | 0.25 rad/s | 1.2 rad/s | $\pm 0.5^\circ$ |

*(参数说明：$b_0$ 为名义输入增益估计 nominal input-gain estimate，$\omega_c$ 为控制器闭环带宽 controller bandwidth，$\omega_o$ 为观测器带宽 observer bandwidth)。*

- **真实硬件资格结果与运行**：
  1. **3-axis LADRC 空载 10s 资格认证**：500 周期（约 9.98 s），430 active 周期，0 hard stop，0 motor fault，0 recovery reject，观测器全程稳定。**结论：PASS / QUALIFIED**。
  2. **偏心负载硬件实验**：Classical 组（500 cycles，429 active cycles）与 3-axis LADRC 组（500 cycles，446 active cycles）均完整完成真实硬件运行，无致命运行时故障（fatal runtime failure）。

- **偏心负载 A/B 核心数据与证据边界**：
平台 $SO(3)$ 姿态误差 RMS 统计：

| 轴向 | Classical RMS | 3-axis LADRC RMS | Base 激励比 (LADRC / Classical RMS) |
| :--- | :---: | :---: | :---: |
| **Roll** | $0.446^\circ$ | $1.072^\circ$ | 1.896× |
| **Pitch** | $0.868^\circ$ | $1.309^\circ$ | 1.501× |
| **Yaw** | $0.608^\circ$ | $0.441^\circ$ | 0.525× |

> [!WARNING]
> **上述数据不能直接作为控制器因果优劣结论，当前实验未证明 LADRC 全面优于 Classical**。因为两次实验中人工施加的 Base 晃动激励并不匹配（LADRC 组的 Base Roll 激励约为 Classical 的 1.896 倍，Pitch 约为 1.501 倍，而 Yaw 仅为 0.525 倍）。因此：
> - 本次 LADRC run 的 Roll / Pitch 误差偏高；
> - 本次 LADRC run 的 Yaw 误差偏低；
> - 当前非同源扰动实验无法证明 LADRC 全面优于 Classical，亦不能将 Yaw 轴误差降低单独归因于 LADRC 控制器。

- **关键工程发现**：
  - **Pitch 显著残差饱和**：Pitch LADRC 在 446 个 active 采样中出现 73 次限幅饱和（占比 **16.37%**），$u_\text{raw}$ 峰值达到约 $1.025^\circ$（被 clip 约束至 $\pm 0.5^\circ$ 残差上限）。这客观暴露了当前强扰动下 Pitch 残差控制能力裕量（authority）与参数匹配的工程边界，属于正常的工程限幅表现而非算法失效。
  - **LESO 观测器状态稳定**：LESO $z_1$ 与 $z_2$ 全程保持数值稳定。必须明确 $z_2$ 表征的是**等效集总扰动项（Equivalent Lumped Disturbance）**，而非真实物理扰动力矩（physical torque）。
  - **长时运行时通信与调度长尾**：J13 实验同时暴露出底层长时运行的通信与调度长尾抖动问题：
    - Max Scheduler Gap：Classical $\approx 274.9\text{ ms}$，LADRC $\approx 370.6\text{ ms}$
    - Max UART Drain：Classical $\approx 182.6\text{ ms}$，LADRC $\approx 363.1\text{ ms}$
    - Max Feedback Age：Classical $\approx 269.9\text{ ms}$，LADRC $\approx 352.4\text{ ms}$
    10s 短时实验能够稳定完成，但此类 timing long-tail 仍是制约系统连续运行至数分钟乃至 10 分钟级的核心瓶颈。共同根因尚未完全定论，移交后续专班排查。

- **J13 最终核心结论**：
  - First-Order 3-axis LADRC 已在真实 6PUS 并联平台上成功部署并运行；
  - 空载 10s qualification 确认 PASS；
  - 偏心负载真实硬件运行完整完成，LESO 全程数值稳定；
  - 当前没有证据证明 LADRC 全面优于 Classical；
  - Pitch 存在明显的残差限幅饱和（16.37%）；
  - 底层 runtime timing long-tail 移交 J14 专班攻关。
- **详细文档与交接**：
  - 详细总结：[`agent_docs/stages/J13/J13_FINAL_SUMMARY.md`](./agent_docs/stages/J13/J13_FINAL_SUMMARY.md)
  - 阶段交接：[`agent_docs/stages/J13/J13_TO_J14_HANDOFF.md`](./agent_docs/stages/J13/J13_TO_J14_HANDOFF.md)

### J14-A 长时运行与通信可靠性攻关 (阶段闭环成果)

- **状态**：**CLOSED / FROZEN** (Release Tag：`v1.0-j14-a-three-process-runtime-reliability-qualified`)。
- **生产解耦架构**：
  - **Parent Node**：`SingleThreadedExecutor` 单线程内联回调处理（消除多线程 wait-set scanning 竞争，解决动态话题负载下的 IPC 租约饥饿机制）；
  - **UART Child**：独立子进程，RS485 串口物理独占 owner (`/dev/ttyUSB0` @ 921600 baud)；
  - **Feedback Bridge**：独立子进程，底层电机反馈 IPC 独占 consumer (`AF_UNIX SOCK_SEQPACKET` @ 50 Hz)；
  - **Compatibility Relay**：外部 ROS 话题转换适配器，零直接串口或指令仲裁权限。
- **真实硬件资格认证与结果**：
  1. **Q3-R1D (Classical P/PD Active 60s)**：**PASS**（3529 driver 周期，2945 ACTIVE 周期 @ 50 Hz，0 故障停机，双 IMU age < 57 ms）；
  2. **Q3-R2 (3-axis LADRC Active 60s)**：**PASS**（3539 driver 周期，2942 ACTIVE 周期 @ 50 Hz，0 故障停机，双 IMU age < 66 ms，LESO 观测器状态全程有界稳定）；
  3. **三个官方 Demo 实机确认**：全部三个 Demo 均已由用户在真实硬件上完整确认（**HUMAN HARDWARE CONFIRMATION = PASS**），走通完整生命周期：Startup HOME → Dual-IMU 预检 → ACTIVE → 操作员 Ctrl+C → 受控 Shutdown HOME → Pause 6/6 → 串口干净释放与 0 残留进程。
- **官方 Demo 运行命令**：

```bash
# 1. Base SO(3) 纯前馈模式
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward.sh

# 2. Base SO(3) 前馈 + 经典任务空间 P/PD 闭环
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward_pd.sh

# 3. Base SO(3) 前馈 + 三轴一阶 LADRC 残差闭环
cd /home/lyapunov/bone_drill
bash bone_ws/scripts/run_j14_demo_feedforward_ladrc.sh
```

- **详细文档与交接**：
  - 阶段权威总结：[`agent_docs/stages/J14/J14_FINAL_SUMMARY.md`](./agent_docs/stages/J14/J14_FINAL_SUMMARY.md)
  - 跨阶段交接规范：[`agent_docs/stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md`](./agent_docs/stages/J14/J14_TO_NEXT_STAGE_HANDOFF.md)
  - 生产驱动架构说明：[`docs/J14_PRODUCTION_DRIVER.md`](./docs/J14_PRODUCTION_DRIVER.md)
  - Demo 操作指南：[`docs/J14_DEMO_ENTRYPOINTS.md`](./docs/J14_DEMO_ENTRYPOINTS.md)

### 文档

详细技术结果（完整 8 项指标与计算口径）：[`docs/j11/J11_FINAL_CONTROL_COMPARISON.md`](./docs/j11/J11_FINAL_CONTROL_COMPARISON.md)

学习 / ROS2 / 操作资料入口：[`docs/`](./docs/)（含 `docs/整体学习参考/` 下的 ROS2 全栈教程 `6PUS_ROS2_CODE_LEARNING_TUTORIAL.md` 与实机操作指南 `6PUS_SERVO_OPERATION_GUIDE.md`，以及 `docs/基础操作/` 下的基础操作资料）

---

## 3. 代码与文档体系结构 (Repository Layout)

项目严格遵行代码、人读技术知识与机读数据资产三者职责边界隔离：

```text
/home/lyapunov/bone_drill/
├── bone_ws/                               <-- [ROS 2 工作空间] 源代码、功能包、测试用例与运行脚本
│   ├── src/
│   │   ├── pus6_msgs/                     # 自定义 ROS 2 消息接口 (TimedJointFeedback, FkStateStamped 等)
│   │   ├── pus6_kinematics/               # 解析逆解、LM 数值正解与 6pus_geometry.yaml 唯一几何真值源
│   │   ├── pus6_motor_driver/             # RS485 串口驱动、50Hz 伺服传输与两级安全看门狗
│   │   ├── pus6_control/                  # 有状态加加速度连续轨迹整形器与控制器
│   │   ├── pus6_bringup/                  # 系统级 Launch 启动编排与参数配置
│   │   └── h30_imu/                       # Yesense H30 IMU C++ 驱动与 SO(3) 相对姿态解算
│   └── scripts/                           # 可复现的实机/仿真运行脚本与验收工具链
│
├── agent_docs/                            <-- [Agent 内部工作文档] stage 历史、决策、调参历史、交接、故障审计
│   ├── stages/                            # 阶段实施方案 (J1..J11)、验收总结与跨阶段交接规范 (Handoff)
│   ├── reports/                           # 深度实测分析报告 (J10 工作空间分析、J9.1 动平台力臂分析)
│   ├── decisions/                         # 架构决策记录 (ADR) 与硬件准入测试报告
│   ├── frozen/                            # 只读冻结基线清单与代码校验指纹
│   └── archive/                           # 历史遗留操作手册与过时参考资料
│
├── results/                               <-- [机读实验与运行数据] 自动化测试与实机运行生成的数据证据
│   ├── README.md                          # 数据集索引与实验证据导航
│   ├── stewart3_j8_figure6_20260916_.../  # 权威 J8 笛卡尔九阶段实机运动全量时序 (Stewart-3 图 6)
│   ├── j7/                                # 权威 J7 预检与 50Hz 伺服基准 JSON
│   ├── j9_1/                              # 权威 J9.1 双 IMU 姿态与力臂耦合实测数据集
│   └── j10/                               # 权威 J10 六自由度安全包络实测量测数据
│
├── hardware/                              <-- [硬件与几何数据] 电机选型表、电气接线图与 3D 机械结构
├── up/                                    <-- [UART 底层验证基线] 独立多电机串口测试台与基线驱动
├── STM32/                                 <-- [固件] 下位机工程源码
├── docs/                                 <-- [用户文档] 用户主动查看的学习/操作/教程/技术结果
└── AGENTS.md                              <-- [全局 Agent 规则] 操作规范、安全红线与工程不变量
```

> **职责边界原则**：
> - `docs/` = 用户主动需要查看的文档（学习、操作、教程、最终技术结果与汇报入口）。
> - `agent_docs/` = Agent 内部工作文档（stage 历史、决策、调参历史、交接、故障审计），用户正常不需要查看。
> - `results/` = 机器程序运行、测试和实机实验产出的 CSV、遥测 JSON 与运行原始日志。

---

## 4. 推荐阅读与学习路径 (Recommended Reading Order)

对于首次接触本项目的工程师、研究者或智能体，建议按如下次序阅读：

1. **宏观概览与基线**：首先通读本文档 [`README_CN.md`](./README_CN.md)（或 [`README.md`](./README.md)），掌握系统整体架构与阶段进展。
2. **ROS 2 全栈技术教程**：研读 [`docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](./docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)，掌握双链路解耦设计、闭式逆解、LM 数值正解及 50Hz 伺服驱动的详细实现。
3. **权威文档检索地图**：参考 [`agent_docs/README.md`](./agent_docs/README.md)，快速定位具体阶段的技术方案、交接规范与硬件准入报告。
4. **物理实机操作与安全**：若需操作物理样机，务必在操作前核对 [`docs/整体学习参考/6PUS_SERVO_OPERATION_GUIDE.md`](./docs/整体学习参考/6PUS_SERVO_OPERATION_GUIDE.md)。
5. **权威实验证据库**：查阅 [`results/README.md`](./results/README.md)，加载并验证物理样机真实飞行数据（如 J8 连续运动数据集）。

---

## 5. 离线公开复现指南 (Offline Public Reproducibility)

第三方研究人员无需拥有真实 6PUS 样机，即可在纯软件离线环境中构建代码、执行全量单元测试与分析真实硬件数据：

### 1. 环境准备
- **操作系统**：Ubuntu 24.04 LTS
- **ROS 2 版本**：Jazzy Jalisco
- **Python 版本**：$\ge 3.10$

```bash
# 克隆仓库
git clone https://github.com/lyapunov/bone_drill.git
cd bone_drill

# 引入 ROS 2 环境变量
source /opt/ros/jazzy/setup.bash
```

### 2. 编译工作空间
```bash
cd bone_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 执行离线单元与集成测试 (Mock 模式)
全部测试均在 `hardware_mode:=mock` 下运行，且**代码级严格断言 **`hardware_write_count = 0`****：
```bash
# 运行 bone_ws 下全部 6 个功能包的 346+ 项单元与集成测试
colcon test --ctest-args -R test
colcon test-result --all

# 运行 J8 图 6 纯旁路记录器单元测试
pytest scripts/test_j8_figure6_recorder.py -v
```

### 4. 加载与分析真实实机实验数据
无需真机，直接用 Python 读取跟踪的权威 J8 九阶段 50Hz 实机时序：
```python
import pandas as pd
import json

# 加载 J8 笛卡尔运动实测时序
dataset_dir = "../results/stewart3_j8_figure6_20260916_135013"
cmd_df = pd.read_csv(f"{dataset_dir}/command_steps.csv")
fb_df = pd.read_csv(f"{dataset_dir}/joint_feedback.csv")
fk_df = pd.read_csv(f"{dataset_dir}/fk.csv")

print(f"J8 实机采样周期数: {len(cmd_df)} 帧 @ 50 Hz (~{len(cmd_df)/50:.1f} 秒)")
print("前四轴反馈步数:", fb_df[['fb_1', 'fb_2', 'fb_3', 'fb_4']].head())
```

---

## 6. 核心安全红线与实机操作守则

- **默认零运动**：所有节点与测试脚本默认启动于 Mock / FakeBus 模式。
- **真机授权原则**：使用 `--real-hardware` 或 `hardware_mode:=active` 驱动物理串口前，必须确认 6 轴处于有效回零位置（`[995, 1005]` 步）、硬件急停可靠，并取得操作人员明确确认。
- **写指令零自动重试**：RS485 上的运动写指令严禁发生超时自动重试，通信异常时触发单次暂停（0x1A）并安全停机。
