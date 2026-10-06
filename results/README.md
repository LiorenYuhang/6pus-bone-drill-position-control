# Results & Experimental Evidence Index

本文档为 `/home/lyapunov/bone_drill/results` 的**数据集与实验证据索引清单**。  
根据 [`AGENTS.md`](../AGENTS.md) 规定，`results/` 专用于存放程序运行、自动化测试、仿真评测以及实机实验生成的各类机读数据证据与运行时产物（machine-generated / experiment-generated evidence and runtime artifacts）。本文档是 `results/` 目录下唯一允许的索引元数据例外，所有人类与 Agent 撰写的深度分析与结论报告统一归档于 [`agent_docs/reports/`](../agent_docs/reports/)。

**Performance Summary**：原始 evidence 保存在本目录；HOME 附近定位与姿态跟踪性能的 metric semantics、详细工程解释、限制与证据追溯见 [`agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md`](../agent_docs/reports/REAL_HARDWARE_PERFORMANCE_SUMMARY.md)。

---

## 1. 证据分级体系 (Evidence Hierarchy)

为了在保证实机实验全可复现性的同时，防止海量高频运行时日志造成 Git 仓库膨胀，本项目对实验数据实施严格的四级治理：

| 级别 | 定义与准入标准 | Git 版本控制策略 | 说明与典型产物 |
| :--- | :--- | :---: | :--- |
| **Canonical Final Evidence**<br>*(权威基准证据)* | 支撑核心里程碑结论、论文/博客图表（如 Stewart-3 图 6）、最终控制结论、架构验收准入与阶段冻结的不可替代实机数据。具备完备元数据（`metadata.json`）、完整时序、严格单调递增时间戳与无故障断言。 | **RECOMMENDED**<br>(纳入 Git，供 clone 后重算/作图) | • `stewart3_j8_figure6_20260916_135013/`<br>• `j7/` (C10 预检与 50Hz 伺服基准)<br>• `j9_1/` (双 IMU 姿态与力臂可辨识性基线)<br>• `j10/` (6 自由度单轴安全包络量测基线)<br>• J11 M1/M2 最终 raw CSV + summary |
| **Representative Evidence**<br>*(代表性样本)* | 验证算法阶段性表现或特定边界条件的典型采样切片，用于离线校验或跨环境基准对齐。 | **OPTIONAL**<br>(按需精简后跟踪) | • 仿真模型对齐样本<br>• 控制器参数基准响应片段 |
| **Intermediate / Tuning**<br>*(中间过程与调试数据)* | 探索性边界搜索、高频扫描原始遥测、调参重试、单步调试 log 以及未达准入标准的运行产物。数据量通常在数十至数百兆。 | **LOCAL ONLY**<br>(保留于本地，Git 忽略) | • `results/J10_A*/` 各轴超大扫描数据<br>• `results/j11/j11_2a/` 早期零增益粗测与本地调试数据<br>• `results/j11/m2_pid/` 中的中间调参目录 |
| **Smoke Test / Debug**<br>*(Demo 冒烟与故障证据)* | Demo smoke-test 与运行时故障的 raw telemetry。长期信息由 `agent_docs/` 中的对应 audit 文档保存，不要求全量 raw 上传。 | **LOCAL ONLY by default**<br>(默认不提交完整 raw) | • Demo A/B smoke-test 遥测<br>• 单次 USB-UART fault 的完整 raw CSV |

---

## 2. 真实硬件数据与仿真数据判定准则 (Real-Hardware vs. Simulation)

根据 `AGENTS.md` 核心安全与证据真实性要求，必须严格区分仿真/测试与真实物理硬件数据：

- **真实硬件数据集 (Real-Hardware Evidence)**：
  - 必须由物理 6PUS 样机与真实传感器运行产生；
  - 包含真实 UART 硬件纳秒单调时戳（如 `snapshot_start_monotonic_ns`, `snapshot_end_monotonic_ns`）；
  - 包含 6 轴电机底层物理响应（实际位置 `fb_1`..`fb_6`、电机电流 `current_ma_1`..`current_ma_6`、硬件故障字 `fault_1`..`fault_6`）；
  - 包含硬件前置检查通过标记（Preflight Check 1..10 PASS，0 timeout，0 fault，`hardware_write_count > 0` 经明确书面授权）。
- **仿真与无实机测试数据集 (Simulation / Mock / Dry-Run Evidence)**：
  - 基于模型计算或 FakeBus 生成；
  - 严格记录 **`hardware_write_count = 0`**；
  - 仅用于算法逻辑、坐标变换与调度确定性证明，严禁将仿真或软件调度结果表述为真实物理平台性能。

---

## 3. 权威基准数据集索引 (Canonical Datasets Index)

当前仓库跟踪的核心基准数据集及与 [`agent_docs/`](../agent_docs/) 分析报告的映射关系如下：

### 3.1 J8 笛卡尔空间九阶段连续实机运动全量时序 (Stewart-3 图 6)
- **数据集路径**：[`results/stewart3_j8_figure6_20260916_135013/`](./stewart3_j8_figure6_20260916_135013/)
- **性质**：真实物理样机（Real-Hardware, J8 Qualified Baseline）
- **规模**：3.9 MB，持续时长 137.132s，6900 个 50Hz 伺服周期，100.0% 反馈完整率，0 fault，0 timeout。
- **文件清单**：
  - `target_pose.csv` (6,390 帧): 50Hz 笛卡尔目标位姿（$x, y, z, \text{roll}, \text{pitch}, \text{yaw}, q_x, q_y, q_z, q_w$）
  - `command_steps.csv` (6,909 帧): 逆解与轨迹整形后下发给 6 轴电缸的步数指令（`cmd_1`..`cmd_6`）
  - `joint_feedback.csv` (6,901 帧): Design-B 0x29 伴随反馈的 6 轴实际步数、位置(m)、电流(mA)及故障状态
  - `fk.csv` (6,900 帧): 独立单线程 Fixed-Lag 零外推 LM 数值正解重构位姿与收敛残差
  - `stage_events.csv` (18 记录): 9 阶段（HOME $\to$ X $\to$ Y $\to$ Z $\to$ Roll $\to$ Pitch $\to$ Yaw $\to$ 6DOF $\to$ HOME）纳秒级边界时间戳
  - `metadata.json`: 运行环境、Git 提交、机构参数、频次与数据完整性断言
  - `recorder_summary.json`: 记录器退出统计与各时序刷盘确认元数据
- **对应分析与报告**：
  - 权威准入报告：[`agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md`](../agent_docs/decisions/J8_FINAL_HARDWARE_QUALIFICATION.md)
  - 教学透视追踪：[`docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md#9`](../docs/整体学习参考/6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)

### 3.2 J10 单轴 6 自由度实机安全运行包络边界基准
- **数据集路径**：[`results/j10/`](./j10/)
- **性质**：真实物理样机实测与外推元数据（Real-Hardware Boundary Measurements）
- **包含内容**：
  - `J10_A2_2_X_positive_offline_projection.json`: X 正向离线几何外推基准
  - `J10_A3_3_X_POS20_20260909_085910_revalidation.json`: X+20mm 复测验证
  - `J10_A4_X_POS_LARGE_MOTION_20260909.json`: X 大位移运动边界
  - `J10_A5_X_POS_OPERATIONAL_BOUNDARY_20260909.json`: X 运行包络边界
  - `J10_A6_1_ACTUATOR_LIMIT_POLICY_CLEANUP_20260909.json`: 执行器限位规则清理
  - `J10_A6_X_POS_EXPLORATORY_ENVELOPE_CLOSURE_20260909.json`: X 包络闭环元数据
  - `J10_X_POS10_20260908_151149_analysis.json`: X+10mm 分析基准
  - `J10_X_POS30_20260909_091510_boundary_measurement.json`: X+30mm 边界测量
- **对应分析与报告**：
  - 阶段总论：[`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](../agent_docs/stages/J10/J10_FINAL_SUMMARY.md)
  - 15 篇各轴独立分析报告：[`agent_docs/reports/j10/`](../agent_docs/reports/j10/)

### 3.3 J9.1 双 H30 IMU 静态/动态姿态与力臂特性
- **数据集路径**：[`results/j9_1/`](./j9_1/)
- **性质**：双 Yesense H30 IMU 真实硬件采集（Dual IMU Hardware Sensing Baseline）
- **包含内容**：
  - `closed_loop_attitude/`: Roll/Pitch/Yaw 单轴及 3DOF 复合姿态闭环伺服遥测 CSV 与元数据
  - `open_loop_attitude/`: 姿态开环对比测试时序
  - `dual_imu_baseline/`: 基座与动平台静态零偏与微秒时间对齐基准
  - `platform_lever_arm_analysis/`: 动平台力臂可辨识性矩阵分析数据
  - `rotation_lever_arm_characterization/`: 旋转力臂实测响应
  - `translation_characterization/`: X/Y/Z 平移激励下的 IMU 耦合响应
  - `translation_coupling_analysis/`: 平移耦合特征矩阵 CSV 与 JSON
- **对应分析与报告**：
  - J9 最终总结：[`agent_docs/stages/J9/J9_FINAL_SUMMARY.md`](../agent_docs/stages/J9/J9_FINAL_SUMMARY.md)
  - 平移耦合与力臂分析报告：[`agent_docs/reports/j9_1/`](../agent_docs/reports/j9_1/)

### 3.4 J7 六轴 50Hz 伺服通信驱动基准
- **数据集路径**：[`results/j7/`](./j7/)
- **性质**：底层 RS485 真实硬件通信基准（Low-level Comm Benchmarks）
- **包含内容**：
  - `j7_c10_read_only_preflight.json`: C10 只读预检硬件状态
  - `j7_first_six_axis_servo_hold.json`: 六轴 50Hz 首次静止伺服保位时序统计
  - `j7_id1_continuous_servo_stream.json`: 单电机高频伺服连续流压测
  - `j7_single_motor_servo_comparison.json`: 单机对比基准
- **对应分析与报告**：
  - J7 架构决策：[`agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md`](../agent_docs/decisions/J7_C0_SERVO_ARCHITECTURE_DESIGN.md)

### 3.5 J11-M1 动基座姿态前馈与驱动准入权威基准
- **数据集索引总表**：[`results/j11/M1_DATASET_INDEX.md`](./j11/M1_DATASET_INDEX.md)
- **数据性质与定位**：`Frozen Engineering Evidence + Preliminary Control Baseline`（已冻结的工程验证凭据与初步控制基准，严禁作为 Final Paper 对比数据；后续 PID/ADRC 研发就绪后将统一协议重新横向对比）
- **官方 Release Tag**：`v1.0-j11-m1-feedforward-hardware-qualified`
- **核心基准清单**：
  1. **`CANONICAL_M1_STATIC`** ([`results/j11/runtime_validation/20260918_111527/`](./j11/runtime_validation/20260918_111527/)):
     - 用途: 300s 长期连续运行与双层安全策略 (Two-Tier Safety Policy) 冻结运行时静态基线；
     - 事实: 14,944 次硬件写操作，17 次暂态告警均自动恢复，Hard STOP = 0，过流 = 0，电机故障 = 0，HOME 误差 $\le 2\text{ steps}$。
  2. **`CANONICAL_M1_DYNAMIC`** ([`results/j11/runtime_validation/20260918_112354/`](./j11/runtime_validation/20260918_112354/)):
     - 用途: 120s 真实低速动基座扰动全 SO(3) 纯前馈动态工程基线；
     - 事实: 5,929 次硬件写操作，退出原因 `scheduled_duration_complete`（非故障中断），Base 最大 SO(3) 扰动 $13.54^\circ$，电机行程 808~1203 步，Platform 倾角误差 RMS $1.55^\circ$。
- **对应分析与报告**：
  - J11-M1 最终总结与规范：[`agent_docs/stages/J11/J11_M1_FINAL_SUMMARY.md`](../agent_docs/stages/J11/J11_M1_FINAL_SUMMARY.md)
  - J11 阶段研发主案：[`agent_docs/stages/J11/README.md`](../agent_docs/stages/J11/README.md)
  - 当前状态卡：[`agent_docs/decisions/CURRENT_STAGE.md`](../agent_docs/decisions/CURRENT_STAGE.md)

### 3.6 J12 经典前馈+P/PD动态性能与物理负载摸底基准
- **数据集路径**：[`results/j12/`](./j12/)
- **性质**：真实物理样机实机测试（Real-Hardware Dynamic Performance & Load Characterization）
- **官方 Release Tag**：`v1.0-j12-classical-dynamic-performance-frozen`
- **核心基准清单**：
  1. **J12-1 大角度静态负载边界** ([`results/j12/j12_1_large_angle_static_load/`](./j12/j12_1_large_angle_static_load/)):
     - `20261005_104403`: Roll $\approx -25.6^\circ$ 静态台阶，跟踪正常，稳态电流 $<15\text{ mA}$；
     - `20261005_110720`: Roll $+32.3^\circ$ 静态稳定台阶与深倾角负载测试。
  2. **J12-2 组合姿态准静态筛选** ([`results/j12/j12_2_combined_quasistatic/20261005_122117/`](./j12/j12_2_combined_quasistatic/20261005_122117/)):
     - 4 组代表性 3DOF 复合姿态准静态筛选，3584 周期 0 故障退出，稳态电流 $20 \sim 150\text{ mA}$。
  3. **J12-3 手持高动态性能表征** ([`results/j12/j12_3_dynamic_performance/20261005_152615/`](./j12/j12_3_dynamic_performance/20261005_152615/)):
     - 2007 闭环有效周期，Base 实测有效峰值角速度 $92.33^\circ/\text{s}$（P95 = $46.16^\circ/\text{s}$），电机电流峰值仅 $468\text{ mA}$（0 次 $>500\text{ mA}$），证实机构非电流受限。
- **对应分析与报告**：
  - 权威总结报告：[`agent_docs/J12_FINAL_SUMMARY.md`](../agent_docs/J12_FINAL_SUMMARY.md)
  - 阶段交接规范：[`agent_docs/J12_TO_J13_HANDOFF.md`](../agent_docs/J12_TO_J13_HANDOFF.md)
  - 当前状态卡：[`agent_docs/decisions/CURRENT_STAGE.md`](../agent_docs/decisions/CURRENT_STAGE.md)

### 3.7 J13-2A2 LADRC 影子模式实机资格认证基准
- **数据集路径**：[`results/j13/j13_2a2_shadow/20261005_205126/`](./j13/j13_2a2_shadow/20261005_205126/)
- **性质**：真实物理样机实机测试（Real-Hardware LADRC Shadow Mode Run, Classical P/PD Active, LADRC Passive Observer）
- **核心基准清单**：
  1. **Static 静态基线** (`static/`): 1,445 周期，零硬件故障，状态观测器误差收敛 $z_1-e$ RMSE $\approx 0.0010^\circ$；
  2. **Roll/Pitch Gentle 温和扰动** (`roll_pitch_gentle/`): 2,195 周期，峰值电流 61 mA，观测器稳定无震荡；
  3. **Yaw Gentle 温和扰动** (`yaw_gentle/`): 2,180 周期，峰值电流 35 mA，三轴独立解耦正常；
  4. **3DOF Gentle 复合扰动** (`three_axis_gentle/`): 2,942 周期，三轴复合动态观测正常，0% 限幅饱和，0% 控制权泄漏。
- **对应分析与报告**：
  - 资格认证总结：[`agent_docs/stages/J13/J13_2A_LADRC_SHADOW_QUALIFICATION.md`](../agent_docs/stages/J13/J13_2A_LADRC_SHADOW_QUALIFICATION.md)
  - 当前状态卡：[`agent_docs/decisions/CURRENT_STAGE.md`](../agent_docs/decisions/CURRENT_STAGE.md)

---

## 4. 离线复现与数据查看指引

第三方研究人员无需连接真实 6PUS 硬件，即可直接加载和分析上述 Canonical 数据集：

```python
import pandas as pd
import json

# 1. 加载 J8 图 6 真实实机 50Hz 遥测时序
j8_path = "results/stewart3_j8_figure6_20260916_135013"
target_df = pd.read_csv(f"{j8_path}/target_pose.csv")
feedback_df = pd.read_csv(f"{j8_path}/joint_feedback.csv")
fk_df = pd.read_csv(f"{j8_path}/fk.csv")
events_df = pd.read_csv(f"{j8_path}/stage_events.csv")

with open(f"{j8_path}/metadata.json") as f:
    meta = json.load(f)

print(f"J8 Real-Hardware Run: Duration = {meta['duration_s']}s, Rate = {meta['mean_frequency_hz']}Hz")
print(f"Recorded Stages: {list(events_df['stage_name'].unique())}")
```
