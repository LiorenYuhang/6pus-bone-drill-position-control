# J11-M1 Dataset Index

## 1. 数据集定位与学术/工程声明 (Dataset Status & Scope)

> [!IMPORTANT]
> **当前 J11-M1 数据集性质声明**:
> 本数据集属于 **`Frozen Engineering Evidence + Preliminary Control Baseline`**（已冻结的工程验证凭证与初步控制基准），**严禁标记为 Final Paper Comparison Dataset（最终论文横向对比数据集）**。
> 
> **理由**:
> 后续 J11-M2 (PID) 与 J11-M3 (ADRC) 研发就绪后，必须在完全统一的人工/机械扰动协议与评价基准下，同期执行重测（`Feedforward` vs `Feedforward + PID` vs `Feedforward + ADRC`），以确保论文终稿学术对比的严格性与公平性。

---

## 2. 官方标准基准 (Official Canonical Baselines)

| 字段 | CANONICAL_M1_STATIC | CANONICAL_M1_DYNAMIC |
| :--- | :--- | :--- |
| **Run ID** | `20260918_111527` | `20260918_112354` |
| **Category** | `CANONICAL` | `CANONICAL` |
| **Canonical** | **YES** | **YES** |
| **Controller Mode** | Operating Home Static Hold (Zero-Gain Feedforward) | Base-IMU Full-SO(3) Feedforward Attitude Hold |
| **Duration** | 300.0 s | 120.0 s |
| **Termination Reason** | `scheduled_duration_complete` | `scheduled_duration_complete` |
| **Purpose** | 验证 Frozen Driver Runtime 静态 300s 长期稳定性与双层安全策略可用性 | 验证最终冻结链路在真机低速人工动态扰动下的前馈防抖补偿稳定性 |
| **Evidence Path** | `results/j11/runtime_validation/20260918_111527/` | `results/j11/runtime_validation/20260918_112354/` |
| **Git Tracked** | **YES** (11/12 files tracked; `.log` ignored) | **YES** (11/12 files tracked; `.log` ignored) |
| **Frozen Tag Included** | **YES** (`v1.0-j11-m1-feedforward-hardware-qualified`) | **YES** (`v1.0-j11-m1-feedforward-hardware-qualified`) |
| **关键量化指标** | Writes: 14944; Warnings: 17; Max Age: 305ms; Max Stall: 456ms; Hard STOP: 0; Fault: 0 | Writes: 5929; Base SO(3) Max: 13.54°; Platform Tilt RMS: 1.55°; Max Current: 156mA; Hard STOP: 0; Fault: 0 |

---

## 3. 早期功能成立里程碑 (Functional Milestone)

| 字段 | MILESTONE_M1_FUNCTIONAL |
| :--- | :--- |
| **Run ID** | `j11_3b4` |
| **Category** | `MILESTONE` |
| **Canonical** | **NO** (功能里程碑，非最终性能与运行时基线) |
| **Controller Mode** | Base-IMU Full-SO(3) Feedforward Attitude Hold (`R_BP_cmd = Delta_R_B^T @ R_BP0`) |
| **Duration** | 300.0 s (Active: 110.4 s) |
| **Termination Reason** | `scheduled_duration_complete` |
| **Purpose** | 首次正式证明 Base-IMU Full-SO(3) 前馈防抖功能在真机完全成立 (First Formal Functional Pass) |
| **Evidence Path** | `results/j11/j11_3b4/` |
| **Git Tracked** | **YES** (全部 7 个核心数据与总结文件) |
| **Frozen Tag Included** | **YES** (`v1.0-j11-m1-feedforward-hardware-qualified`) |

---

## 4. 运行时诊断与测试演进数据集 (Runtime Diagnostic & Evolution Datasets)

所有历史排障与中间验证数据均作为工程证据保留，不作为控制性能基准：

| Run ID | Category | Controller Mode | Duration | Termination Reason | Canonical | Purpose | Evidence Path | Git Tracked | Frozen Tag Included |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `20260917_153556` | `RUNTIME_DIAGNOSTIC` | Static Hold | 177.8 s | `operator_interrupt` | NO | RT1 stall 诊断基线与测试注入 | `results/j11/runtime_validation/20260917_153556/` | YES | YES |
| `20260917_155622` | `RUNTIME_DIAGNOSTIC` | Static Hold | 202.2 s | `operator_interrupt` | NO | RT1 底层 UART 驱动 stall 事件量化 | `results/j11/runtime_validation/20260917_155622/` | YES | YES |
| `20260917_173156` | `RUNTIME_DIAGNOSTIC` | Static Hold | 5.8 s | `transport_stall_abort` | NO | RT2 调度器解耦初步测试 (触发停机) | `results/j11/runtime_validation/20260917_173156/` | YES | YES |
| `20260917_193550` | `RUNTIME_DIAGNOSTIC` | Static Hold | 60.0 s | `scheduled_duration_complete` | NO | RT2 遥测解耦 60s 稳态验证 | `results/j11/runtime_validation/20260917_193550/` | YES | YES |
| `20260918_093055` | `RUNTIME_DIAGNOSTIC` | Static Hold | 1.6 s | `feedback_policy_abort` | NO | RT3 反馈单阈值策略测试 (283ms 立即停机) | `results/j11/runtime_validation/20260918_093055/` | YES | YES |
| `20260918_095052` | `RUNTIME_DIAGNOSTIC` | Static Hold | 18.3 s | `feedback_policy_abort` | NO | RT3 反馈恢复策略排障运行 | `results/j11/runtime_validation/20260918_095052/` | YES | YES |
| `20260918_101936` | `RUNTIME_DIAGNOSTIC` | Static Hold | 282.1 s | `feedback_policy_abort` | NO | RT3 最终阈值调整前 282s 运行 (触发 1.0s 停机) | `results/j11/runtime_validation/20260918_101936/` | YES | YES |

---

## 5. 历史探索与中间验证数据集 (Non-Canonical Historical Runs)

| Run ID | Category | Controller Mode | Duration | Termination Reason | Canonical | Purpose | Evidence Path | Git Tracked | Frozen Tag Included |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `j11_pre_motion` | `NON_CANONICAL` | Pre-Motion Offline | N/A | `completed` | NO | 离线运动学与影子闭环积分验证 | `results/j11/pre_motion/` | YES | YES |
| `j11_passive_dry_run` | `NON_CANONICAL` | Passive Dry-Run | 30.0 s | `completed` | NO | 50Hz 遥测与 IMU 数据采集管道验证 | `results/j11/passive_dry_run/` | YES | YES |
| `j11_2a` | `NON_CANONICAL` | Zero-Gain Powered | 30.0 s | `completed` | NO | 上电零增益基线探索测试 | `results/j11/j11_2a/` | NO (.gitignore) | NO |
| `j11_2b` | `NON_CANONICAL` | Roll +0.5° Stationary | 10.0 s | `completed` | NO | 极小角度指令静态响应测试 | `results/j11/j11_2b/` | YES | YES |
| `j11_2b2` | `NON_CANONICAL` | Roll +0.5° Stationary | 10.0 s | `completed` | NO | 极小角度指令重复性测试 | `results/j11/j11_2b2/` | YES | YES |
| `j11_2c1` | `NON_CANONICAL` | Roll +0.5° Manual Dist | 30.0 s | `completed` | NO | 人工扰动下的微小角度响应初探 | `results/j11/j11_2c1/` | YES | YES |
| `j11_3a` | `NON_CANONICAL` | Roll +1.0° Manual Dist | 30.0 s | `completed` | NO | 1.0° 阶跃扰动前馈响应初探 | `results/j11/j11_3a/` | YES | YES |
| `j11_3a1` | `NON_CANONICAL` | Base IMU Axis Qual | 60.0 s | `completed` | NO | Base IMU 与机械坐标轴对齐标定测试 | `results/j11/j11_3a1/` | YES | YES |
| `j11_3b` | `NON_CANONICAL` | Full SO(3) FF (Clamped) | 20.0 s | `completed` | NO | 初步全姿态前馈小范围验证 | `results/j11/j11_3b/` | YES | YES |
| `j11_3b2` | `NON_CANONICAL` | Full SO(3) FF | 80.0 s | `completed` | NO | 扩大角度范围前馈验证 | `results/j11/j11_3b2/` | YES | YES |
| `j11_3b3` | `NON_CANONICAL` | Full SO(3) FF | 5.0 s | `keyboard_interrupt` | NO | 交互退出机制缺陷测试 | `results/j11/j11_3b3/` | YES | YES |
| `j11_m1_reproduction` | `NON_CANONICAL` | Full SO(3) FF Reproduce | 300.0 s | `completed` | NO | M1 功能复现与保护阈值调试运行 | `results/j11/j11_m1_reproduction/` | YES | YES |
| `j11_staged_validation` | `NON_CANONICAL` | Staged Launch Validation | 20.0 s | `completed` | NO | 分步启动脚本流程校验 | `results/j11/j11_staged_validation/` | YES | YES |
| `j11_video_manual` | `NON_CANONICAL` | Video Demo Runs | Multi | `completed` | NO | 视频录制专用手动运行记录 | `results/j11/j11_video_manual/` | YES | YES |
