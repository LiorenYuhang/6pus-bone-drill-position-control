# J5-B3 双 IMU 相对姿态实时估计算法总结与冻结文档 (J5-B3 Final Summary)

**冻结时间**: 2026-08-19 20:40  
**当前状态**: $\mathbf{J5\text{-}B3\ 核心算法与实时链路\ PASS\ /\ FROZEN}$

---

## 1. J5-B3 各子阶段演进与验收矩阵 (Milestones Matrix)

| 子阶段编号 | 任务定义与核心突破 | 核心指标 / 结论 | 状态 |
| :--- | :--- | :--- | :---: |
| **J5-B1** | 软件时间同步 (Software Sync) | 线性时间映射 + 环形插值缓冲（Base PPM: $-14.63$, Plat PPM: $-17.56$），插值时间跨度 $\le 5\text{ ms}$，丢帧率 $0.0\%$ | **PASS / FROZEN** |
| **J5-B3-0** | Quaternion / SO(3) 相对姿态离线基础验证 | 解析双机四元数与旋转矩阵，确立相对姿态李代数误差度量体系 | **PASS** |
| **J5-B3-1** | 共刚体误差源诊断 | 隔离分析时间异步、单轴零偏漂移、尺度因子与坐标系未对准等误差贡献量 | **PASS** |
| **J5-B3-2A** | gyro-SVD 固定 sensor-frame 旋转外参标定 | 利用共刚体旋转角速度流通过 SVD 求解传感器系初始转换矩阵 $\mathbf{R}_{SB,SP}$ | **PASS** |
| **J5-B3-2B** | bias / scale / timing 剩余误差诊断 | 明确 session 间静态陀螺零偏残差为长期相对姿态开环积分漂移的主导因素 | **PASS** |
| **J5-B3-3** | 双 gyro SO(3) 传播算法数学建模 | 确立无预旋转失真的双边李群积分：$\mathbf{R}_{k+1} = \operatorname{Exp}(-\boldsymbol{\omega}_B \Delta t) \cdot \mathbf{R}_k \cdot \operatorname{Exp}(\boldsymbol{\omega}_P \Delta t)$ | **PASS** |
| **J5-B3-4** | 实时 ROS 2 节点实现 | 编写 `h30_relative_attitude_node`，发布高频相对四元数、相对角速度与诊断信息 | **PASS** |
| **J5-B3-4V** | 共刚体动态验证 ($80.8^\circ/\text{s}$) | 19,802 帧 $99.0\text{ s}$ 实机大动态翻转，动态 $\text{SO}(3)$ RMS $0.2767^\circ$，动态后归零残差 $0.3899^\circ$ | **PASS / FROZEN** |
| **J5-B3-5** | Session-Start 启动自动零偏估计 | 启动后连续 6.0s 静止自适应估计 $\hat{\mathbf{b}}_B, \hat{\mathbf{b}}_P$，静止漂移率由 $1.968^\circ/\text{min}$ 降至 **$0.363^\circ/\text{min}$**（降低 81.6%） | **PASS** |
| **J5-B3-5A** | 静止门禁加固与鲁棒截尾均值 | 基于 15 万帧噪声统计设定 $\|\boldsymbol{\omega}\| \le 0.012\text{ rad/s}$ 与 $\sigma_{\text{win}} \le 0.004\text{ rad/s}$ 双门禁 + 5% Trimmed Mean。*(注：算法与离线仿真通过，实机在线晃动打断测试仍为待补小型工程验证项)* | **PASS / 待补实测打断** |

---

## 2. 实时 C++ 节点 vs 离线 B3-3 算法复现一致性 (RT vs Offline)

在不同工况下，实时 C++ 节点输出与离线 Python 算法数学重放的比对结果：

* **407s 纯静态测试** (`j5_b3_4v_20260819_bag`, 81,439 帧)：
  * 四元数各分量最大差值：$< 2.75 \times 10^{-5}$
  * $\text{SO}(3)$ 差异 RMS：**$0.0011^\circ$**
* **$80.8^\circ/\text{s}$ 大动态测试** (`j5_b3_4v_dynamic_20260819_bag`, 19,802 帧)：
  * 四元数各分量最大差值：**$2.86 \times 10^{-3}$**
  * $\text{SO}(3)$ 差异 RMS：**$0.0547^\circ$**（Mean: $0.0206^\circ$，P95: $0.1365^\circ$）

---

## 3. 最终实机动态共刚体验证指标 (Dynamic Co-Rigid Verification)

数据集：`up/sensor/IMU/data/j5_b3_4v_dynamic_20260819_bag`（99.00s，19,802 帧）

* **激励强度**:
  * Base 峰值角速度: **$1.4102\text{ rad/s}$ ($80.80^\circ/\text{s}$)**
  * Platform 峰值角速度: **$1.4072\text{ rad/s}$ ($80.63^\circ/\text{s}$)**
* **相对姿态共模抑制表现**:
  * 初始静止段 (0 ~ 31.11s) $\text{SO}(3)$ RMS: **$0.0497^\circ$** (P95: $0.0805^\circ$)
  * 动态翻转段 (31.11 ~ 56.47s) $\text{SO}(3)$ RMS: **$0.2767^\circ$**
  * 动态翻转段 (31.11 ~ 56.47s) $\text{SO}(3)$ P95: **$0.4776^\circ$**
  * 动态翻转段 (31.11 ~ 56.47s) $\text{SO}(3)$ Max: **$0.5973^\circ$**
  * 归位静止段 (56.47 ~ 99.00s) $\text{SO}(3)$ RMS: **$0.3955^\circ$**
  * 25s 大动态后归零残差: **$0.3899^\circ$** ($23.39\text{ arcmin}$)
  * 全时段有效漂移率: **$0.3762^\circ/\text{min}$** ($22.57^\circ/\text{h}$)

> [!IMPORTANT]
> **边界说明与指标性质**：
> 以上动态指标属于**两只 IMU 固连在同一刚体板上的“共刚体共模抑制验证 (Co-Rigid Common-Mode Rejection)”**，证明了双边时间同步与双边李群积分能够**显著抑制共同空间旋转造成的虚假相对姿态变化**（动态极值误差 $< 0.6^\circ$，相比原厂直出姿态失真降低 92.3%）。
> **该指标不代表 6PUS 机器人在两端分别安装时的最终相对姿态绝对跟踪精度**（后者需结合 6PUS 正运动学 FK 及外参标定）。

---

## 4. 阶段状态修正与后续路线 (Stage Roadmap)

* **J5-B3 (双 IMU 相对姿态核心算法与实时链路)**: $\mathbf{PASS\ /\ FROZEN}$
* **J5-B3-5A 实机运动打断测试**: 作为剩余小型工程验证项，待后续适时回补；
* **J5-B2 (最终安装外参/杆臂标定)**: $\mathbf{DEFERRED}$（待 6PUS 机构本体装配与正运动学联调时执行）；
* **J5-B4 (平动/加速度融合估计)**: $\mathbf{PENDING}$（待完成）；
* **未来优化项 (Future Optimization)**:
  * **Hardware SYNC 硬件同步引脚触发**：当前 Software Sync 精度已达标，后续若硬件布线允许，可接入硬件 GPIO 脉冲同步作为进阶优化。
* **下一步阶段 (Next Step)**:
  * 原文此处是 B3 完成时的历史路线记录；J5-C1、J5-C2-SW、J5-C2-RT-SW 已在后续阶段完成并冻结。当前不直接进入新的功能开发，而是等待完整机构装配后按 `CURRENT_STAGE.md` 进入 J5-C2-HW。

## 5. 当前权威状态补充（后续阶段修正）

* J5-B1 = **PASS / FROZEN**；J5-B2 = **DEFERRED**；J5-B3 = **PASS / FROZEN**；J5-B4 = **PENDING**。
* J5-B3 的实机数据是两只 IMU 固连在同一块测试板上的共刚体验证，不是最终 Base/Platform 安装外参或 lever arm 验收。
* 完整 6PUS 机构尚未装配，因此实机 FK 精度、最终双 IMU 外参、Dual-IMU/FK 融合和 ESKF 尚未开始。
