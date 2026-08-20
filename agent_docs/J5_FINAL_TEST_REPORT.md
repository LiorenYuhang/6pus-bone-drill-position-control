# J5 装配前阶段最终测试与验收总报告 (J5 Final Test & Acceptance Report)

> **测试报告标识**: `TR-6PUS-J5-FINAL-20260820`  
> **基线版本**: `v1.0-j5-pre-assembly-frozen`  
> **报告性质**: 6PUS 并联骨科手术机器人装配前软件架构、驱动协议、独立执行器台架与实时观测链最终测试验收报告  
> **验收结论**: $\mathbf{J5\ PRE\text{-}ASSEMBLY\ BASELINE\ PASS\ /\ FROZEN}$

---

## 1. 验收测试环境与设备清单

### 1.1 主控与操作系统
* **OS**: Ubuntu 24.04 LTS (x86_64, Linux kernel 6.8)
* **Middleware**: ROS 2 Jazzy Jalisco
* **Python Runtime**: Python 3.12.3
* **C++ Compiler**: GCC 13.2.0 (C++17)

### 1.2 物理硬件与接口映射
* **执行器总线**: 6 台推杆电缸（ID1 ~ ID6），单总线半双工 RS485，波特率 921600 8N1
  * 串口设备路径: `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` (`/dev/ttyUSB0`)
  * 安装状态: 独立台架自由状态（未形成并联闭环）
* **Base H30 IMU**: Yesense H30 惯导模块，USB 虚拟串口，波特率 460800 8N1
  * 串口设备路径: `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00` (`/dev/ttyACM0`)
* **Platform H30 IMU**: Yesense H30 惯导模块，USB 虚拟串口，波特率 460800 8N1
  * 串口设备路径: `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00` (`/dev/ttyACM1`)
* **并联机械本体**: **尚未总装**

---

## 2. J5 核心模块测试结果汇总 (Test Results Matrix)

| 测试项编号 | 验证模块 | 验证方式与测试时长 | 核心考核指标 / 判定门禁 | 实测结果与统计证据 | 判定 |
| :---: | :--- | :--- | :--- | :--- | :---: |
| **TR-01** | **工作空间全量单元测试** | `pytest bone_ws/src -v` | 100% 通过，0 失败，0 告警挂起 | **162 passed in 37.02s** (覆盖逆解、正解 LM、FixedLag、驱动、控制、IMU) | **PASS** |
| **TR-02** | **UART 通信与 0 写入只读审计** | 实机长时串口监听 (60.02s) | `writes = 0`，外层轮询 10Hz，通信时延稳定 | 采集 600 周期，`writes = 0`，均值 $18.18\text{ ms}$ (P95 $19.84\text{ ms}$)，跨度 $15.18\text{ ms}$ | **PASS** |
| **TR-03** | **UART Batch 原子性与防交错** | 392 次并发读写压力测试 | 6 轴 Batch 指令严格原子，0 交错，0 乱序 | 连续 Batch Overlap = $0\ /\ 392$ ($100.00\%$ 原子性无交错) | **PASS** |
| **TR-04** | **独立台架 PARK $\to$ HOME 就位** | 50 step/s 实机 S 曲线顶升 | 目标 `[1000]*6`，稳态就位误差 $\le 2\text{ steps}$ | 最终步数 `[999, 999, 999, 999, 998, 999]`，最大误差 $2\text{ 步}$ | **PASS** |
| **TR-05** | **独立台架 HOME $\to$ PARK 收拢** | 50 step/s 实机 S 曲线缩回 | 目标 `[0]*6`，稳态收拢误差 $\le 2\text{ steps}$ | 最终步数 `[1, 1, 0, 0, 0, 0]`，最大误差 $1\text{ 步}$ | **PASS** |
| **TR-06** | **J4 连续笛卡尔动态控制回归** | J4 Heave 5.0s 动态轨迹 | 0 安全违规，0 故障停机，平稳回泊 | 连续 Heave $\pm 2.0\text{ mm}$ 完成，0 Faults，0 前检违规，平稳收拢 | **PASS** |
| **TR-07** | **双 H30 0x51 硬件微秒时间戳** | 双路 15.0s 连续数据流比对 | 双路 100% 具备 0x51，步长恒定 5000us | Base (195.4Hz) 2931/2931 具备；Platform (195.4Hz) 2931/2931 具备；0 丢包 | **PASS** |
| **TR-08** | **Session-Start 自动零偏估计** | 16.0s 静态自校准 | 6.0s 收集 1200 样本，5% 截尾均值收敛 | 控制台输出 `Auto gyro bias init COMPLETE (1200 samples, 6.00 s)`，状态变迁成功 | **PASS** |
| **TR-09** | **B3 相对姿态实时解算** | 16.0s 实时节点输出 | $\ge 100\text{ Hz}$ 稳定发布，同板动态抑制 | 发布率 **$122.06\text{ Hz}$**，单步 $4997.4\mu\text{s}$，同板 $80.8^\circ/\text{s}$ 动态下 $\text{SO}(3)$ RMS $0.2767^\circ$ | **PASS** |
| **TR-10** | **Timed Telemetry 统一时间轴** | 15.0s 驱动下发 (25Hz) | 纳秒时间戳严格单调，Command-Feedback 配对 | 374 帧 `TimedJointFeedback` 100% 单调递增，无回退无乱序 | **PASS** |
| **TR-11** | **Fixed-Lag 状态重建与抖动分析** | 15.0s 100Hz 查询 (1500次) | 严格零外推门禁，WAIT 分布符合物理机理 | $1456\text{ 次}$ SYNCED ($97.07\%$)，启动填冲 11 次，运行期抖动等待 33 次 (2.20%) | **PASS** |
| **TR-12** | **Pus6FkNode 实时解算观测链** | 15.0s 独立单线程执行器 | 独立执行器无饿死，解算率稳定，残差 $< 0.1\text{mm}$ | 解算率 $90.6\text{ Hz}$，几何残差均值 $8.65\times 10^{-12}\text{ m}$，进程干净退出 | **PASS** |

---

## 3. 专项验证与时序机理分析

### 3.1 READ_ONLY 通信时序三要素分离
依据驱动源码与 60s 实验日志，严格界定三个独立时序指标：
* **外层轮询/发布频率**: 设定 10.0Hz（定时器周期 100ms），实际 $600\text{ cycles} / 60.02\text{ s} = \mathbf{9.996 \approx 10.0\text{ Hz}}$；
* **单次六轴 Snapshot 串行通信用时**: 均值 $\mathbf{18.18\text{ ms}}$，P50 $18.07\text{ ms}$，P95 $19.84\text{ ms}$（总线物理吞吐上限 $\approx 55\text{ Hz}$）；
* **ID1 $\to$ ID6 扫描物理跨度 (Scan Span)**: 均值 $\mathbf{15.18\text{ ms}}$。

### 3.2 Fixed-Lag 零外推机制与 WAIT 产生机理
在 100Hz 查询（$t_q = t_{\text{now}} - 60\text{ ms}$）下：
1. **启动阶段缓冲填冲** ($t \le 200\text{ ms}$)：产生 11 次 `EMPTY_BUFFER` / `WAIT_TOO_EARLY`，为队列填充至少 2 帧的物理冷启动过程；
2. **运行阶段通信抖动** ($t > 200\text{ ms}$)：产生 33 次 `WAIT_FUTURE_SAMPLE`（占比 2.20%）。当 25Hz 串口轮询由于系统调度产生 $13\sim 32\text{ ms}$ 抖动时，在查询时刻 $t_q$ 处，最新的反馈帧尚未到达，导致右边界 $t_i[k] \ge t_q$ 缺失。
3. **零外推保证**：`FixedLagJointSynchronizer` 坚决拒绝外推插值，立即返回 `WAIT_FUTURE_SAMPLE`；待 5~10ms 后新帧到达即刻恢复 `SYNCED`。

### 3.3 Platform H30 0x51 时间戳修复确证
* **根因确诊**：Platform 内部输出配置寄存器（Class 0x04）掩码曾为 `0x00D8`（Bit12 = 0，关闭 0x51/0x52）；
* **固化修复**：下发 `59 53 04 12 00 d8 80 6e 8c` 写入 Flash 固化 `0x80D8` 掩码；
* **实测验证**：双路 15s 2931 帧实现 100.00% `has_sample_timestamp`，步长恒定 5000us，驱动启动 6.0s 顺利触发 `Auto gyro bias init COMPLETE`。

---

## 4. J5 完成范围 vs J6 延期范围严格界限

```
   ┌───────────────────────────────────────────────────────────────────────┐
   │                          J5 阶段已完成范围                            │
   │  • ROS 2 控制与观测双链路架构                                         │
   │  • 单总线 RS485 921600 驱动、Batch Atomicity 全局锁与 0 写入只读审计  │
   │  • 6pus_geometry.yaml 唯一权威真值源加载机制                          │
   │  • 独立执行器台架 S 曲线 PARK ([0]*6) ↔ HOME ([1000]*6) 闭环就位     │
   │  • J4 Jerk-limited Shaper、LVW 缓冲与 Heave 动态台架回归              │
   │  • 双 H30 IMU 0x51 硬件时间戳、Software Sync 与 Auto Bias 自适应估计   │
   │  • B3 双 IMU 相对姿态实时解算 (122Hz) 与同板动态共模抑制              │
   │  • TimedJointFeedback 统一时间轴与 FixedLag 严格零外推状态重建        │
   │  • 解析雅可比 LM 正运动学求解器与 Pus6FkNode 单线程实时观测节点       │
   │  • 核心功能包 162 项单元测试 100% PASS                                │
   └───────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼ [物理分界线：机械本体物理总装]
   ┌───────────────────────────────────────────────────────────────────────┐
   │                          J6 阶段明确延期范围                          │
   │  • 完整 6PUS 机械本体物理总装 (J6 Phase 0)                            │
   │  • 真实机构机械零位、连杆与安装角实测复核 (J6 Phase 2/6)             │
   │  • 极小单轴微动 (≤1mm) 确认方向与被动球铰无干涉 (J6 Phase 3)         │
   │  • 机械总装后完整机构可达性与小幅协调运动 (J6 Phase 4/5)              │
   │  • J5-C2-HW 真实机构正运动学 FK 精度与物理位姿验收 (J6 Phase 7)       │
   │  • 双 IMU 最终安装、J5-B2 空间外参标定与 Lever Arm 标定 (J6 Phase 8)  │
   │  • J5-B4 平动模型、加速度融合估计与 Dual-IMU/FK ESKF (J6/J7 阶段)    │
   └───────────────────────────────────────────────────────────────────────┘
```

---

## 5. 测试验收结论

J5 阶段所有规划的装配前软件功能包、驱动协议、独立执行器台架运动控制与实时观测链路已全部达到验收指标，技术链路闭环，证据完备。

**最终结论**：$\mathbf{TR\text{-}6PUS\text{-}J5\text{-}FINAL\quad PASS\ /\ FROZEN}$。
