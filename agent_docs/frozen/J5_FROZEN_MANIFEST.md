# J5 最终冻结清单 (J5 Final Frozen Manifest)

> **Manifest 状态**: $\mathbf{J5\ PRE\text{-}ASSEMBLY\ BASELINE\ FROZEN}$  
> **冻结日期**: 2026-08-20  
> **版本标识**: `v1.0-j5-pre-assembly-frozen`  
> **适用范围**: 6PUS 装配前软件架构与独立执行器台架验证收尾基线  
> **下一阶段入口**: [`agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md)（J6 POST-ASSEMBLY COMMISSIONING）

---

## 1. 冻结版本定位与边界定义

J5 阶段定位为 **装配前软件全链路与独立执行器台架验证基线（PRE-ASSEMBLY SOFTWARE & BENCH BASELINE）**。

### 1.1 J5 包含内容 (Included Scope)
* **ROS 2 控制软件架构**：`pus6_msgs`, `pus6_kinematics`, `pus6_motor_driver`, `pus6_control`, `pus6_bringup`, `h30_imu`；
* **UART 六电机通信与安全链**：单总线半双工 RS485 921600 8N1，CRC16 校验，0x29 伴随反馈，Batch-Atomicity 全局总线锁与 6 级安全硬门禁；
* **J2 / J3 / J4 控制阶段**：
  * J2：六轴同步通信与 0 写入只读审计（10Hz 轮询，18.18ms 串行通讯耗时）；
  * J3：解析逆解、S 曲线规划、独立电缸 PARK (`[0]*6`) $\leftrightarrow$ HOME (`[1000]*6`) 闭环就位；
  * J4：Jerk-limited Stateful Shaper、Latest-Value-Wins (LVW) 缓冲、UART batch-atomicity 全局锁机制与 J4 Heave 独立台架回归；
* **J5-A 几何配置唯一真值源**：`6pus_geometry.yaml` 统一定义（$c=40\text{mm}, d=27.5\text{mm}, e=84\text{mm}, \lambda=30^\circ, \mu=60^\circ$）；
* **J5-B1 / J5-B3 双 H30 IMU 链路与相对姿态**：
  * 0x51 硬件采样微秒时间戳全面启用（双路 100% 具备，5000us 步长，0 丢包）；
  * Software Sync 时钟频偏消除（Base $-14.63\text{ PPM}$, Plat $-17.56\text{ PPM}$）；
  * Session-Start 6.0s 自动陀螺零偏估计（Auto Bias COMPLETE，1200 样本，5% 截尾均值收敛）；
  * 李群 $\text{SO}(3)$ 双边积分相对姿态解算（122Hz 实时发布）；
  * 同板大动态共模抑制验证（$80.8^\circ/\text{s}$ 下动态 RMS $0.2767^\circ$）；
* **J5-C1 Timed Telemetry 统一时间轴**：Host `CLOCK_MONOTONIC` 纳秒时间戳、`TimedJointFeedback` / `TimedJointCommand` 语义、UART 周期 1-to-1 配对；
* **J5-C2-SW 数值正运动学 (FK)**：解析雅可比 LM 求解器、IK→FK 数值一致性（残差 $< 10^{-8}\text{ mm}$）、Warm Start 与分支保护；
* **J5-C2-RT-SW FK 实时观测节点**：`Pus6FkNode` 独立 `SingleThreadedExecutor` 硬约束（200Hz 解算，FixedLag 严格零外推门禁，残差均值 $< 10^{-11}\text{ m}$）；
* **六电机独立台架真实运行验证**：RS485 总线 0 写入 READ_ONLY 审计、台架电缸 PARK $\leftrightarrow$ HOME 就位与 J4 动态回归。

### 1.2 J5 明确不包含内容 (Excluded / Deferred to J6+)
* 完整 6PUS 并联机构机械本体总装；
* 真实机构正运动学 FK 精度验收与物理定位精度测量；
* 最终 Base / Platform 双 IMU 安装外参标定（J5-B2）；
* Lever Arm 标定；
* 平动模型与加速度融合估计（J5-B4）；
* Dual-IMU / FK 多传感器融合与 ESKF 滤波；
* 被动球铰机械干涉与全域物理极限工作空间验证。

---

## 2. 硬件范围与当前物理状态 (Hardware Scope)

| 硬件子系统 | 物理连接与安装状态 | 串口设备 By-ID 路径 | 当前工作模式与验证状态 |
| :--- | :--- | :--- | :---: |
| **6 台推杆电缸** | ID1 ~ ID6 挂载于单总线 RS485，处于**台架自由状态**（未组装并联机械闭环） | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` (921600) | READ_ONLY 0 写入 PASS；<br>台架 PARK $\leftrightarrow$ HOME 就位 PASS |
| **Base H30 IMU** | 固定于同一测试板（同板测试） | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00` (460800) | 195.4Hz，0x51 时间戳 100% PASS |
| **Platform H30 IMU**| 固定于同一测试板（同板测试） | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00` (460800) | 195.4Hz，0x51 时间戳 100% PASS |
| **6PUS 机械本体** | **尚未总装** | N/A | **待现场机械装配** |

---

## 3. 已验证功能与指标矩阵 (Verified Capabilities)

| 验证模块 | 验证方式 | 核心指标与实测证据 | 状态 |
| :--- | :--- | :--- | :---: |
| **六电机 READ_ONLY** | 60.02s 串口只读监听 | 600 周期，`writes = 0`，外层轮询 10.0Hz，通信耗时 18.18ms，扫描跨度 15.18ms | **PASS / FROZEN** |
| **独立台架 HOME 就位**| 50 step/s 实机顶升 | 最终步数 `[999, 999, 999, 999, 998, 999]`，误差 $\le 2\text{ steps}$ | **PASS / FROZEN** |
| **独立台架 PARK 收拢**| 50 step/s 实机缩回 | 最终步数 `[1, 1, 0, 0, 0, 0]`，误差 $\le 1\text{ step}$ | **PASS / FROZEN** |
| **J4 连续控制回归** | J4 Heave 5.0s 动态轨迹 | 0 安全违规，0 故障代码，平稳完成并安全回泊 | **PASS / FROZEN** |
| **双 H30 硬件时间戳** | 15.0s 串口报文比对 | 双路 0x51/0x52 采样时间戳 100.00% 输出，步长恒定 5000us，TID 0 丢包 | **PASS / FROZEN** |
| **B3 Auto Bias 估计** | 16.0s 实时节点运行 | 6.0s 完成 1200 样本 5% 截尾均值收敛，状态变迁为 COMPLETE | **PASS / FROZEN** |
| **B3 相对姿态实时发布** | 16.0s 实时节点运行 | 122.06Hz 稳定发布，单步步长 4997.4us，同板大动态共模抑制 RMS $0.2767^\circ$ | **PASS / FROZEN** |
| **FixedLag 零外推对齐**| 15.0s 100Hz 查询 (1500次)| 97.07% SYNCED，微观通信抖动下严格拒绝外推触发 WAIT_FUTURE_SAMPLE (2.20%) | **PASS / FROZEN** |
| **Pus6FkNode 实时解算** | 15.0s 实时节点运行 | 90.6Hz 解算，残差均值 $8.65\times 10^{-12}\text{ m}$，单线程执行器稳定运行 | **PASS / FROZEN** |
| **核心单元测试集** | `pytest bone_ws/src -v` | **162 passed in ~39.85s (100% 通过)** | **PASS / FROZEN** |

---

## 4. 已知限制与安全红线 (Known Limitations & Red Lines)

> [!CAUTION]
> **终审安全红线与物理边界说明**：
> 1. **严禁将数值/台架测试升级为机构精度**：IK↔FK 仿真一致性（$< 10^{-8}\text{ mm}$）与独立电缸台架测试仅证明软件算法与驱动协议自洽，**绝不代表真实物理机器人的加工制造精度、球铰间隙或末端定位精度**。
> 2. **严禁在机械总装前盲目执行高速运动**：所有涉及真实机械机构的测试必须等待物理总装完成。
> 3. **禁止提前启动 J6 融合项**：在机构总装与 J5-C2-HW 验收前，不得提前开展 Dual-IMU/FK 融合、ESKF 滤波或平动加速度估计。

---

## 5. J6 POST-ASSEMBLY COMMISSIONING 入口与路线

完整 6PUS 机械本体总装完成后，严格按 [`agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`](file:///home/lyapunov/bone_drill/agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md) 的 9 个阶段开展现场调试：

```text
Phase 0 (机械静态检查，完全不上电)
   │
   ▼
Phase 1 (电气与串口 READ_ONLY 审计，0 写入)
   │
   ▼
Phase 2 (六轴 ID、反馈与零位映射确认)
   │
   ▼
Phase 3 (【首次真实运动】极小单轴微动确认方向与无干涉，≤1mm，50 step/s)
   │
   ▼
Phase 4 (STORAGE_PARK / OPERATING_HOME 机械可达性与就位)
   │
   ▼
Phase 5 (六轴协调小幅笛卡尔运动测试)
   │
   ▼
Phase 6 (最终机械几何参数实测与 YAML 冻结)
   │
   ▼
Phase 7 (J5-C2-HW 真实机构正运动学 FK 验收)
   │
   ▼
Phase 8 (双 IMU 最终安装、J5-B2 外参标定与 Lever Arm 标定)
```
