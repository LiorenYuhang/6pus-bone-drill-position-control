# 6PUS 骨科并联机器人防抖控制与感知框架

> [English](README.md) | [中文]  
> **面向 6-PUS 并联骨科手术机器人的 ROS 2 实时感知与主动防抖控制系统**

---

## 1. 项目简介 (Project Overview)

**6PUS-Anti-Vibration** 是专为 6-PUS（移动副-万向节-球副）Stewart 并联机构研发的开源高精度实时感知与运动控制框架。项目面向骨科手术机器人领域的高精细操作，旨在通过实时位姿解算、加加速度连续轨迹整形与高频伺服通信，实现平台在动基座扰动条件下的主动抗扰与高带宽振动抑制。

### 核心阶段划分：
- **J1–J10（已完成并冻结）**：构建了坚实的真实物理硬件运动与感知基线，涵盖多电机底层串口通信、微秒级闭式解析逆解、三阶加加速度连续轨迹平滑整形、50Hz 连续流式伺服驱动、双 H30 IMU 微秒级时钟对齐解算、数值正运动学重构，以及在真实物理平台上完成的 6 自由度单轴安全运行包络边界全域测定。
- **J11-M1（已完成并冻结）**：完成 Base-IMU 全 SO(3) 前馈姿态保持实机闭环、专用驱动层运行资格认证、两级安全策略 (Two-Tier Safety Policy)、300s 静态长周期达标以及真机低速动基座扰动防抖补偿验证（官方 Release Tag: `v1.0-j11-m1-feedforward-hardware-qualified`）。
- **J11-M2+（主动防抖研究推进）**：正在推进 J11-M2 双 IMU PID 残差姿态闭环稳定控制；后续演进路线包括自抗扰控制 (ADRC) 与自适应控制。

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
│  • J11-M2: 双 IMU 任务空间 PID 残差姿态稳定控制 [当前推进阶段]          │
│  • 后续前沿研究路线 (Roadmap)：                                         │
│    - 自抗扰控制 (ADRC) 与扩张状态观测器 (ESO) 总扰动实时估计与补偿      │
│    - IMU 角加速度前馈与扰动观测器 (DOB) 抑制高频动基座振颤              │
│    - 强化学习与自适应在线刚度自调优控制                                 │
└─────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **真实性与验证边界**：阶段 J1 至 J10 以及 J11-M1 阶段已全部在真实物理硬件上完成实机闭环验收并正式冻结（官方 Release Tag: `v1.0-j11-m1-feedforward-hardware-qualified`）。当前推进阶段为 J11-M2（双 IMU PID 残差姿态闭环控制）。严禁虚构或提前宣称未经验证的研究结论。

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

### J11 姿态防抖控制实机评定结果 (M1 vs. 达标 3DOF P 基线)

J11-M2 阶段正式引入了**任务空间 $SO(3)$ 姿态 PID 控制框架**。三轴姿态同时闭环比例反馈（**J11-M2-4 3DOF P 集成验证**）已在真实六自由度并联机构上完成动基座低速自然扰动实机闭环验证。

> [!NOTE]
> **J11-M2 当前实际控制状态与基线**：
> - **控制结构**：$\mathbf{\text{3DOF Base-IMU } SO(3) \text{ 前馈}} + \mathbf{\text{3DOF Platform-IMU } SO(3) \text{ P 反馈}}$
> - **当前权威达标基线**：$K_{p\_roll} = 0.05, K_{p\_pitch} = 0.05, K_{p\_yaw} = 0.05$（$K_i = 0, K_d = 0$，残差限幅 $\pm 0.5^\circ$）
> - **调参状态**：M2-5 增益整定进行中。Candidate 1（$K_{p\_yaw} = 0.10$）经实机验证因系统级性能退化而不予保留。
> - *由于采用人工动基座扰动，输入扰动能量并非严格一致，因此控制器横向评定严格采用归一化抑制比（$G_{\text{Peak}}, G_{\text{RMS}}$）。*

| 旋转轴 | 评定指标 | M1 纯前馈 (`20260918_112354`) | M2 达标 3DOF P 基线 (`20260918_152354`) | 归一化抑制提升率 |
| :--- | :--- | :---: | :---: | :---: |
| **Roll** | $G_{\text{Peak}}$ / $G_{\text{RMS}}$ | 0.4778 / 0.4071 | 0.4900 / 0.3066 | -2.55% (峰值) / **+24.68% (RMS)** |
| **Pitch** | $G_{\text{Peak}}$ / $G_{\text{RMS}}$ | 0.5946 / 0.4153 | 0.3110 / 0.2705 | **+47.69% (峰值)** / **+34.87% (RMS)** |
| **Yaw** | $G_{\text{Peak}}$ / $G_{\text{RMS}}$ | 0.7983 / 0.2912 | 0.9030 / 0.3374 | -13.12% (峰值) / -15.87% (RMS) *(调参目标)* |

*完整权威评定档案、李代数误差推导与 50Hz 实机遥测记录详见：* [`agent_docs/j11/J11_M1_M2_CONTROL_COMPARISON.md`](./agent_docs/j11/J11_M1_M2_CONTROL_COMPARISON.md)。

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
├── agent_docs/                            <-- [项目知识与文档库] 人类与 Agent 可读的技术方案、架构与报告
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
├── STM32/ / teach/                        <-- [固件与示教器] 下位机工程源码与示教工具
├── 6PUS_REAL_HARDWARE_OPERATION_GUIDE.md  <-- [实机操作入口] 实机上电、回零、运动与关机速查指南
└── AGENTS.md                              <-- [全局 Agent 规则] 操作规范、安全红线与工程不变量
```

> **职责边界原则**：
> - `agent_docs/` = 人类与 Agent 可读的架构知识、阶段方案与分析总结。
> - `results/` = 机器程序运行、测试和实机实验产出的 CSV、遥测 JSON 与运行原始日志。

---

## 4. 推荐阅读与学习路径 (Recommended Reading Order)

对于首次接触本项目的工程师、研究者或智能体，建议按如下次序阅读：

1. **宏观概览与基线**：首先通读本文档 [`README_CN.md`](./README_CN.md)（或 [`README.md`](./README.md)），掌握系统整体架构与阶段进展。
2. **ROS 2 全栈技术教程**：研读 [`6PUS_ROS2_CODE_LEARNING_TUTORIAL.md`](./6PUS_ROS2_CODE_LEARNING_TUTORIAL.md)，掌握双链路解耦设计、闭式逆解、LM 数值正解及 50Hz 伺服驱动的详细实现。
3. **权威文档检索地图**：参考 [`agent_docs/README.md`](./agent_docs/README.md)，快速定位具体阶段的技术方案、交接规范与硬件准入报告。
4. **物理实机操作与安全**：若需操作物理样机，务必在操作前核对 [`6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`](./6PUS_REAL_HARDWARE_OPERATION_GUIDE.md)。
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
全部测试均在 `hardware_mode:=mock` 下运行，且**代码级严格断言 $\mathbf{\text{hardware\_write\_count} = 0}$**：
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
