# 6PUS 骨科并联机器人防抖控制与感知框架

> [English](README.md) | [中文]  
> **面向 6-PUS 并联骨科手术机器人的 ROS 2 实时感知与主动防抖控制系统**

---

## 1. 项目简介 (Project Overview)

**6PUS-Anti-Vibration** 是专为 6-PUS（移动副-万向节-球副）Stewart 并联机构研发的开源高精度实时感知与运动控制框架。项目面向骨科手术机器人领域的高精细操作，旨在通过实时位姿解算、加加速度连续轨迹整形与高频伺服通信，实现平台在动基座扰动条件下的主动抗扰与高带宽振动抑制。

### 核心阶段划分：
- **J1–J10（已完成并冻结）**：构建了坚实的真实物理硬件运动与感知基线，涵盖多电机底层串口通信、微秒级闭式解析逆解、三阶加加速度连续轨迹平滑整形、50Hz 连续流式伺服驱动、双 H30 IMU 微秒级时钟对齐解算、数值正运动学重构，以及在真实物理平台上完成的 6 自由度单轴安全运行包络边界全域测定。
- **J11-M1（已完成并冻结）**：完成 Base-IMU 全 SO(3) 前馈姿态保持实机闭环、专用驱动层运行资格认证、两级安全策略 (Two-Tier Safety Policy)、300s 静态长周期达标以及真机低速动基座扰动防抖补偿验证（官方 Release Tag: `v1.0-j11-m1-feedforward-hardware-qualified`）。
- **J11（最终冻结就绪）**：最终保留 Base-IMU SO(3) Feedforward + Platform-IMU Classical P/PD Attitude Feedback；$K_p=[0.05,0.05,0.025]$，$K_d=[0,0.0015,0.0015]\,\text{s}$，$K_i=0$，$f_c=5\,\text{Hz}$（Roll=P，Pitch=PD，Yaw=PD）。Roll-D 调参已关闭；Demo A（Feedforward）PASS，Demo B（Feedforward + P/PD）PASS ON REPEAT。下一阶段：J12 ADRC。

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
│  • J11-M2 Final: Roll=P, Pitch=PD, Yaw=PD                                │
│             Kp=[0.05, 0.05, 0.025], Kd=[0, 0.0015, 0.0015] s, Ki=0      │
│             fc=5.0 Hz；Roll-D qualified repeat 未保留，调参关闭          │
│  • 当前：FINAL FREEZE READY；Demo A PASS，Demo B PASS ON REPEAT         │
│  • 下一阶段 J12：外部负载/力扰动下的 ADRC 抗扰                          │
│    - ADRC 与扩张状态观测器 (ESO) 总扰动实时估计与补偿                    │
│    - IMU 角加速度前馈与扰动观测器 (DOB) 抑制高频动基座振颤              │
│    - 强化学习与自适应在线刚度自调优控制                                 │
└─────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **真实性与验证边界**：J1 至 J10 与 J11-M1 已冻结；J11-M2 最终 P/PD 控制器具有已有 qualified hardware evidence。最终两个 Demo smoke test 均已执行（Demo A PASS；Demo B PASS ON REPEAT —— 首次 Demo B 因 motor CH340 USB-UART 瞬时 disconnect 提前结束，见 [`agent_docs/j11/J11_DEMO_B_USB_UART_FAULT_AUDIT.md`](./agent_docs/j11/J11_DEMO_B_USB_UART_FAULT_AUDIT.md)），也未宣称 J12 ADRC 的未验证性能。

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

- M1 canonical：`results/j11/runtime_validation/20260918_112354/J11_DISTURB_QUAL_feedforward_powered.csv`
- M2 representative：`results/j11/m2_pid/20260920_095850/j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_telemetry_powered.csv`

未来作图、重算 Peak / RMS / G_Peak / G_RMS 均可直接以上述 canonical raw CSV 为起点。

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

**J11 状态**：FINAL FREEZE READY —— Demo A（Feedforward）PASS；Demo B（Feedforward + P/PD）PASS ON REPEAT。P/PD Demo 首次运行曾出现一次独立 motor USB-UART 瞬时掉线，repeat 正常；该事件作为非阻塞硬件可靠性事项归档。**下一阶段**：J12 外部负载/力扰动下的 ADRC disturbance rejection。

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
