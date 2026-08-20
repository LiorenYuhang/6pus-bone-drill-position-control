# 6PUS ROS2 & Motor Control — Agent Operational Rules

本文档为参与 6PUS 并联骨科手术机器人开发与维护的所有 Agent（包括 Codex / ChatGPT / Claude / Antigravity）的**全局操作规范与行为准则**。

---

## 1. 文档架构与职责边界 (Separation of Concerns)

为了避免同一事实在多处重复维护造成版本冲突，明确如下职责分工：

```
/home/lyapunov/bone_drill/
├── 6PUS_REAL_HARDWARE_OPERATION_GUIDE.md  <-- [用户实机总入口] 唯一权威实机操作与命令速查指南 (Root Entry)
├── AGENTS.md                              <-- [Agent 工作规则] Agent 操作安全红线、测试纪律与工作准则 (本文档)
├── agent_docs/                            <-- [阶段权威方案与交接] 项目全局架构、各阶段 (J2/J3/J4) 执行方案与设计归档
│   ├── BONE_WS_ROS2_ARCHITECTURE_PLAN.md
│   ├── J2-A_ROS2_READONLY_REAL_HARDWARE_PLAN.md
│   ├── J2-B_ROS2_SINGLE_MOTOR_REAL_HARDWARE_PLAN.md
│   ├── J2-B2_ROS2_SINGLE_MOTOR_POSITION_TRACKING_PLAN.md
│   ├── J2-C_ROS2_SIX_AXIS_VECTOR_REAL_HARDWARE_PLAN.md
│   ├── J2-F0_ROS2_READONLY_COMMUNICATION_PERFORMANCE_PLAN.md
│   ├── J3_IK_REAL_HARDWARE_INTEGRATION_PLAN.md
│   ├── J4_ROS2_CONTINUOUS_CARTESIAN_CONTROL_PLAN.md
│   └── ROS2_UART_EXISTING_CODE_AUDIT.md
├── bone_ws/                               <-- [正式 ROS2 工作空间] 源代码 (src/)、自动化测试与实机运行脚本 (scripts/)
├── hardware/                              <-- [硬件与几何参数] 硬件规格书、机构几何解析定义 (Kinematics.zip)
├── up/                                    <-- [UART 底层验证基线] 原始独立串口测试代码、日志与基线协议
└── STM32/ / teach/                        <-- [固件与示教器] 下位机工程与示教工具
```

### 职责边界规则：
1. **`AGENTS.md` (本文档)**：负责 Agent 工作规则、安全底线、编码约束与环境纪律。**不记录具体的硬件时序数值或某个特定阶段的执行日志**。
2. **`agent_docs/`**：负责技术方案、架构规范、算法数学公式、阶段规划与交接总结。**后续所有新生成的方案和阶段设计文档默认统一放入 `agent_docs/`**。
3. **`6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`**：面向操作人员的实机开机、回零、运动与关机速查表。**保留在根目录原位**。

---

## 2. Agent 核心安全红线 (Safety Guardrails)

1. **实机运动零误动原则**：
   - 默认任何代码运行、自动化测试、基准评测均必须在 **Mock / FakeBus 模式** 下进行（`hardware_mode:=mock`）。
   - 严禁在未经用户明确书面授权时执行任何带有 `--real-hardware` 或 `hardware_mode:=active` 的物理写操作。
   - 每次无硬件验证必须显式核对并回报 $\mathbf{\text{hardware\_write\_count} = 0}$。
2. **底层协议冻结**：
   - `pus6_motor_driver` 中对应底层 UART 通信的 `protocol.py`、`uart_bus.py`、`motor.py` 视为通信基线，严禁擅自修改。
3. **零重试写命令 (Zero-Retry on Motion Command)**：
   - 写运动命令（0x29 目标寄存器、0x25 阻塞模式）严禁自动重试；
   - 发生通信异常时，必须由驱动节点向各轴发送一次暂停（0x1A），并在确认完成后优雅退出。
4. **诚实与数据真实性**：
   - 严格区分 Mock 软件测试结果与物理硬件实测结果。
   - 绝不允许将 Mock 调度通过等同于 50 Hz 物理硬件实机通过。
   - 报告中的耗时、时延、超调、抖动指标必须直接提取自真实脚本运行日志，严禁主观推测或美化数据。

---

## 3. 核心技术约定 (Technical Standards)

1. **单位制**：
   - 上层 ROS2 节点内部一律严格采用 **SI 国际标准单位制**（位置：米 $\text{m}$，速度：$\text{m/s}$，角度：弧度 $\text{rad}$，角速度：$\text{rad/s}$，电流：安培 $\text{A}$，力：牛顿 $\text{N}$）；
   - 底层电缸寄存器交互时显式转换（$1\text{ mm} = 40\text{ step}$，$1\text{ step} = 0.025\text{ mm}$）。
2. **坐标基准**：
   - 6PUS 机构运动学、逆解与笛卡尔控制一律以 **`OPERATING_HOME = [1000]*6` ($z_{\text{nom}} \approx 132.6123\text{ mm}$)** 为软件零位原点。
   - `STORAGE_PARK = [0]*6` 仅用于机械收拢与关机断电，严禁在此位置直接运行姿态控制算法。
3. **调度与解耦**：
   - ROS2 订阅回调一律采用 **Latest-Value-Wins (无锁/单槽原子覆盖)**，严禁在 Subscriber Callback 中执行阻塞式硬件通信，严禁累积 FIFO 队列。
   - 驱动节点内建 **Stale-Target Watchdog**，在上层目标流中断时自动挂起写流，严禁无限重发陈旧目标。
