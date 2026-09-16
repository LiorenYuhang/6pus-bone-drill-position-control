# 6PUS ROS2 & Motor Control — Agent Operational Rules

本文档为参与 6PUS 并联骨科手术机器人开发与维护的所有 Agent（包括 Codex / ChatGPT / Claude / Antigravity）的**全局操作规范与行为准则**。

---

## 1. 文档架构与职责边界 (Separation of Concerns)

为了避免文档分散与事实在多处重复维护造成版本冲突，明确如下职责分工：

```
/home/lyapunov/bone_drill/
├── 6PUS_REAL_HARDWARE_OPERATION_GUIDE.md  <-- [根目录实机操作入口] 用户实机操作与命令速查指南
├── AGENTS.md                              <-- [根目录 Agent 规则] 操作规则、安全约束、变更纪律与工程不变量 (本文档)
├── agent_docs/                            <-- [文档与方案唯一根] 架构、阶段方案、设计决策、分析报告与归档
├── results/                               <-- [机读实验与运行数据唯一根] 自动化测试与实机运行生成的 CSV、JSON、日志等证据
├── bone_ws/                               <-- [正式 ROS2 工作空间] 源代码、测试与运行脚本
├── hardware/                              <-- [硬件与几何数据] 硬件规格书与机构几何数据
├── up/                                    <-- [UART 底层验证基线] 原始独立串口测试代码与基线协议
└── STM32/ / teach/                        <-- [固件与示教器] 下位机工程与示教工具
```

### 文档与数据资产存放治理规则 (Documentation & Artifact Location Policy)：
1. **分类基准**：目录归属以**“资产用途 (Asset Purpose)”**为唯一判定依据，严禁仅凭文件扩展名作机械划分。
2. **`agent_docs/` (文档与知识唯一根)**：定位为 **human/Agent-readable project knowledge and documentation**。负责系统架构、阶段方案、设计决策、分析与测试报告（如 `agent_docs/reports/`）、Frozen Manifest 与操作交接指南等所有人类或 Agent 撰写与维护的知识沉淀。
3. **`results/` (运行数据与证据唯一根)**：定位为 **machine-generated / experiment-generated evidence and runtime artifacts**。专门存放程序运行、自动化测试、仿真评测及真实实机实验生成的各类数据证据与运行时产物（包括传感器遥测、电机反馈、轨迹 CSV、运行日志、环境元数据等）。`results/` 专用于存放数据产物，严禁在其中撰写独立的分析文档。
4. **根目录整洁与禁令 (Root Hygiene)**：根目录 `reports/` 已正式废弃并移除。严禁 Agent 在根目录新建随意命名的文档或输出目录（例如根目录 `docs/`、`reports/`、`notes/`、`summaries/` 等）。
5. **资产隔离原则**：避免将知识库文档混入 `results/`，也避免将大体量机读证据/运行时日志倾倒至 `agent_docs/`。
6. **创建与迁移纪律**：
   - Agent 产出任何新资产前，必须先查阅上述目录树规范，严禁直接在根目录下创建未知目录。
   - 迁移或调整旧数据前，必须先全局检索被引用的代码与文档，确保相对路径同步修正，杜绝破损链接。
7. **`AGENTS.md` (本文档)**：仅负责 Agent 全局操作规则、安全约束、变更纪律与工程不变量。不得记录实验日志、数值结果、验证表格、阶段结论、控制器参数或具体算法设计细节。
8. **设计文档与控制细节**：控制器、算法与其他研究设计细节统一放入 `agent_docs/`，不得写入 `AGENTS.md`。
9. **`6PUS_REAL_HARDWARE_OPERATION_GUIDE.md`**：面向操作人员的实机开机、回零、运动与关机速查表，保留在根目录原位。

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

## 3. 冻结基线与验证资产保护 (Frozen Baseline Protection)

1. **冻结资产只读原则**：J4 冻结资产、J5 冻结基线、PID 验证基线及后续所有冻结里程碑的资产，禁止原地修改；冻结不等于禁止使用，允许复制或基于其创建分支用于后续实验，但新开发必须在新版本、新分支或新的阶段命名空间中进行。
2. **禁止原地改写**：Agent 不得直接修改、覆盖、删除或重命名任何冻结文件、冻结配置、验证输出、日志、报告、结果数据或其他已验证证据。
3. **证据可恢复性**：已有验证证据必须保持可追溯、可恢复且不变；新实验或新验证不得覆盖已有的验证结果。
4. **版本化变更**：任何新实验、行为变化或基线演进，必须在新版本、新分支或新的阶段命名空间中进行；原冻结基线及其验证证据必须保持不变并可恢复。
5. **清单职责边界**：`AGENTS.md` 仅定义冻结保护规则，不维护冻结文件的详细清单；具体范围、版本标识与受保护文件以对应的 Frozen Manifest 或阶段权威文档为准。

---

## 4. 证据分级与结论真实性 (Evidence Classification)

1. **仿真结果 (Simulation Result)**：指 MATLAB/Simscape 及其他基于模型的运行结果，仅能支持模型层面的观察和结论。
2. **软件测试结果 (Software Test Result)**：指单元测试、集成测试、Mock/FakeBus 测试、调度测试及其他无实机验证结果，仅能支持软件行为、接口或调度结论。
3. **真实硬件验证 (Real Hardware Validation)**：指在实际物理平台上完成、具有记录证据且符合实机安全授权要求的实验，方可支持真实硬件行为结论。
4. **结论与证据一致性**：Agent 严禁将仿真结果或软件测试结果表述为真实硬件性能；所有结论必须与实际证据等级一致，并明确其验证边界。
5. **可信度与可追溯性**：本规则用于维护研究结论可信度与工程验证可追溯性，不限制控制算法研究、控制器选择或控制器参数设计。

---

## 5. 核心技术约定 (Technical Standards)

1. **单位制**：
   - 上层 ROS2 节点内部一律严格采用 **SI 国际标准单位制**（位置：米 $\text{m}$，速度：$\text{m/s}$，角度：弧度 $\text{rad}$，角速度：$\text{rad/s}$，电流：安培 $\text{A}$，力：牛顿 $\text{N}$）；
   - 底层电缸寄存器交互时显式转换（$1\text{ mm} = 40\text{ step}$，$1\text{ step} = 0.025\text{ mm}$）。
2. **坐标基准**：
   - 6PUS 机构运动学、逆解与笛卡尔控制一律以 **`OPERATING_HOME = [1000]*6` ($z_{\text{nom}} \approx 132.6123\text{ mm}$)** 为软件零位原点。
   - `STORAGE_PARK = [0]*6` 仅用于机械收拢与关机断电，严禁在此位置直接运行姿态控制算法。
3. **调度与解耦**：
   - ROS2 订阅回调一律采用 **Latest-Value-Wins (无锁/单槽原子覆盖)**，严禁在 Subscriber Callback 中执行阻塞式硬件通信，严禁累积 FIFO 队列。
   - 驱动节点内建 **Stale-Target Watchdog**，在上层目标流中断时自动挂起写流，严禁无限重发陈旧目标。
4. **控制架构不变量 (Control Architecture Invariants)**：
   - 坐标系定义与名称、原点定义、轴方向与 handedness、电机编号与支链编号、运动学几何约定、`OPERATING_HOME`、`STORAGE_PARK`、软件零位定义、SI 单位制以及仿真到硬件的坐标映射，均属于项目级工程不变量。
   - Agent 不得随意修改上述定义。
   - 任何有意修改必须创建新版本，并同步完成权威文档更新、跨仿真与 ROS2/硬件链路的影响评估，以及受影响接口的回归验证。
