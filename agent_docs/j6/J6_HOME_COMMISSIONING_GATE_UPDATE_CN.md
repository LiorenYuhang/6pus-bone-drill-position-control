# J6 HOME 首次实机调试门禁与执行评审更新

**文档版本：** v1.0
**适用对象：** 总装完成后的 6PUS 首次 `STORAGE_PARK -> OPERATING_HOME` 协调运动
**证据等级：** 工程策略 / 执行门禁，不是实机验证结果

本文件更新 J6 Phase3 / Phase4 首次 HOME 的门禁解释。它只更新调试策略和记录要求，不修改控制代码、底层驱动、`geometry.yaml`、`pus6_control` 或 J5 frozen 文件。本文件编制和审查期间不执行硬件、不打开真实串口、不发送电机命令。

## 1. 当前确认状态

| 项目 | 状态 | 说明 |
| --- | --- | --- |
| 机械总装 | 已完成 | 进入总装后首次闭链运动评审 |
| Phase1 READ_ONLY | PASS | 六轴通信和只读健康状态已通过 |
| Phase2 电机映射 | PASS | `ID1=A1`、`ID2=A2`、`ID3=A3`、`ID4=A4`、`ID5=A5`、`ID6=A6` |
| `q_park_reference` | 已确认 | `[0,0,0,0,0,0]`，作为安全起始参考 |
| `OPERATING_HOME` | 已定义 | `[1000,1000,1000,1000,1000,1000]` |
| `q_bottom` | **未标定** | 不宣称机械极限已测量，不作为首次 HOME 的必要校准项 |
| 首次 HOME 策略 | 允许有条件进入 | 只允许协调 PARK -> HOME，必须满足本文件门禁 |
| 本文件的硬件授权 | 未提供 | 本文件不授权本次实机运动 |

`q_park_reference` 是软件和装配记录确认的安全起始参考；它不等同于机械 bottom。现场仍须记录实际静态反馈及其偏差，不得自动改写 offset 或将偏差解释为机械极限测量。

## 2. Phase3 门禁更新

### 2.1 新的 Phase3 入口定义

对于“首次总装闭链运动”，Phase3 不再依赖 actuator-level 单轴 micro-jog PASS。新的入口门为 **Cartesian commissioning readiness**：

- 当前 CAD / 仿真几何、坐标系、ID 映射和 IK 接口版本已核对；几何标定仍是后续精化工作，不是首次 HOME 的前置条件。
- `q_park_reference`、实际六轴静态反馈和 `OPERATING_HOME` 目标已记录；任何起始偏差均保留原始值，不做位置补偿。
- HOME 轨迹预览已经完成：六轴目标向量、每轴增量、中间点、硬件限位和本次 commissioning 包络均通过检查。
- 使用协调六轴 IK / 轨迹接口；不得把五轴固定的单轴微动当作总装首动主路径。
- 相机可观察平台和主要支链，急停可立即操作，至少两名人员承担运动授权与机械观察。
- 六轴反馈、电流、故障、温度、反馈时间戳和写审计字段已准备；双 H30 / 相机证据按会话时间轴归档。
- 不自动重试、不自动扩大位移、不自动返回 HOME 或 PARK；每个阶段均需显式授权。

现有单轴 probe 保留为后续诊断工具。它不再是首次 HOME 的必经门，但任何单轴诊断仍需单独审批，不能以“其它五轴没有目标写入”替代闭链安全审查。

### 2.2 Phase3 判定

| 判定 | 含义 |
| --- | --- |
| `PASS` | 上述 Cartesian readiness 证据齐全，允许申请 HOME 授权 |
| `HOLD` | 轨迹预览、会话包络、观察设备、基线或人员门禁缺失 |
| `FAIL` | IK/限位检查失败、已有故障或配置不一致 |

`PASS` 只表示具备申请 HOME 的软件和现场条件，不等同于已完成真实运动验证。

## 3. HOME 执行门

### 3.1 允许的序列

```text
q_park_reference / 实际静态反馈
    -> 只读基线与目标预览
    -> 协调低速 HOME 轨迹
    -> HOME 静态确认与人工观察
```

本阶段只允许 `STORAGE_PARK -> OPERATING_HOME`。不得隐式执行 PARK、HOME 重试、Cartesian motion 或几何/offset 修改。到达 HOME 后必须先保持并确认，再决定是否进入后续 Cartesian micro-motion。

### 3.2 HOME 运动前硬门

- Phase0、Phase1、Phase2 证据可追溯，六轴 ID mapping 不变。
- `q_park_reference=[0]*6` 已确认；实际 `q_start_raw` 已现场读取并记录，且处于批准的 PARK 起始容差内。
- 六轴最新基线至少包含 target、actual/raw step、current、fault、temperature、硬件 limits、反馈时间戳，基线前写入计数为 0。
- 六轴 HOME 目标严格为 `[1000]*6`，目标及所有轨迹中间点都通过硬件保护限位和本次 commissioning 包络检查。
- 24 V 电源限流、E-stop、双人观察、近景相机和记录时钟已确认。
- 配置参数（控制模式、速度寄存器、安全上限）已由 `--validated-config` 对应的人工证据核验；该旗标本身不是自动配置验证。

### 3.3 首次 HOME 命令配置

供现场授权人审查的参数组合如下，**本文件未执行该命令**：

```text
--action home
--speed 50
--control-rate 10
--phase2-passed --phase3-passed
--e-stop-ready --two-person-observation
--real-hardware --confirm-motion --validated-config
--port <已核验的 by-id 串口>
--output <新的、不覆盖既有证据的 JSON 路径>
```

`--maintenance-authorized` 只适用于后续 PARK，不是 HOME 门。首次 HOME 应省略 `--duration`，使用已验证的低速轨迹计算；若必须指定 duration，必须在写入前计算并记录：

```text
planned_peak_speed = 1.875 * max_axis_displacement / duration
```

要求 `planned_peak_speed <= 50 step/s`。任何 duration 使峰值超过 50 step/s，或速度寄存器读回值不符合已批准的低速配置，均为 `HOLD`。不能只检查 CLI 的 `--speed=50`。配置写入的速度上限和轨迹规划速度必须分别记录并核对。

## 4. 速度政策

本次采用此前验证的 commissioning 低速配置：目标轨迹峰值 **50 step/s**、控制周期 **10 Hz**。该速度是“可观察且可及时急停”的实际调试速度，不以极限慢速为目标，也不允许未经审查的加速。

速度 PASS 必须同时具备：

1. CLI / API 请求速度严格为 50 step/s。
2. duration（若存在）满足上述峰值速度不等式。
3. 轨迹报告中的 `planned_peak_speed_step_s <= 50`。
4. 每轴速度配置寄存器写入及读回值、配置写时间戳和授权状态完整。
5. 运行时目标写入时间戳与目标增量能证明没有超出批准的 commissioning 速度。

当前旧版 HOME wrapper 会接受任意正 duration，且其委托 runner 可能设置高于 50 的寄存器上限。因此在代码行为未改变前，首次 HOME 不得传入未经审查的 duration；无法提供上述证据时保持 `HOLD`。

## 5. 写审计更新要求

报告必须将总写入拆分，而不是只给一个 `hardware_write_count`：

| 类别 | 内容 | 通过要求 |
| --- | --- | --- |
| `expected_configuration_writes` | 六轴控制模式、速度和安全上限配置（如使用） | 每轴地址、payload、时间戳、读回值、授权记录齐全 |
| `expected_motion_writes` | HOME 轨迹的六轴目标写入（0x29） | 目标向量、轴 ID、帧序号、时间戳与预览完全匹配 |
| `expected_stop_writes` | 仅在安全停止时的暂停写入（0x1A） | 触发原因、发送结果、停止确认齐全 |
| `unexpected_writes` | 不属于上述批准类别的任何写入 | 数量必须为 0，否则立即 `STOP/HOLD` |

`hardware_write_count` 应等于上述分类总和；Mock 写入和实机写入必须分开。配置写不计入运动写，但不能从总审计中隐藏。现有 wrapper 的 `hardware_writes_recorded >= 0` 不能作为有效 PASS 条件，只能作为报告字段存在性检查。

## 6. 首次 HOME 停止处理

以下任一事件必须触发锁存式立即停止：

- 任一 fault 非零；
- 任一轴 current 超过批准阈值（当前基线阈值为 500 mA）；
- RS485 超时、校验/协议错误、反馈读取失败或轴掉线；
- target-actual 跟踪误差超过会话批准阈值，或反馈不再单调跟随；
- 平台方向异常、倾斜异常、球铰/杆件/线缆干涉、异常声音或急停状态异常。

停止顺序必须是：

1. 立即停止继续发送目标，不等待 settle window。
2. 发送一次经批准的六轴暂停路径；不自动重试运动目标。
3. 读取并记录六轴停止后的 actual、target、current、fault 和反馈时间戳。
4. 核对六轴暂停结果；任一轴无法确认停止时，操作员保持急停并断开动力电源。
5. 报告标记 `STOP`，保留原始轨迹、写审计和异常原因；不得自动恢复、HOME、PARK 或重试。

现有委托 J3 runner 在检测 fault / 过流后先等待默认 `settle_time=2 s` 再调用 `pause_all()`，且反馈读取异常不保证进入暂停路径。因此，在该行为未被重新验证前，不能把旧 wrapper 单独视为满足“首次 HOME 立即停止”门。

## 7. 反馈与现场观察记录

一次有效 HOME 会话至少记录：

- `q_park_reference`、实际 `q_start_raw`、HOME 目标和每个六轴轨迹目标/实际值；
- ID1-6 current、fault、temperature、硬件限位和逐轴时间戳；
- 规划峰值速度、实际配置速度、控制频率、duration 和停止确认时间；
- 平台是否按预期上升、球铰状态、杆件/线缆干涉、异常声音；
- 近景相机快照时间戳及可见/遮挡支链；
- 双 H30 基座/平台姿态及相对姿态时间配对（如已安装并纳入会话）。

HOME 静态确认应满足：六轴残差在预先批准容差内、fault=0、current 回落至静态范围、平台无可见干涉。该 PASS 不表示 `q_bottom` 已标定，也不表示几何参数已经冻结。

## 8. 本次门禁结论

| 层级 | 结论 |
| --- | --- |
| Phase3 政策定义 | `PASS`：以 Cartesian commissioning readiness 取代单轴 micro-jog 依赖 |
| PARK -> HOME 路径 | `允许申请`：须满足本文件的相机、急停、监控和授权门 |
| `q_bottom` | `未标定`：记录风险，不阻断首次 HOME |
| 速度门 | `HOLD 直至证据齐全`：必须证明实际峰值不超过 50 step/s |
| 写审计门 | `HOLD 直至分类字段齐全` |
| 立即停止门 | `HOLD 直至停止确认路径被验证` |
| 本次实机执行授权 | **未授权 / 未执行** |

## 9. 现场签署

| 角色 | 姓名 / 签名 | 日期 | 结论 |
| --- | --- | --- | --- |
| 机械观察员 |  |  |  |
| 电气 / 急停观察员 |  |  |  |
| IK / 轨迹复核人 |  |  |  |
| 写审计复核人 |  |  |  |
| 实机运动授权人 |  |  | 未授权 |
