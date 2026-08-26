# J7-C3 六电机同目标 Servo 硬件验证方案

## 1. 审查范围与安全边界

本文件是六电机同目标 Servo Mode（`mode=1`）首次真实硬件验证的执行设计，仅用于实验前评审和现场门禁准备。

本阶段不执行以下动作：

- 不连接、打开或操作真实串口；
- 不发送运动命令，不执行动作测试；
- 不修改 `protocol.py`、`uart_bus.py`、`motor.py`；
- 不修改 ROS2 节点、现有控制链路、配置文件或 J6 runner；
- 不修改或覆盖 J6 frozen baseline。

所有现场操作必须在获得明确实机授权、具备急停和双人观察条件后进行。任何 Mock、代码审查或调度结果都不能表述为硬件性能验证。

## 2. 实验目标与不变量

目标是验证六个电机能否在同一独立 Servo 实验路径中进入 `mode=1`，并以 write-only `0x29` 连续接收同目标流：

```text
PARK  [0, 0, 0, 0, 0, 0]
  ->
HOME  [1000, 1000, 1000, 1000, 1000, 1000]
```

第一阶段的同目标运动用于降低姿态变化和支链差异，但不能将其视为无风险：机构仍可能存在装配误差、摩擦、卡滞、线缆干涉和单轴失步。J6 frozen baseline 保持不变，且本实验不调用 J6 的 mode=0 runner。

固定不变量：

- `mode=0`、`HOME=[1000]*6`、`PARK=[0]*6` 的 J6 baseline 只读保存；
- Servo 实验使用隔离的 J7 transport/path；
- 运动帧采用 write-only、零重试语义；
- 上层目标使用 latest-value-wins，目标停止后由 watchdog 挂起写流并执行一次受控停止；
- 六轴 UART 由单一 writer 串行发送，禁止多个线程直接写串口。

## 3. 六轴 Servo 启动流程

启动流程必须先完成只读检查，再逐轴配置和 readback。任何一轴失败都不得进入运动阶段。

### 3.1 上电与现场条件

进入软件启动前，现场负责人确认：

1. 六轴机构处于可观察、无工具和人员进入运动包络的位置；
2. 急停可用、限位/机械防护已确认，电源和电流限制符合批准的实验配置；
3. 两名观察者已就位，通信、采集和停止责任明确；
4. 当前没有运行 ROS2 J6 节点、旧 runner 或其他串口占用者；
5. 运动前允许从 PARK 或经批准的安全静止位置开始，不能假设上电位置就是 PARK。

### 3.2 只读静态检查

对 ID1-ID6 逐轴读取并记录：

- 电机 ID 与物理支链映射；
- 当前 mode；
- fault/alarm；
- 位置；
- 电流（若寄存器提供）；
- 通信响应、帧校验和响应延迟。

门禁：六轴 ID 唯一且映射正确，fault 清零或有批准的可解释状态，反馈可读，位置处于机械允许范围，且没有未处理的 RX 残留或协议错误。

### 3.3 mode=1 与 speed 配置

在单一 UART writer 内按轴顺序执行配置。配置帧允许 response 和 readback：

1. 写 `0x25 = 1`（Servo Mode）；
2. 写批准的 `0x28` speed 值；
3. 读取 `0x25`、`0x28`，核对每轴 readback；
4. 记录配置前后 mode、speed、fault 和位置。

配置阶段的 response 必须与当前请求匹配。残留、格式错误或超时必须使该轴配置失败，不得靠重试运动帧掩盖问题。六轴全部 readback 通过后才允许 arm；任一轴仍为 `mode=0`、`mode=2` 或 readback 不一致时，立即退出配置流程并保持静止。

### 3.4 Arm 前最终检查

arm 前再次确认：

- 六轴均为 `mode=1`；
- speed 为批准值且无隐式默认值覆盖；
- 无 fault、通信超时、RX overflow/residual 未处理项；
- watchdog 已 armed，初始目标和目标时间戳有效；
- write-only 运动接口、反馈接口和停止路径已选择且日志文件可写；
- 已定义当前 phase 的停止阈值和人工急停动作。

## 4. 分阶段安全运动流程

### Phase 0：全部静止确认

**进入条件**：启动只读检查和六轴 mode/speed readback 全部通过，且机械状态允许静止采集。

**动作**：不发送运动目标。持续采集六轴位置、fault、current、mode、通信状态和 RX 计数，确认至少一个完整观察窗口内无自发位置变化、无 fault 上升、无通信超时。

**通过条件**：六轴位置在批准的静止容差内，反馈年龄、RX residual 和 watchdog 均正常；人工确认可进入小范围测试。任一条件不满足则保持静止并退出。

### Phase 1：小范围同目标测试

建议首个目标段为同目标 `500 -> 550` steps（或经现场负责人批准的更小范围），不直接跳到 PARK/HOME。起点必须通过只读反馈确认，不能仅按软件目标假定当前位置为 500。

**进入条件**：Phase 0 PASS；六轴均在 `mode=1`；起点位置、方向、机械间隙和运动包络已确认；实时停止监控正常。

**执行约束**：

- 使用独立 Servo target stream，固定 50 Hz 目标刷新；
- 六轴同一 cycle 使用同一个目标值，单 UART writer 依序发送 ID1-ID6；
- 0x29 运动帧 write-only，不等待运动 response，不自动 retry；
- 目标更新停止或 watchdog 触发时，停止发送陈旧目标并执行一次批准的暂停/停机序列。

**通过条件**：六轴均向正确方向运动，无 fault、卡滞、明显位置分叉或通信 deadline miss，且停止后位置稳定。记录的同步误差、电流和 feedback age 必须在实验批准阈值内；不得仅凭目视“平滑”判定通过。

### Phase 2：PARK -> HOME

**进入条件**：Phase 1 PASS，所有轴的 mode、方向、位置反馈、停止逻辑和采集证据均复核通过；现场重新确认无干涉、无负载异常和急停可达。

**动作**：

1. 先确认六轴实际位置已处于 PARK `[0]*6` 或经批准的等效安全起点；
2. 在 50 Hz write-only stream 下发送同目标 PARK 保持窗口，确认系统稳定；
3. 发送同目标 HOME `[1000]*6`，持续刷新直至六轴均达到批准的位置窗口；
4. 到达后继续短暂保持并采集，再按批准流程停止写流。

PARK 的 raw=0 是收拢/停放语义，HOME 的 raw=1000 是工作基准语义。若任一轴未到达、反馈缺失或监控条件触发，立即停止，不继续扩大行程。

## 5. 实时安全监控与立即停止条件

监控线程只读反馈和 transport 统计，不与唯一 UART writer 竞争写权限。每个 Servo cycle 关联 cycle id、目标时间戳和六轴反馈快照。

实时监控至少包括：

- 六轴 `target_position`、`feedback_position` 与位置误差；
- 六轴间位置差、最大/最小轴偏差和 ID1-ID6 TX skew；
- fault/alarm、current（及可用的温度/限位状态）；
- command deadline、cycle jitter、target age；
- communication timeout、反馈年龄和连续丢反馈计数；
- RX residual、stale response、malformed frame、overflow、response mismatch；
- watchdog 状态、停止原因和停止完成状态。

出现下列任一情况，立即停止发送新的运动目标，并执行一次受控暂停/停机；若暂停确认不可得，使用已批准的电源/急停流程：

- 任一轴 fault、过流、限位或位置反馈异常；
- 任一轴位置误差或六轴同步差超过该 phase 的批准阈值；
- 目标流超时、target age 超限或 watchdog 触发；
- UART 写失败、连续 deadline miss、通信超时或反馈年龄超限；
- RX buffer overflow、无法归属的残留帧、格式错误或 response mismatch；
- 任一轴 mode readback 不再是 `mode=1`；
- 观察者发现异常噪声、振动、卡滞、结构干涉或急停请求。

停止动作不得 retry 运动帧，不得继续发送陈旧目标。停止后记录原因、最后一个完整 cycle、各轴位置和 fault，再由现场负责人决定是否断电或退出。

## 6. 反馈与 UART 调度建议

Servo TX 与反馈读取共用一条 UART，必须由单一调度器仲裁。不能在每个 20 ms cycle 内强制执行完整六轴 request-response scan，因为既有六轴 request-response 历史均值约 `18.39 ms`、P95 约 `20.05 ms`、最大约 `23.89 ms`，会挤占 50 Hz deadline。

首次六轴验证建议采用以下可观测策略，并在现场根据批准阈值选择：

- Servo：50 Hz 六轴 write-only target stream；
- 位置反馈：每个 Servo 周期最多轮询一个轴，ID1-ID6 round-robin，记录每轴 feedback age；
- 安全反馈：每 100 ms 做一次 full six-axis scan，或在 UART 空闲窗口 defer；
- 配置和反馈 response 必须在独立 transaction 中清理/归属 RX，不能将旧 0x29 残留误配为 0x32 等反馈响应。

若 full scan、配置或异常处理占用窗口，应优先保持运动 deadline 和 watchdog 语义，明确记录 feedback defer，而不是阻塞 writer 等待响应。

## 7. 数据采集规范

每次实验生成不可覆盖的新记录，至少包含：

### 7.1 指令与发送

- UTC/单调时钟的 `command_timestamp`；
- `cycle_id`、目标值、target age；
- 每轴 ID、TX start、TX end、帧长度和写入结果；
- batch complete time、ID1-ID6 TX skew；
- cycle period、cycle jitter、deadline miss、defer/overwrite count。

### 7.2 反馈与状态

- 每轴 feedback timestamp、position、feedback age；
- position error、六轴最大同步差；
- current、fault/alarm、mode、限位/温度（若可读）；
- UART timeout、写失败、RX residual 数量、stale/malformed/overflow/mismatch 计数；
- watchdog armed/triggered、停止时间、停止原因和停止结果。

### 7.3 分段标记

日志必须标明 `Phase 0/1/2`、目标段、起止时间、操作者、授权编号和硬件配置摘要。报告中严格区分“软件调度证据”和“真实硬件证据”。

## 8. 与 J6 mode=0 的对比指标

J7 Servo 结果只与保持不变的 J6 frozen baseline 做同条件、同量程、同采样方式的对比：

| 指标 | J6 mode=0 Positioning | J7 mode=1 Servo | 判定重点 |
|---|---|---|---|
| 运动连续性 | 位置步进/到位式行为及其日志 | 50 Hz target stream 下的位置变化 | 是否减少可见步进和周期性停顿 |
| 位置跟踪 | 目标到位误差、到达时间 | command-to-position error、feedback age | 跟踪误差是否可解释且无持续发散 |
| 六轴同步 | 六轴命令批次和到位差 | TX skew、位置差、到达时间差 | 同目标时各轴是否保持批准同步窗口 |
| 电流变化 | J6 运动期间 current 峰值/波动 | Servo 运动期间 current 曲线 | 是否出现异常峰值、周期振荡或过流 |
| 卡顿/振动 | 记录 jerk、停顿、异常声音/振动 | 同样的仪器和观察条件记录 | 目视观察只能作为辅助证据 |
| 通信行为 | response 等待和 command 周期 | write-only、RX residual、feedback defer | 是否有 deadline miss、残留误配或 watchdog 触发 |

结论必须基于时间戳、位置反馈、电流和 fault 数据。仅凭“看起来更平滑”不能证明 Servo 在机械系统上优于 J6。

## 9. 分阶段 PASS/FAIL 与收尾

- **Phase 0 PASS**：六轴静止、状态健康、反馈和 RX 清洁；否则不 arm。
- **Phase 1 PASS**：小范围同目标运动和停止均满足位置、同步、通信及安全阈值；否则不进入 Phase 2。
- **Phase 2 PASS**：PARK->HOME 全程无立即停止条件，六轴达到位置窗口并有完整证据；否则标记 FAIL，不重复发送运动帧。

任一阶段 FAIL 后，保持停止状态，保存日志和故障快照，由负责人进行只读复盘。不得通过修改 J6 baseline、放宽阈值或跳过 Phase 1 来“修复”结果。

实验结束后：停止 Servo 写流，确认 watchdog/暂停状态，读取并记录六轴最终位置、mode、fault、current 和 RX 统计；按批准的断电/机械停放流程收尾。所有记录归档到新的实验目录，不能覆盖既有 J6 证据。

## 10. J7-C3 结果边界

本方案只定义首次六轴同目标 Servo 验证的安全流程和证据要求。即使 Phase 2 PASS，也只能支持“在本次硬件、负载、阈值和日志条件下的六轴 Servo 行为验证”，不能直接推出振动抑制能力、任意姿态运动能力或长期可靠性。后续实验必须另行评审，不得改变 J6 frozen baseline。
