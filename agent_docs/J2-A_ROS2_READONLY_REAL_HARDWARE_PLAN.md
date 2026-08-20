# Stage J2-A — ROS2 正式 Driver 六机真实 UART 纯只读实机验证方案

> **后续状态修正**：本文保留 J2-A 的原始执行方案与历史审计记录。J2-A 后续实机只读验证已记录为 **COMPLETE / PASS**，全生命周期 `hardware_write_count = 0`。当前串口默认使用 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`；文中出现的 `/dev/ttyUSB0` 仅是早期方案或 by-id 链接解析出的内核设备名，不是操作手册的默认入口。正文早期的 `INCOMPLETE`、STOP 或重试状态均为中间快照，最终状态以本文末尾的 G 节为准。

> 项目根目录：`/home/lyapunov/bone_drill`  
> ROS2 正式工作空间：`/home/lyapunov/bone_drill/bone_ws`  
> 正式电机驱动包：`pus6_motor_driver`  
> 当前阶段：`Stage J2-A`  
> 当前状态：`COMPLETE / PASS`（原始计划时点的 `PREPARED / NOT EXECUTED` 仅作为历史记录保留）
>
> 本文件是 Stage J2-A 的权威执行规范。
>
> **J2-A 只允许真实 UART 读取，不允许任何硬件写入，不允许任何电机运动。**

---

# 1. J2-A 的阶段目标

J2-A 是新的正式 ROS2 六电机驱动第一次连接真实硬件。

目标不是测试电机运动，而是验证以下完整链路：

```text
/home/lyapunov/bone_drill/bone_ws
        ↓
pus6_motor_driver
        ↓
hardware_mode = read_only
        ↓
SixMotorController
        ↓
UartBus
        ↓
CH340 / ch341-uart
        ↓
921600 8N1
        ↓
ID1 ~ ID6
```

J2-A 必须证明：

1. 正式 ROS2 driver 可以稳定连接真实六机 UART；
2. ID1~ID6 均能稳定读取；
3. ROS2 标准状态和 raw diagnostic 数据发布正确；
4. 六机顺序采样时间戳完整；
5. 获得真实六机 acquisition cycle 性能数据；
6. 整个节点生命周期严格：

```text
hardware write_count = 0
```

包括：

```text
节点启动
→ 60s运行
→ Ctrl+C / shutdown
→ 串口关闭
```

全过程都必须为 0 硬件写入。

---

# 2. 权威依据与优先级

执行 J2-A 前必须阅读并遵守：

```text
/home/lyapunov/bone_drill/BONE_WS_ROS2_ARCHITECTURE_PLAN.md
```

以及 UART bring-up 权威规则：

```text
/home/lyapunov/bone_drill/up/motor/uart_multi/AGENTS.md
```

必要时参考：

```text
/home/lyapunov/bone_drill/up/motor/uart_multi/UART实机运行记录.md
/home/lyapunov/bone_drill/up/motor/uart_multi/UART六电机编号与测试方案.md
```

旧 ROS2 UART 工程：

```text
/home/lyapunov/inspire_motor_uart
```

仅作为历史参考。

其审计报告：

```text
/home/lyapunov/bone_drill/agent_docs/ROS2_UART_EXISTING_CODE_AUDIT.md
```

不得覆盖当前 `uart_multi` 已通过 M2 / M4 / M6 实机验证得到的最新协议与安全结论。

---

# 3. 当前 UART 权威基础参数

正式 UART 参数：

```text
Port:
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0

Resolved:
/dev/ttyUSB0

USB-UART:
QinHeng CH340

Linux driver:
ch341-uart

Baudrate:
921600

Frame:
8N1

Read timeout:
0.05 s
```

当前设备不存在：

```text
latency_timer
```

因此 J2-A：

- 不修改 Linux USB-UART latency；
- 不向 sysfs 写任何值；
- 不执行任何 `latency_timer=1ms` 优化；
- 首先实测当前标准 Linux 驱动下的真实性能。

---

# 4. 当前六机历史参考位置

M6-C Retry COMPLETE / PASS 后历史停机位置：

```text
ID1 ≈ 67 step
ID2 ≈ 75 step
ID3 ≈ 31 step
ID4 ≈ 63 step
ID5 ≈ 23 step
ID6 ≈ 17 step
```

以上只能作为：

> 静态位置参考中心

不得作为必须逐样本严格相等的门禁。

此前实机已经证明不同电机可能存在有限的 1~2 step 静态位置读回抖动。

因此 J2-A 应关注：

- 合法位置范围；
- histogram；
- min / max；
- span；
- 是否存在持续单向漂移；
- current；
- fault；
- UART 通信稳定性。

不得因为单个已知量化/反馈抖动样本直接判定硬件故障。

---

# 5. J2-A 强制 Hardware Mode

J2-A 必须显式使用：

```text
hardware_mode = read_only
```

禁止使用：

```text
hardware_mode = active
```

也不得依赖默认值猜测运行模式。

Agent 在启动正式 ROS2 node 前必须先核对：

```text
hardware_mode == read_only
```

---

# 6. READ_ONLY 模式的绝对权限边界

`read_only` 模式允许：

```text
打开真实 UART
读取寄存器
读取状态
发布 ROS2 Topic
统计时间
统计通信性能
关闭 UART
```

禁止：

```text
任何硬件 write_register
任何 0x25 写
任何 0x29 写
任何 0x1A 写
configure_all()
command_positions()
command_positions_with_metrics()
command_single_axis_for_bringup()
pause_all()
任何其他寄存器写入
```

特别注意：

> READ_ONLY 模式退出节点时也禁止发送 pause。

因此：

```text
Ctrl+C
destroy_node()
shutdown
close()
```

全过程都不得产生硬件写操作。

---

# 7. J2-A 0-Write 红线

J2-A 最重要的验收条件：

```text
hardware write_count = 0
```

必须覆盖整个生命周期：

```text
serial open
↓
ROS2 node start
↓
60s read-only operation
↓
ROS2 shutdown
↓
serial close
```

不是只要求“运动阶段 0 write”。

如果当前正式 driver 无法可靠审计实际 write_count：

> 不得猜测为 0。

必须停止测试并报告：

```text
J2-A INCOMPLETE
```

直到能够证明硬件写入计数。

---

# 8. J2-A 禁止自动配置

节点启动后不得自动执行：

```text
control mode configuration
speed configuration
target synchronization
position target
pause
fault clear
flash save
任何其他配置写
```

特别禁止：

```text
0x25
0x29
0x1A
```

节点启动阶段只允许：

```text
connect
read
publish
```

---

# 9. 实机运行持续时间

在通信与状态正常的前提下：

```text
目标连续采样时间 = 60 s
```

如果出现明确 STOP 条件，可以提前结束。

不得为了获得完整 60s 数据而忽略：

- fault；
- 非预期运动；
- 明显持续位置漂移；
- 明显异常电流；
- 严重 UART 通信错误。

---

# 10. 六机读取内容

至少读取 ID1~ID6 的：

```text
actual position
current
force raw
temperature
fault
```

如果当前正式状态接口同时读取合法的 target / register target，可以一并记录。

但是不得为了增加统计字段而新增任何写操作。

---

# 11. ROS2 Topic 验证

至少核对：

```text
/pus6/joint_states

/pus6/joint_steps_raw
/pus6/joint_currents_raw
/pus6/joint_forces_raw
/pus6/joint_temperatures_raw
/pus6/joint_faults_raw
```

---

# 12. `/pus6/joint_states` 单位语义

类型：

```text
sensor_msgs/msg/JointState
```

必须满足：

```text
name:
motor_1 ... motor_6

position:
SI unit = m

velocity:
[]

effort:
[]
```

不得将：

```text
current_mA
```

写入：

```text
JointState.effort
```

也不得将未标定：

```text
force_g_raw
```

直接转换并填入 N。

---

# 13. Step 与 SI 单位换算

当前正式换算：

```text
40 step/mm
```

因此：

```text
1 step
= 0.025 mm
= 25 µm
= 0.000025 m
```

即：

```text
40000 step/m
```

J2-A 必须抽样核对：

```text
joint_steps_raw
```

与：

```text
joint_states.position
```

之间换算一致。

例如：

```text
67 step
→ 0.001675 m

75 step
→ 0.001875 m
```

允许位置读回本身存在静态 step 抖动，但换算关系必须严格正确。

---

# 14. Force 数据语义

当前 UART force 保留：

```text
force_g_raw
```

当前没有完成统一 Force Zero Calibration。

因此不得认为：

```text
所有电机具有统一 -400 g 偏置
```

正确结论是：

> 不同电机可能存在各自非零的静态 force 原始基线，并可能受到个体、安装和姿态影响。

J2-A：

- 只统计 raw force；
- 不建立绝对力控制阈值；
- 不转换为正式 N；
- 不修改任何 force 参数。

---

# 15. 六轴采样的正确时间语义

六台电机共享一个 UART。

因此：

```text
ID1
↓
ID2
↓
ID3
↓
ID4
↓
ID5
↓
ID6
```

属于：

> Ordered Sequential Acquisition Cycle

不是：

> Atomic Snapshot

不得使用“六轴同时采样”“原子采样”等不准确术语。

---

# 16. SixAxisStateSnapshot 时间戳审计

正式 driver 已提供：

```text
timestamp_start
timestamp_end
per_axis_timestamps
positions_steps
positions_meters
```

J2-A 必须验证这些字段在真实硬件下有效。

对每轮六轴读取计算：

```text
T_cycle = timestamp_end - timestamp_start
```

同时记录：

```text
t_ID1
t_ID2
t_ID3
t_ID4
t_ID5
t_ID6
```

并分析：

```text
t_ID6 - t_ID1
```

代表一次六轴顺序采样的实际轴间时间跨度。

---

# 17. J2-A 时序性能统计

整个 60s 测试至少统计：

```text
total acquisition cycles
successful acquisition cycles
failed acquisition cycles
```

对：

```text
T_cycle
```

统计：

```text
min
mean
median / P50
P95
P99
max
standard deviation（建议）
```

并计算：

```text
effective acquisition frequency
≈ 1 / mean(T_cycle)
```

注意：

> 不允许预设一定是 50 Hz。

必须以真实 J2-A 数据为准。

---

# 18. ROS2 Topic 实际发布频率

区分：

```text
requested timer frequency
```

和：

```text
actual effective publish frequency
```

不得因为 ROS2 timer 设置为：

```text
50 Hz
```

就直接报告系统实际反馈频率为：

```text
50 Hz
```

必须测量实际 topic 发布频率。

建议报告：

```text
mean publish frequency
P95 interval
P99 interval
max interval
```

如果实际发布频率受 UART acquisition cycle 限制，应明确记录。

---

# 19. UART 通信统计

J2-A 必须统计至少：

```text
total UART read transactions
successful reads
timeouts
checksum errors
wrong_id
length errors
protocol errors
other UART exceptions
```

如果底层能够区分：

```text
first_success
retry_success
final_failure
```

也一并记录。

但注意：

> READ_ONLY 允许读事务按既定安全规则进行 read retry；不得因此产生任何写事务。

---

# 20. ID1~ID6 静态位置统计

对每台电机分别统计：

```text
position histogram
mode/reference（如果适用）
min
max
span
mean（可选）
```

重点检查：

```text
是否合法
是否有持续单向漂移
是否存在明显非预期运动
```

不得只根据：

```text
历史最终位置
```

要求每次严格相等。

---

# 21. Current 统计

对 ID1~ID6 分别统计：

```text
current min
current max
current mean
```

静态时原则上应保持合理低值。

如果出现明显持续非零或异常大电流，应立即调查。

J2-A 不允许通过软件写入改变控制状态。

---

# 22. Force Raw 统计

对每台电机记录：

```text
force_g_raw min
force_g_raw max
force_g_raw mean
```

仅作为：

```text
静态原始传感器基线数据
```

不得进行力闭环判断。

---

# 23. Temperature 统计

分别记录：

```text
temperature min
temperature max
```

如温度明显异常，应终止测试。

---

# 24. Fault 统计

对每台电机记录：

```text
fault set
```

正常预期：

```text
{0x00}
```

如果任一电机出现：

```text
fault != 0
```

原则上：

```text
STOP J2-A
```

并保存当时完整上下文。

---

# 25. 持续位置漂移判定

需要区分：

```text
有限静态量化/反馈抖动
```

与：

```text
持续单向位置变化
```

例如：

```text
67 ↔ 66 ↔ 67
```

可能属于静态反馈抖动。

而：

```text
67 → 66 → 65 → 64 → ...
```

属于需要立即调查的持续漂移。

不得为了通过测试而自动扩大容差。

---

# 26. J2-A STOP 条件

出现以下任一情况，立即结束软件只读测试：

1. 任一电机 fault 非 0；
2. 检测到明显非预期持续运动；
3. 检测到持续单向位置漂移；
4. 明显异常电流；
5. 严重 UART 通信异常；
6. ROS2 driver 出现异常导致状态不可可信；
7. 发现任意硬件 write；
8. hardware_mode 不是 `read_only`；
9. shutdown 产生硬件 write；
10. 数据记录不足以证明生命周期 0 write。

---

# 27. J2-A 发生危险时的处理原则

由于：

```text
hardware_mode = read_only
```

软件层不得为了停止电机而发送：

```text
pause
```

否则 J2-A 的 0-write 条件被破坏。

如果真实机构出现物理危险：

> 操作员立即使用硬件急停 / 断电。

此时记录：

```text
J2-A STOP / HARDWARE E-STOP
```

不得自动切换到 active 模式。

---

# 28. STOP / FAIL 后禁止自动重试

如果 J2-A 出现：

```text
STOP
FAIL
INCOMPLETE
```

Agent 必须：

```text
停止
↓
保存日志
↓
分析原因
↓
向用户报告
```

禁止：

```text
自动修改代码
↓
自动重新运行J2-A
```

也禁止为了通过而：

```text
放宽安全门禁
降低审计要求
切换active模式
```

任何 Retry 必须重新获得用户明确授权。

---

# 29. J2-A PASS 条件

只有同时满足以下全部条件，才允许：

```text
Stage J2-A = COMPLETE / PASS
```

必须满足：

1. ID1~ID6 全部稳定响应；
2. 完成计划的真实只读采样；
3. UART 无严重通信异常；
4. 六机位置处于合法静态状态；
5. 无持续单向漂移；
6. 无非预期运动；
7. fault 无异常；
8. current 无明显异常；
9. `/pus6/joint_states` 单位为 m；
10. JointState velocity 为空；
11. JointState effort 为空；
12. raw topics 正确；
13. step ↔ meter 换算正确；
14. SixAxisStateSnapshot 时间戳完整；
15. 获得真实 T_cycle 统计；
16. 获得真实 ROS2 publish frequency；
17. 获得 UART error statistics；
18. 节点启动到退出全过程：

```text
hardware write_count = 0
```

---

# 30. J2-A 原始数据与日志要求

J2-A 必须保存原始运行证据。

建议在：

```text
/home/lyapunov/bone_drill/bone_ws/log/
```

或当前正式实验日志目录中建立唯一时间戳日志。

日志不得覆盖。

至少包含：

```text
运行时间
运行命令
hardware_mode
port
baudrate
ROS distro
git状态（若适用）
六机raw state
SixAxisStateSnapshot timestamps
UART统计
ROS2 publish统计
write audit
最终PASS/STOP结果
```

若已有统一实验日志结构，应优先沿用现有结构，不重复制造平行日志体系。

---

# 31. J2-A 完成后文档同步

PASS 后更新：

```text
/home/lyapunov/bone_drill/BONE_WS_ROS2_ARCHITECTURE_PLAN.md
```

并建立/更新正式 J2 实机运行记录。

必须记录：

```text
J2-A COMPLETE / PASS
```

及权威日志路径。

不得删除历史失败记录。

---

# 32. J2-A 与 J2-B 的严格阶段隔离

即使 J2-A PASS：

> 不得自动执行 J2-B。

J2-A 结束后只能：

```text
准备 J2-B
```

J2-B 必须单独审批。

---

# 33. J2-B 当前预期方向

J2-B 的目标是：

> 正式 ROS2 driver 第一次真实硬件写入与单电机微运动验证。

J2-B 后续必须重新建立完整安全链：

```text
六机只读检查
↓
指定 moving motor
↓
其余5台严格观察
↓
核对 moving motor 当前配置
↓
必要时仅配置 moving motor
↓
单机 target
↓
运动监测
↓
pause
↓
stop confirmation
```

J2-A 不执行上述任何步骤。

---

# 34. 当前禁止的后续阶段

本轮禁止：

```text
J2-B
J2-C
J3
IK实机
PID闭环
ADRC
SAC
```

---

# 35. Agent 最终报告格式

J2-A 执行结束后，最终至少报告：

## A. 基本信息

```text
运行时间
运行持续时间
hardware_mode
UART port
baudrate
ROS2 Jazzy环境
```

## B. 六机状态

ID1~ID6：

```text
position histogram
min/max/span
current min/max
force raw min/max/mean
temperature min/max
fault set
```

## C. UART

```text
transactions
success
timeout
checksum
wrong_id
other errors
```

## D. Timing

```text
total cycles

T_cycle:
min
mean
P50
P95
P99
max

effective acquisition frequency

ID1→ID6 sampling span
```

## E. ROS2

```text
joint_states frequency
raw topics frequency
SI单位核对
step/m换算核对
```

## F. Write Audit

必须明确：

```text
startup writes = ?
runtime writes = ?
shutdown writes = ?
total hardware writes = ?
```

J2-A PASS 的正确结果必须为：

```text
startup writes = 0
runtime writes = 0
shutdown writes = 0

TOTAL HARDWARE WRITES = 0
```

## G. 最终结论

只能使用：

```text
J2-A COMPLETE / PASS
```

或：

```text
J2-A STOP / NOT PASSED
```

或：

```text
J2-A INCOMPLETE
```

不得模糊判定。

---

# 36. 当前阶段核心原则

J2-A 的意义不是证明：

> 电机可以运动。

这已经由 UART bring-up 的 M6-C 完成。

J2-A 真正要证明的是：

> 新的正式 ROS2 `pus6_motor_driver` 可以安全、稳定、可测量地读取真实六电机，并且在 READ_ONLY 生命周期内做到绝对 0 硬件写入。

同时建立后续真实闭环控制所需的：

```text
UART通信性能基线
六轴采样周期基线
ROS2发布频率基线
六轴时间偏差基线
静态位置反馈基线
```

这些数据将用于决定后续：

```text
J2-B
J2-C
J3 IK
PID
ADRC
SAC
```

的真实控制频率与软件架构。

---

# 37. 当前状态

```text
Stage J2-A
PREPARED / NOT EXECUTED
```

等待用户明确人工授权后方可执行真实六机 UART 只读测试。
