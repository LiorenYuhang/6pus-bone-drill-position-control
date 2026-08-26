# UART 电机控制实现对比审计

## 审计范围

- 项目 A：`/home/lyapunov/bone_drill`
- 项目 B：`/home/lyapunov/inspire_motor_uart`
- 本报告仅基于代码阅读；未运行真实硬件，未修改协议、参数、ROS2 或硬件配置。

## 结论摘要

项目 A 的主机轨迹生成实际上比项目 B 更复杂，包含 50 Hz 控制循环和带速度、加速度、加加速度限制的状态整形器。项目 A 出现卡顿的主要软件层风险不在轨迹公式，而在执行路径：

1. 项目 A 设计上使用速度模式 `mode=2`，项目 B 的平滑路径使用伺服模式 `mode=1`。
2. 项目 A 每个周期向 6 个电机顺序发送目标，并等待 6 个响应；项目 B 伺服发送目标时不等待响应。
3. 项目 A 的上游控制频率可能为 50 Hz，而 J4-F1B 运行器的 UART worker 为 25 Hz，Latest-Value-Wins 会丢弃中间目标点。

因此，两者并不是相同的 UART 工作负载。项目 B 的平滑表现很可能主要来自“伺服模式 + 高频无响应写入 + 单电机”的组合。

## 1. 指令生成链路

### 项目 A

```text
笛卡尔轨迹/ROS2目标
 -> 逆运动学
 -> Pus6ControlNode（50 Hz）
 -> StatefulJerkLimitedShaper
 -> /pus6/command_joint_steps
 -> Pus6MotorDriverNode（50 Hz，J4-F1B 为 25 Hz）
 -> SixMotorController
 -> ID1..ID6 顺序写入寄存器 0x29
 -> UartBus
 -> 六个电机
```

依据：

- 控制循环和发布：[control_node.py:39](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py:39)、[control_node.py:154](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/control_node.py:154)
- 状态整形器：[joint_trajectory_controller.py:298](/home/lyapunov/bone_drill/bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py:298)
- 最新值覆盖：[motor_driver_node.py:350](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:350)
- 六轴顺序写入：[six_motor_controller.py:497](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:497)

项目 A 是“主机插值 + 绝对位置点”架构。整形器输出最终会被转换、四舍五入为整数步数。

### 项目 B

```text
终端输入目标位置
 -> servo_position 共享变量
 -> 伺服线程（约 50 Hz）
 -> send_servo_position()
 -> 写寄存器 0x29，不读取响应
 -> 电机 SERVO 模式内部插补
```

依据：

- SERVO 模式定义及“至少 50 Hz”： [protocol.py:46](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/protocol.py:46)
- 伺服目标不等待响应：[motor_controller.py:377](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/motor_controller.py:377)、[motor_controller.py:388](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/motor_controller.py:398)
- 伺服线程：[motor_node.py:237](/home/lyapunov/inspire_motor_uart/ros2_jazzy/src/insmotor_driver/insmotor_driver/motor_node.py:237)

项目 B 的主机没有轨迹插值器；目标值可以直接跳变，平滑性由电机伺服模式内部处理。

## 2. UART 与调度对比

| 项目 | `bone_drill` | `inspire_motor_uart` | 对平滑性的影响 |
|---|---|---|---|
| 波特率 | 921600，8N1 | 921600，8N1 | 基本相同 |
| 电机数量 | 六电机共享一条 UART | 当前控制器实例为单电机 | 项目 A 总线负载明显更高 |
| 运动写入 | 每轴写 0x29 后等待响应 | 伺服写 0x29 后立即返回 | 项目 B 延迟和阻塞更小 |
| 运动接收 | 每周期接收六个写响应 | 伺服周期不接收响应；状态定时器约 1 Hz | 项目 A 更容易受响应延迟影响 |
| 调度方式 | ROS2 worker 定时器 + 共享 UART 锁 | 独立伺服线程 + `sleep(0.02)` | 项目 A 存在六轴顺序时间偏移 |
| 帧间隔 | 默认最小 1 ms | 未在代码中强制 1 ms | 项目 A 额外加入固定等待 |
| 超时 | 50 ms 阻塞读取 | 响应型命令使用阻塞读取 | 项目 A 运动路径必经阻塞读取 |
| USB 延迟 | 未配置适配器 latency timer | 尝试设置为 1 ms | 可能造成实际延迟差异 |

项目 A 的一次运动周期实际是：

```text
ID1 写入 -> 等待响应 -> ID2 写入 -> 等待响应 -> ... -> ID6 写入 -> 等待响应
```

项目 A 会记录写周期超过目标周期的 deadline miss：[motor_driver_node.py:576](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:576)、[motor_driver_node.py:594](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:594)。

## 3. 电机模式与寄存器

### 项目 A

- 控制模式常量为 `SPEED_MODE = 2`。
- 默认速度为 `50` 步/s。
- 配置块写入 `0x25..0x28`，目标位置 `0x29` 单独写入。
- 没有配置显式加速度或减速度寄存器。
- `auto_configure` 默认关闭，因此实际运行模式必须通过只读寄存器确认。

依据：[safety.py:28](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py:28)、[safety.py:36](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py:36)、[six_motor_controller.py:437](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:437)。

### 项目 B

- 支持定位模式 `0`、伺服模式 `1`、速度模式 `2`。
- 伺服模式要求持续更新目标位置，并由电机内部执行位置插补。
- 定位模式和速度模式则由电机内部完成相应运动。
- 代码没有设置显式加速度/减速度参数。

## 4. 反馈与闭环

项目 A：

- 每次目标写入响应都包含状态反馈。
- 反馈会发布为 `/pus6/joint_states`。
- 控制器使用反馈初始化整形器并进行安全检查，但默认外环修正为零；不是完整 PID 闭环。
- 活动运动期间会跳过独立后台轮询，避免进一步占用 UART。

项目 B：

- 伺服目标生成不使用反馈。
- ROS2 状态定时器约每秒读取一次状态。
- 没有外部 PID 或轨迹跟踪器。

## 5. 根因候选排序

### 高概率

1. **电机控制模式不同**：项目 A 的设计模式是速度模式，项目 B 的平滑路径是伺服模式。两种模式对连续目标更新的处理语义不同。
2. **项目 A 的六轴阻塞式请求/响应**：每个目标点要完成六次写入和六次接收，导致周期变长、轴间时间偏移和调度抖动。
3. **50 Hz 生产与 25 Hz UART 消费不匹配**：中间轨迹点会被 Latest-Value-Wins 覆盖，实际发送轨迹比控制器生成的轨迹稀疏。

### 中概率

1. USB-UART latency timer 不同。
2. 项目 A 的 1 ms 帧间隔和共享 UART 锁增加了额外等待。
3. 项目 A 的反馈、恢复和其他 UART 操作可能与运动 worker 竞争总线。

### 低概率

1. 整数步数四舍五入造成的微小量化。
2. Project B 自身的 `sleep(0.02)` 调度抖动。
3. 非运动命令的重试逻辑；项目 A 的运动写入明确使用零重试。

## 6. 建议的下一步只读验证

1. 只读读取两个项目运行时的 `0x25`、`0x28`、`0x29`，确认真实控制模式、速度和目标值。
2. 记录 UART 实际发送时间、响应完成时间、周期时间和六轴之间的时间差。
3. 用单电机、相同 50 Hz 目标流比较两种模式，先在 Mock/FakeBus 中验证调度，再按既有安全授权进行硬件验证。
4. 只读检查两个 USB-UART 设备的 latency timer。
5. 对齐记录“目标位置、实际位置、发送时间”，区分主机阶梯输入和电机内部模式造成的差异。

## 最终判断

最值得优先验证的是：**项目 A 是否在速度模式下，以阻塞式六轴请求/响应方式运行；项目 B 是否在伺服模式下，以无响应 50 Hz 位置刷新方式运行。** 这两个软件层差异足以解释明显的运动平滑度差异。
