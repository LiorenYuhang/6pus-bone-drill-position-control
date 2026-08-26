# J7-B0：J6 Frozen Baseline 运行模式只读审查

## 1. 审查范围

- 项目：`/home/lyapunov/bone_drill`
- 当前阶段：J7-B0，J6 运行配置只读确认
- 审查对象：J6 HOME/PARK standalone runner、通用 ROS2 driver、配置文件、UART 寄存器写入路径
- 本次工作：仅代码和冻结文档审查
- 本次未执行：硬件连接、串口读写、运动命令、代码/配置/参数修改

结论必须区分两条路径：

1. **J6 frozen HOME/PARK standalone 路径**：显式使用 `mode=0`。
2. **通用 ROS2 driver 路径**：默认配置/服务仍使用 `mode=2`。

## 2. 配置链路图

### 2.1 J6 frozen HOME/PARK 实际路径

```text
J6 Phase 4 wrapper
  run_j6_phase4_home_park.py
    -> run_j3_startup_homing.run_homing_session()
      -> POSITIONING_MODE = 0
      -> controller.configure_all(mode=0, speed=motor_reg_speed)
        -> SixMotorController.configure_all()
          -> Motor.write_register(0x25, [0, 0, 0, motor_reg_speed])
            -> CMD_WR_REGISTER (0x32)
              -> UART
                -> 电机控制模式寄存器 0x25

J6 action=home
  -> OPERATING_HOME_RAW_STEPS = [1000]*6
J6 action=park
  -> STORAGE_PARK_RAW_STEPS = [0]*6
  -> quintic trajectory @ 10 Hz
  -> controller.command_positions(step_cmd)
    -> 六轴顺序写入 0x29
      -> UART
        -> 电机目标位置寄存器 0x29
```

关键代码：

- J6 wrapper 固定 commissioning speed 为 50：[run_j6_phase4_home_park.py:29](/home/lyapunov/bone_drill/bone_ws/scripts/run_j6_phase4_home_park.py:29)
- J6 wrapper 委托 J3 homing runner：[run_j6_phase4_home_park.py:136](/home/lyapunov/bone_drill/bone_ws/scripts/run_j6_phase4_home_park.py:136)
- J3 明确声明 `POSITIONING_MODE = 0`：[run_j3_startup_homing.py:60](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:60)
- J3 HOME/PARK 目标选择：[run_j3_startup_homing.py:272](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:272)
- J3 配置模式和速度：[run_j3_startup_homing.py:342](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:342)、[run_j3_startup_homing.py:355](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:355)
- J3 插值并发送目标：[run_j3_startup_homing.py:393](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:393)、[run_j3_startup_homing.py:401](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:401)

### 2.2 通用 ROS2 driver 路径

```text
system_params.yaml / motors.yaml
  -> pus6_system.launch.py
    -> Pus6MotorDriverNode
      -> auto_configure=true 时 controller.configure_all()
      -> /pus6/configure_motors 服务时 controller.configure_all()
        -> SixMotorController.configure_all()
          -> 默认 mode=SPEED_MODE=2, speed=DEFAULT_SPEED=50
            -> 写 0x25..0x28
```

当前 ROS2 节点没有声明或读取 `mode`、`speed` 参数；它只在 `auto_configure` 开启或配置服务调用时执行无参数的 `configure_all()`：[motor_driver_node.py:99](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:99)、[motor_driver_node.py:294](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:294)、[motor_driver_node.py:855](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:855)。

因此，YAML 中的 `mode: 2`、`speed: 50` 不是当前 ROS2 节点的直接参数来源。它们与代码默认值一致，但当前节点不从 YAML 字段读取它们。

## 3. mode / speed 来源分析

### 3.1 mode=0 Positioning Mode

J6 frozen HOME/PARK 的 mode=0 来源如下：

```text
run_j3_startup_homing.py
  POSITIONING_MODE = 0
    -> configure_all(mode=POSITIONING_MODE, ...)
      -> payload = [0, 0, 0, motor_reg_speed]
        -> 起始寄存器 0x25
```

`configure_all()` 会逐轴写入并读回校验 0x25 和 0x28：[six_motor_controller.py:437](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:437)、[six_motor_controller.py:455](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:455)、[six_motor_controller.py:460](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:460)。

### 3.2 mode=2 SPEED_MODE 默认路径

代码中存在多条 mode=2 路径：

| 来源 | 行为 |
|---|---|
| `safety.py:SPEED_MODE` | 常量值为 `2` |
| `SixMotorController.configure_all()` | 默认 `mode=SPEED_MODE` |
| ROS2 `auto_configure=true` | 调用无参数 `configure_all()`，最终为 mode=2 |
| `/pus6/configure_motors` 服务 | 调用无参数 `configure_all()`，响应文本明确写 mode=2、speed=50 |
| J2/J3 通用测试、FakeBus 初始状态 | 多处使用或模拟 mode=2 |

依据：[safety.py:36](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py:36)、[six_motor_controller.py:439](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:439)、[motor_driver_node.py:867](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py:867)。

`motors.yaml` 和 `system_params.yaml` 也保留 `mode: 2`、`speed: 50` 字段，但当前节点没有读取这两个字段：[system_params.yaml:12](/home/lyapunov/bone_drill/bone_ws/src/pus6_bringup/config/system_params.yaml:12)、[motors.yaml:9](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/config/motors.yaml:9)。

### 3.3 mode=1 Servo Mode

项目 A 没有独立的 `SERVO_MODE` 常量、Servo 专用 ROS2 参数、Servo 专用 runner 或无响应位置发送接口。

但 `configure_all(mode: int, ...)` 本身没有禁止任意整数 mode，因此从通用 API 角度可以传入 `mode=1`：[six_motor_controller.py:437](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:437)。这只是潜在的通用写入能力，不是当前 J6 或 ROS2 的 Servo 支持路径。

另外，J2-B/J2-C 的专用配置函数明确只接受 `SPEED_MODE=2`，因此它们不能直接作为 Servo 实验入口：[six_motor_controller.py:607](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:607)、[six_motor_controller.py:718](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:718)。

### 3.4 speed=50 来源与 0x28 写入

`speed=50` 有三个相关来源：

1. 驱动默认常量 `DEFAULT_SPEED = 50`：[safety.py:37](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py:37)。
2. J6 Phase 4 门禁常量 `COMMISSIONING_SPEED_STEP_S = 50`：[run_j6_phase4_home_park.py:29](/home/lyapunov/bone_drill/bone_ws/scripts/run_j6_phase4_home_park.py:29)。
3. J3 runner 命令行参数 `--speed` 默认使用 `DEFAULT_SPEED`：[run_j3_startup_homing.py:607](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:607)。

J3 runner 并不直接把 `speed` 原值写入 0x28，而是计算：

```text
motor_reg_speed = max(50, round(planned_peak_speed * 1.2))
```

然后写入：

```text
0x25 = mode
0x26 = 0
0x27 = 0
0x28 = motor_reg_speed
```

依据：[run_j3_startup_homing.py:330](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:330)、[run_j3_startup_homing.py:342](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:342)、[six_motor_controller.py:455](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:455)。

因此：

- `speed=50` 是 J6 commissioning 的规划速度/门禁值；
- 实际 `0x28` 值是 `motor_reg_speed`；
- 当规划峰值速度超过约 `41.67 step/s` 时，公式可能写入大于 50 的值；
- J3 报告会记录实际派生值 `motor_reg_speed_0x28`：[run_j3_startup_homing.py:567](/home/lyapunov/bone_drill/bone_ws/scripts/run_j3_startup_homing.py:567)。

### 3.5 是否每次启动重新配置

| 运行入口 | 是否启动时写 mode/speed |
|---|---|
| J6 Phase 4 / J3 standalone | 是。每次 `run_homing_session()` 在轨迹前调用 `configure_all(mode=0, ...)` |
| ROS2 driver 默认启动 | 否。`auto_configure=false` |
| ROS2 driver `auto_configure=true` | 是，但写入默认 mode=2、speed=50 |
| `/pus6/configure_motors` 服务 | 调用服务时写入默认 mode=2、speed=50 |
| 没有任何配置调用 | 不写入，电机保留上电/持久化的寄存器状态；代码审查不能推断该值 |

## 4. 0x25~0x29 寄存器使用关系

| 寄存器 | 用途 | 当前代码来源 | J6 实际使用 |
|---|---|---|---|
| `0x25` | 控制模式 | `CONTROL_MODE_ADDR`；`configure_all()` 起始地址 | `0`，Positioning Mode |
| `0x26` | 电压模式电压 | `configure_all()` payload 第 2 项固定为 `0` | 配置阶段写 `0`，J6 不使用电压控制 |
| `0x27` | 力控目标 | `configure_all()` payload 第 3 项固定为 `0` | 配置阶段写 `0`，J6 不使用力控 |
| `0x28` | 目标速度/速度限制 | `TARGET_SPEED_ADDR`；J3 计算 `motor_reg_speed` | 写入派生的 `motor_reg_speed`；J6 文档基线工作点为 50，但应以报告字段/读回为准 |
| `0x29` | 目标位置 | `TARGET_POSITION_ADDR`；`command_positions()` | HOME/PARK 轨迹点，最终目标分别 `[1000]*6` / `[0]*6` |

`configure_all()` 明确只配置到 `0x28`，不会隐藏写入 `0x29`：[six_motor_controller.py:442](/home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/six_motor_controller.py:442)。

## 5. J6 frozen 状态确认

### A. J6 frozen baseline 是否真的运行在 mode=0？

**结论：对 J6 frozen HOME/PARK standalone baseline，答案是“是”。**

理由：

- J6 Phase 4 委托 J3 homing runner；
- J3 runner 硬编码 `POSITIONING_MODE = 0`；
- 轨迹前显式执行 `configure_all(mode=0, ...)`；
- J6 冻结文档记录 HOME/PARK 的验证配置为 `0x25=0`、Positioning Mode、speed=50：[J6_HOME_PARK_VALIDATED_BASELINE_AND_10_CYCLE_PROCEDURE_CN.md:20](/home/lyapunov/bone_drill/agent_docs/j6/J6_HOME_PARK_VALIDATED_BASELINE_AND_10_CYCLE_PROCEDURE_CN.md:20)。

但本次没有连接硬件，因此不能把“当前电机现场寄存器瞬时值”重新宣称为已读回；本报告确认的是 frozen runner 的写入路径和已有冻结证据。

**该结论不适用于通用 ROS2 bringup 的所有启动方式。** ROS2 默认不自动配置；若调用默认配置服务或开启 `auto_configure`，则写入 mode=2。

### B. 为什么上一份审计报告显示设计模式可能是 mode=2？

上一份审计针对的是整个 `bone_drill` 通用驱动，而不是 J6 frozen standalone runner。通用驱动存在以下 mode=2 证据：

- `SPEED_MODE = 2`；
- `configure_all()` 默认参数是 mode=2；
- ROS2 配置文件保留 mode=2；
- ROS2 配置服务文本明确报告 mode=2、speed=50；
- FakeBus 初始模拟状态为 mode=2；
- J2/J3 的其他通用测试大量使用 mode=2。

因此，“通用驱动设计默认可能为 mode=2”和“J6 frozen HOME/PARK runner 显式使用 mode=0”是两个不同层级的结论，并不矛盾。

补充：`test_j6_phase4_home_park.py` 中有一个底层寄存器分离测试使用 `configure_all(mode=2, speed=50)`；该测试只验证配置阶段不写 0x29，并不改变生产 J6 runner 的 `mode=0` 路径，也不能作为 J6 实际模式证据。

### C. HOME/PARK 实际链路

#### HOME

```text
J6 action=home
 -> OPERATING_HOME_RAW_STEPS
 -> [1000,1000,1000,1000,1000,1000]
 -> quintic_progress() 生成 step_cmd
 -> controller.command_positions(step_cmd)
 -> 0x29 逐轴写入
 -> 电机执行 Positioning Mode 目标运动
```

#### PARK

```text
J6 action=park
 -> STORAGE_PARK_RAW_STEPS
 -> [0,0,0,0,0,0]
 -> quintic_progress() 生成 step_cmd
 -> controller.command_positions(step_cmd)
 -> 0x29 逐轴写入
 -> 电机执行 Positioning Mode 目标运动
```

HOME/PARK 常量来自唯一几何配置源：[6pus_geometry.yaml:56](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml:56)、[6pus_geometry.yaml:57](/home/lyapunov/bone_drill/bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml:57)。

## 6. Servo Mode 实验前置条件

以下仅列出未来实验前需要审查的最小位置，不实施修改：

1. **模式常量与安全策略位置**：`bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py`。需要明确 mode=1 的命名、允许范围和与 J6 frozen 的隔离关系。
2. **配置入口位置**：`SixMotorController.configure_all()` 及其调用方。J7 实验必须使用新 runner/新配置入口，不能改变 J6 frozen runner 的 `POSITIONING_MODE=0`。
3. **目标发送位置**：`SixMotorController.command_positions_with_metrics()`、`motor_driver_node.py` 的 UART worker。Servo 模式通常需要固定频率连续更新目标，现有六轴写入路径会等待每轴响应。
4. **无响应写入位置**：当前 `protocol.py`、`uart_bus.py`、`motor.py` 属于通信基线/冻结资产，不应原地修改。若 Servo 实验要求写入后不读响应，应在新的 J7 实验模块或新增隔离适配层中处理。
5. **验证入口**：先增加/使用 J7 专属 Mock/FakeBus 审计，确认 mode=1、0x28、0x29 写入顺序和硬件写入计数，再讨论任何硬件授权。
6. **冻结保护**：J6 的 HOME/PARK 基线、`mode=0`、`[1000]*6`、`[0]*6`、speed=50 文档和历史证据不得被 Servo 实验覆盖。

## 最终结论

- **J6 frozen HOME/PARK standalone baseline：mode=0，Positioning Mode。**
- **通用 ROS2 driver：默认配置路径仍是 mode=2，但默认不自动写入。**
- **speed=50：是 J6 commissioning 规划/门禁值；0x28 的实际写入值由 `motor_reg_speed` 派生，必须以运行报告中的 `motor_reg_speed_0x28` 或只读读回为准。**
- **mode=1：当前没有正式 Servo Mode 运行路径，只存在通用 `configure_all(mode=1)` 的潜在参数能力。**
