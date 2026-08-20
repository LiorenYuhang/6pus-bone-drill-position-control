# 6PUS 装配前最终软件 Baseline

**Baseline 日期**：2026-08-20  
**适用范围**：P0 几何配置唯一来源与 HOME/PARK 安全状态收敛后的软件冻结状态  
**当前入口**：等待完整机构装配后进入 J5-C2-HW  
**本文件性质**：装配前软件基线记录，不代表真实机械精度，也不替代实机安全授权。

## 1. 当前系统状态

### 1.1 阶段状态

| 阶段 | 状态 | 边界 |
| --- | --- | --- |
| J1 UART/底层协议 | PASS / FROZEN | 单总线半双工 921600 8N1；UART transport 后续已应用 batch-atomicity hotfix |
| J2 六轴通信与性能 | PASS / FROZEN | 包括只读、单轴、六轴向量、追踪与通信性能基线 |
| J3 IK/执行器启动与基础运动 | PASS / FROZEN | J3 行为与速度语义保持冻结；HOME/PARK 使用统一配置 |
| J4 连续笛卡尔控制与安全策略 | PASS / FROZEN | 控制算法、安全包络、Latest-Value-Wins 与 UART hotfix 均冻结 |
| J5-A 接口与数据流准备 | PASS / FROZEN | ROS 2 消息、topic 和时间数据接口已冻结 |
| J5-B1 双 IMU 软件时间同步 | PASS / FROZEN | 软件时钟映射与频偏补偿完成 |
| J5-B2 最终双 IMU 外参与 lever arm | DEFERRED | 等待最终机构安装 |
| J5-B3 双 IMU 相对姿态核心 | PASS / FROZEN | 当前仅为两只 IMU 同板共刚体验证 |
| J5-B4 平动/加速度融合 | PENDING | 尚未开始 |
| J5-C1 Timed telemetry/统一时间轴 | PASS / FROZEN | Host `CLOCK_MONOTONIC`、UART sequence、batch atomicity 已冻结 |
| J5-C2-SW 数值 FK | PASS / FROZEN | FK、解析 Jacobian、LM、IK→FK round-trip、Warm Start 已冻结 |
| J5-C2-RT-SW FK 实时节点 | PASS / FROZEN | 独立 `SingleThreadedExecutor` 是硬约束 |
| J5-C2-HW 实际机构 FK | DEFERRED | 必须等机构总装、几何确认和零位确认 |

### 1.2 当前冻结结论

- J4 控制算法和安全策略未因后续文档/配置收敛而改变。
- UART transport 已应用六轴 batch-atomicity hotfix；不能将 J4 描述为“之后完全未修改”。
- J5-C2 软件验收只证明软件链路和数值模型；不代表真实加工、装配、间隙、零位或编码器误差下的机械 FK 精度。
- 完整 6PUS 机构尚未装配；两个 H30 IMU 仍固定在同一块测试板。
- J5-C2-RT-SW 独立 `SingleThreadedExecutor` 最终实测为 `2007 callbacks / 10 s`、`200.09 Hz`、period `P95 ≈ 5.14 ms`；`MultiThreadedExecutor` 在完整 FK 负载下会产生严重 callback starvation，不能作为 FK 节点运行方式。

## 2. 当前架构

### 2.1 ROS 2 节点关系

```text
target_cartesian_pose / target_cartesian_array
                |
                v
       pus6_kinematics_node
       (analytical IK + J4 envelope)
                |
      /pus6/desired_joint_deltas
                |
                v
          pus6_control_node
       (jerk-limited reference shaper)
                |
       /pus6/command_joint_steps
                |
                v
       pus6_motor_driver_node
       (Latest-Value-Wins + preflight + UART)
          |                    |
          |                    +--> /pus6/joint_states
          |                    +--> raw diagnostics
          v
  /pus6/joint_feedback/timed
  TimedJointFeedback
          |
          v
       pus6_kinematics/fk_node
       (独立 SingleThreadedExecutor)
          |                    |
          +--> /pus6/fk/state  FkStateStamped
          +--> /pus6/fk/pose   PoseStamped
```

可选的 `h30_imu_node` 属于 IMU 数据采集链路；当前没有把它接入 ESKF 或 Dual-IMU/FK 融合控制环。

### 2.2 数据流

控制路径：

```text
Cartesian target
 -> analytical IK
 -> desired joint deltas
 -> jerk-limited control
 -> command_joint_steps
 -> motor driver preflight
 -> UART 0x29 batch
```

观测路径：

```text
motor feedback
 -> TimedJointFeedback
 -> FixedLagJointSynchronizer
 -> q_sync (strictly bracketed, zero extrapolation)
 -> numerical FK (LM)
 -> FkStateStamped / PoseStamped
```

FK 观测节点不发布运动命令，也不依赖 HOME/PARK 服务。

### 2.3 Executor 约束

- `fk_node` 必须创建并使用独立 `SingleThreadedExecutor`。
- 该约束来自实际 A-F 性能消融：Python `rclpy` 混合 publisher/subscriber/timer 负载下，`MultiThreadedExecutor` 会造成严重 callback starvation。
- `pus6_motor_driver_node` 仍有自己的驱动线程/回调组结构；不能将其历史多线程配置外推为 FK 节点的实时方案。

## 3. Geometry 唯一来源

### 3.1 权威配置文件

```text
bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml
```

该 YAML 是几何和运动参数的唯一生产来源，包含：

- base/platform anchors、半径、安装角和 strut length；
- nominal height 和 workspace height；
- `steps_per_meter` / `meters_per_step`；
- `STORAGE_PARK`、`OPERATING_HOME`；
- stroke limits；
- absolute raw、software command、normal motion 三层 step limits；
- IMU extrinsic/lever-arm 预留字段及 pending confirmation 标记。

当前软件契约为：`STORAGE_PARK=[0]*6`、`OPERATING_HOME=[1000]*6`、绝对 raw `[0,2000]`、普通 J4 运行包络 `[200,1800]`。

### 3.2 读取关系

```text
6pus_geometry.yaml
        |
        v
geometry_config.load_geometry_config()
        |
        +--> analytical_6pus / kinematics_node
        +--> joint_trajectory_controller / pus6_control_node
        +--> pus6_motor_driver.safety / motor_driver_node
        +--> J3 startup/homing runner (through shared driver/kinematics imports)
```

`system_params.yaml` 和 `motors.yaml` 只保留运行时参数，不再重复 geometry、steps/meter、HOME/PARK 或软件限位。显式构造器参数仍可用于测试 override；生产默认值必须来自上述 YAML。

## 4. HOME/PARK 安全状态机

### 4.1 `OPERATING_HOME`

- raw 目标：`[1000,1000,1000,1000,1000,1000] step`。
- 相对控制坐标：六轴均为 `0 m`。
- 约对应每轴 50 mm 伸出，是 J4 正常运动的控制基准。
- 普通 J4 轨迹必须处于该工作基准附近，并受 `[200,1800]` normal motion envelope 约束。

### 4.2 `STORAGE_PARK`

- raw 目标：`[0,0,0,0,0,0] step`。
- 语义：最短位置、机械收拢和停放维护状态。
- PARK 不是普通 J4 工作空间的一部分，不能用来绕过正常运动安全限制。

### 4.3 `maintenance_mode`

- 默认值：`false`。
- `pus6_control` 的 `/pus6/controlled_park` 在默认模式下拒绝 PARK，并提示使用授权的维护流程。
- `pus6_motor_driver` 普通 preflight 仍强制 `[200,1800]`；只有显式 `maintenance_mode=true` 才允许 raw PARK 进入绝对范围 `[0,2000]`。
- inter-frame delta、反馈新鲜度、fault、通信和 batch-atomicity 等其他安全门禁不因 maintenance mode 消失。
- J3 独立 HOME/PARK runner 仍需原有 `--real-hardware --confirm-motion --validated-config` 显式实机门禁。

### 5.1 当前可复现结果

在 ROS 2 Jazzy 和 workspace 环境已 source、无真实硬件运动的条件下，严格区分两类测试范围：

1. **当前 ROS 2 工作空间核心测试集**（唯一权威运行基准）：
```bash
pytest /home/lyapunov/bone_drill/bone_ws/src -v
# 输出: 162 passed in ~39.85s (100% PASS)
```
覆盖 `pus6_kinematics` (35), `pus6_motor_driver` (60), `pus6_control` (67), `h30_imu`。

2. **根目录全仓库历史用例收集**（含历史独立台架用例）：
```bash
pytest /home/lyapunov/bone_drill --collect-only -q
# 输出: 468 tests collected (162 in bone_ws/src + 306 in up/motor/uart_multi)
```
两者范围不同，严禁混淆使用。

### 5.2 Mock/安全验证

- 几何 YAML loader、分析 FK、控制器、J3 runner 和驱动 safety focused tests 通过。
- J4 pipeline Mock 与 J3 IK-driver integration 通过。
- 默认驱动模式下 raw `[0]*6` preflight 被拒绝。
- 显式 `maintenance_mode=true` 时 raw `[0]*6` preflight 通过。
- 两种验证均使用 Mock，`hardware_write_count = 0`。
- 本 baseline 生成过程未执行 HOME、PARK、J3 Cartesian 或 J4 实机运动。

## 6. 已知未完成事项

以下事项不是本软件 baseline 的 PASS 项，必须在完整机构装配后按门禁顺序执行：

1. 完成 6PUS 机械总装，核对最终杆长、安装角、anchor 几何、行程和机械零位。
2. 实测确认 `STORAGE_PARK`、`OPERATING_HOME` 与六轴 zero mapping；必要时更新 YAML 后重新回归。
3. 完成 J5-B2 最终双 IMU 外参与 lever arm 标定。当前同板共刚体验证不能替代最终安装参数。
4. 启动 J5-C2-HW，执行真实 `q_actual → TimedJointFeedback → Fixed-Lag → q_sync → FK`，验证实际 `T_BP` 和残差稳定性。
5. 在实际 FK 验收通过后，才进入平动模型、Dual-IMU/FK 融合和 ESKF。

当前禁止提前开始 ESKF、Dual-IMU/FK 融合或 C3，也不把数值 IK→FK round-trip 结果写成机械精度。

## 7. 关键文件 SHA256

以下 hash 对应本 baseline 生成时的当前工作树文件内容；文档或源码后续变化后必须重新计算。

| 文件 | SHA256 |
| --- | --- |
| `agent_docs/CURRENT_STAGE.md` | `33ad70692719c2e3d37c542c004196a03e7daab5f08ac8d33695b51c42b4b06d` |
| `agent_docs/J5_FROZEN_MANIFEST.md` | `ebf6777cb910e9e6878a28f58e0902d783bc6742d094616e4e5032f1927c9980` |
| `agent_docs/PRE_ASSEMBLY_DOCUMENT_AND_HARDWARE_ACCEPTANCE.md` | `991b6b0d1f058e74af1abad3d1eed20228e6402c388f650c52c407af5e0ae5f4` |
| `agent_docs/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md` | `298fd16b969d2518061f477c87aa5aa665a43e8282f7e82854d549ab37ded1db` |
| `agent_docs/J4_FINAL_SUMMARY.md` | `44e8d027b69ba8c60422ab740f0f317e3805cfad745b53f5ffd6a586667f19a9` |
| `agent_docs/J5_B3_FINAL_SUMMARY.md` | `aa7b62fb3a1bb86a281e1dfa5cd3bdcc216d8b5381a49330001b6bdb7554f81c` |
| `agent_docs/J5_C2_FINAL_SUMMARY.md` | `8b1d203f57824e10aad91ae396f34380b4a11cba9237a4479595b19857aedf73` |
| `agent_docs/J5_C2_HW_ACCEPTANCE_TOOLCHAIN.md` | `e250e0a22560838d09d93eacd868eb99f44b10ee95254613207db1845b3f61b6` |
| `6PUS_ROS2_CODE_LEARNING_TUTORIAL.md` | `1d21ad8ff526dcc00a44a853baeea3210cef78c6dada4d6b1c94960b48499f43` |
| `6PUS_REAL_HARDWARE_OPERATION_GUIDE.md` | `f4cb6af2fbd8a752ffb0a96007734777803d0aa37f08a4b4f4b925c8d11d1753` |
| `bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml` | `29cc2a8d4fe3981586471a3b9fb01ff894bfee211cfade901678cd01dc557e24` |
| `bone_ws/src/pus6_kinematics/pus6_kinematics/geometry_config.py` | `f4e12038e063ffd985e29c64227ff3a693cc272f0a07663359afb575322ea365` |
| `bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py` | `5c448532644f772fa8ab906bc04b9f0b7b5ebbc007b6ea80cb3dc2607bca9800` |
| `bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py` | `c8a7df4e788a1e70c8188fa728fd6584e0be13d54d46c7965ae422e01e06b3e0` |
| `bone_ws/src/pus6_kinematics/pus6_kinematics/state_synchronizer.py` | `029678cbee89e40c3efbe5f517f0b70fc646d3e25722795fe2af2ea66758b17f` |
| `bone_ws/src/pus6_control/pus6_control/control_node.py` | `11dbf86bc2a0af011c96e315ea5def486e0c8e127513e8dd0a10bdd08745ec5c` |
| `bone_ws/src/pus6_control/pus6_control/joint_trajectory_controller.py` | `7d6abba1d89a27a759408f2a937109c05a0bb548ee8710c999632f45c466952f` |
| `bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py` | `ab45f36044bd613aa60cf89030ffe406815b1054e4b9304542ad5ab02e582aad` |
| `bone_ws/src/pus6_motor_driver/pus6_motor_driver/motor_driver_node.py` | `7f67f802763a48a6e40d1ae9ee0ac44386bb5e209dffd743f5169b359db30340` |
| `bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg` | `b48424f3ced75faf39a1f2bbfef8f3cf7e3ede87ff253ae6276cad2fd1cec830` |
| `bone_ws/src/pus6_msgs/msg/TimedJointCommand.msg` | `eb321809f57e61f4346e49578b4858fc8f5ba3ac35e3071bcead7a6f5dd09696` |
| `bone_ws/src/pus6_msgs/msg/FkStateStamped.msg` | `5b1e76998b58312415433ec734aed0cc300ef2c1547891a54a1e1690723c33fe` |
| `bone_ws/src/pus6_bringup/config/system_params.yaml` | `9c83fc605594e0444e09cbdef7c4f2a1252e3f0918034a1931987df8a3823586` |
| `bone_ws/src/pus6_motor_driver/config/motors.yaml` | `d567427d3ac5e6fb106e7ae9bdcc53017278857d74ae1c9e542fdeb71df04d42` |
| `bone_ws/scripts/run_j3_startup_homing.py` | `be6d4bc2e733f59b39f335e4321427ce215ab1642e423fa1a8f3074775b845b6` |

## 8. Baseline 解除条件

本 baseline 只能在以下条件满足后被下一阶段基线替代：

```text
完整机构装配
 -> 最终 geometry / zero mapping 实测确认
 -> HOME/PARK 现场确认
 -> J5-B2 最终双 IMU 外参与 lever arm
 -> J5-C2-HW q_actual -> FK -> 实际 T_BP 验收
 -> 才允许规划 Dual-IMU/FK 融合与 ESKF
```

在此之前，冻结当前 P0 软件状态，不增加新的 J5 功能。
