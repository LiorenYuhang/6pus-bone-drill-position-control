# J5-C2 最终总结与冻结记录

**冻结日期**：2026-08-20  
**适用范围**：J5-C1、J5-C2-SW、J5-C2-RT-SW 的软件交付与验证边界  
**当前结论**：`J5-C1 = PASS / FROZEN`，`J5-C2-SW = PASS / FROZEN`，`J5-C2-RT-SW = PASS / FROZEN`

本文件是 J5-C2 软件阶段的收尾记录。它冻结已经验证的软件接口、数学核心和调度约束，不把软件仿真或数值一致性扩大解释为真实机械 FK 精度。完整 6PUS 机构尚未装配，两个 IMU 仍在同一块测试板上，因此 J5-C2-HW、最终 IMU 外参、ESKF 和 Dual-IMU/FK 融合均不在本阶段实施。

## 1. C1/C2 架构总览

J5-C2 的完整数据流为：

```text
/pus6/joint_feedback/timed
        TimedJointFeedback
              |
              v
FixedLagJointSynchronizer  -- 六轴历史、RLock、严格 bracket
              |
              v
             q_sync  -- 固定延迟查询时刻的六轴同步位置
              |
              v
       numerical FK (LM)
              |
              +--> /pus6/fk/state  (FkStateStamped)
              +--> /pus6/fk/pose   (geometry_msgs/PoseStamped)
```

J5-C1 提供统一的 Host 时间合同和 UART 批次语义；J5-C2-SW 在该合同上完成固定延迟状态重建和 FK 数学求解；J5-C2-RT-SW 将上述链路放进标称 200 Hz 的 ROS 2 节点，并冻结执行器架构。

## 2. Timed telemetry 与 `CLOCK_MONOTONIC`

源码接口位于 `bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg`。每个轴的 `per_axis_rx_done_monotonic_ns[6]` 是 Host 完成该轴 0x29 应答接收的时间，不是电机内部采样时刻；`snapshot_start_monotonic_ns` 和 `snapshot_end_monotonic_ns` 描述整组扫描跨度。`header.stamp` 只表示 ROS 消息发布时间。

算法时间一律来自 Python `time.clock_gettime_ns(time.CLOCK_MONOTONIC)`，不使用可跳变的 wall clock。ROS `Header` 继续用于消息传输和记录，但不得替代 Fixed-Lag 查询和性能统计所需的 monotonic 时间。

## 3. Fixed-Lag、`q_sync` 与 0 extrapolation

`FixedLagJointSynchronizer` 位于 `bone_ws/src/pus6_kinematics/pus6_kinematics/state_synchronizer.py`，为六个轴分别维护有界历史队列。FK 定时器在当前 monotonic 时刻减去固定延迟后得到 `t_query`，然后对每个轴寻找：

```text
t_i[k-1] <= t_query <= t_i[k]
q_i(t_query) = q_i[k-1] + alpha_i * (q_i[k] - q_i[k-1])
```

只有六个轴都被 bracketed 才能生成 `q_sync`。任一轴缺少未来样本、查询点早于历史窗口或时间点重复，均返回等待/丢弃状态；绝不向前或向后外推。该规则把 UART 的逐轴接收相位差转换为可审计的固定延迟，而不是制造伪位姿。

同步器的公开读写方法使用 `threading.RLock` 保护六个 deque。锁只覆盖历史更新、边界检查和插值读取，不承担硬件通信，也不改变 Latest-Value-Wins 的上游语义。

## 4. FK 数学模型与 IK→FK 验证

实现位于 `bone_ws/src/pus6_kinematics/pus6_kinematics/analytical_6pus.py`，使用冻结的真实几何参数：

* base radius `c = 40 mm`
* platform radius `d = 27.5 mm`
* strut length `e = 84 mm`
* mounting angles `lambda = 30 deg`、`mu = 60 deg`

第 `i` 条支链满足 `||p + R_ZYX b_i - s_i(q_i)|| - e = 0`。求解器对 ZYX 欧拉参数显式计算旋转矩阵的三个解析偏导，组成 6x6 解析 Jacobian，并使用自适应阻尼 Levenberg-Marquardt：残差下降时减小阻尼，残差上升时增大阻尼。

预定义安全包络的 1000 个 IK→FK round-trip 样本全部收敛：平移数值一致性残差均值 `5.83e-9 mm`，姿态残差均值 `1.21e-7 deg`，几何约束残差均值 `1.58e-11 m`，平均迭代 `4.93` 次。这些数字证明代码中的正逆解数值一致性，不证明加工、装配、球铰间隙或编码器零位的物理精度。

## 5. Warm Start 与 Stewart assembly branch

Stewart 并联机构存在多个代数装配分支。FK 成功后将当前位姿保存为下一次求解的初值（Warm Start），使连续数据留在当前分支的局部吸引域内，并降低迭代次数。Warm Start 是连续性策略，不是对未知机械装配状态的自动识别；首次求解仍需要可信的初始位姿和软件零位。J5-C2 冻结的是当前 6PUS 数值 FK 路径，不宣称 FK 在全局上唯一。

## 6. `Pus6FkNode`、消息与门禁

实时节点位于 `bone_ws/src/pus6_kinematics/pus6_kinematics/fk_node.py`，入口为 `fk_node = pus6_kinematics.fk_node:main`。它订阅 `/pus6/joint_feedback/timed`，以 `query_rate_hz=200` 和固定延迟查询同步器，并发布：

* `/pus6/fk/state`：`pus6_msgs/msg/FkStateStamped`，包含 query monotonic 时间、`synced_positions_meters`、位姿、ZYX 角、收敛状态、LM 迭代次数、残差、分支门禁结果和计算耗时；
* `/pus6/fk/pose`：`geometry_msgs/msg/PoseStamped`，用于标准 ROS 位姿消费者。

`FkStateStamped` 的 `header.stamp` 仍是 ROS publish time；`query_monotonic_timestamp_ns` 才是该 q_sync 的算法查询时间。

节点对连续解施加 SO(3) geodesic jump gate：

```text
theta = acos(clip((trace(R_last.T @ R_curr) - 1) / 2, -1, 1))
```

当 `theta > 0.15 rad` 或平移跳变超过 `15 mm` 时拒绝本次发布且不更新 Warm Start 锚点。`WorkspaceLimitError`、`SingularityError`、`KinematicsError`、`ValueError` 和 `ArithmeticError` 在节点边界捕获，异常帧只计数/记录，不得使进程退出或发布垃圾位姿。

## 7. Executor 架构约束与 A-F 消融

`Pus6FkNode.main()` 和实时验收测试必须使用**独立的 `SingleThreadedExecutor`**。最终 200 Hz 验收为 10 秒 `2007 callbacks`、`200.09 Hz`、period `P95 ≈ 5.14 ms`；验收目标仍为 callbacks >= 1900、mean >= 190 Hz、period P95 < 7.5 ms，且不得有百毫秒级持续 stall。

在独立单线程执行器下的分层结果如下。period 统计是相邻 timer callback 间隔；callback 统计是单次 timer callback 耗时。

| 层 | 内容 | callbacks/10s | mean Hz | period P50/P95/P99/max (ms) | callback P50/P95/P99/max (ms) |
|---|---|---:|---:|---|---|
| A | 空 200 Hz timer | 2000 | 199.98 | 5.001 / 5.184 / 5.369 / 5.570 | 0.001 / 0.002 / 0.003 / 0.092 |
| B | +30 Hz publisher | 2001 | 200.06 | 5.001 / 5.199 / 5.428 / 5.661 | 0.001 / 0.002 / 0.003 / 0.007 |
| C | +Fixed-Lag，不做 FK | 2001 | 200.07 | 5.001 / 5.304 / 5.635 / 6.224 | 0.045 / 0.091 / 0.115 / 0.309 |
| D | +FK，不发布 FK 结果 | 2000 | 199.97 | 4.999 / 5.321 / 5.707 / 5.915 | 0.920 / 1.461 / 1.837 / 2.750 |
| E | 完整节点，无异常 | 2000 | 199.98 | 4.999 / 5.270 / 5.674 / 5.936 | 1.079 / 1.663 / 2.014 / 2.572 |
| F | 完整节点 + 异常注入 | 2000 | 199.97 | 4.999 / 5.162 / 5.338 / 5.807 | 1.133 / 1.802 / 2.574 / 5.451 |

对照运行显示，`MultiThreadedExecutor(4)` 下 A/B 尚约 200 Hz，但加入同步器和 FK 后 C-E 仅约 25--38 Hz，period P95 约 145--234 ms、最大间隔约 650--980 ms；单次 callback P99 仍约 3.5 ms。真实根因是 Python `rclpy` 多线程执行器在 publisher/subscriber/timer 混合 ready-callback 负载下的调度 starvation，受到 GIL、CallbackGroup/ready-callback 争用放大；没有证据表明 NumPy 单次计算、`RLock` 等待、logging 或 ROS publish 单次耗时造成百毫秒停顿。故不能用降低门限制造 PASS，也不能把多线程结果作为 FK 节点运行方式。

全 workspace 的当前验证结果为 **162 tests PASS**（新增几何配置与 C2-HW 离线工具链测试后）。本文件冻结的是软件验收结论；本轮收尾不重新运行实机运动。

## 8. 软件验证边界与机构装配后的入口

以下条件未满足前，J5-C2-HW 不启动：完整 6PUS 机构总装、最终杆长/安装角/机械零位复核、两个 IMU 的最终空间安装。装配完成后按以下顺序进入下一阶段：

1. 核对最终机械几何、杆长、安装角、软件零位和 `OPERATING_HOME`；确认 `STORAGE_PARK` 只用于收拢停放。
2. 执行 J5-B2 最终双 IMU 外参与 lever arm 标定，固定坐标系方向和旋转约定。
3. 启动 J5-C2-HW：`q_actual → TimedJointFeedback → Fixed-Lag → q_sync → FK → 实际 T_BP`，用真实平台测量验证 FK 位姿。
4. 只有实际 `T_BP` 验证通过后，才规划平动模型、Dual-IMU/FK 融合和 ESKF。当前不提前启动 C3 或 ESKF。

## 9. 当前跨阶段状态索引（文档统一口径）

| 阶段 | 当前状态 |
|---|---|
| J4 控制算法/安全策略 | **PASS / FROZEN**；UART transport 已应用 batch-atomicity hotfix |
| J5-A | **PASS / FROZEN** |
| J5-B1 | **PASS / FROZEN** |
| J5-B2 | **DEFERRED**，等待最终双 IMU 外参与 lever arm 安装条件 |
| J5-B3 | **PASS / FROZEN**；当前数据仍来自同板共刚体验证 |
| J5-B4 | **PENDING** |
| J5-C1 | **PASS / FROZEN** |
| J5-C2-SW | **PASS / FROZEN** |
| J5-C2-RT-SW | **PASS / FROZEN**；独立 `SingleThreadedExecutor` 约束 |

以上软件结论不代表完整机构的真实机械 FK 精度。当前机构未完整装配、两个 IMU 仍在同一块测试板上；最终外参、lever arm、实机 FK、融合和 ESKF 均未开始。
