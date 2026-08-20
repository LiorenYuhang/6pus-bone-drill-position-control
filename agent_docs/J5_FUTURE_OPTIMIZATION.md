# J5 Future Optimization

## 当前已完成方案

当前双 H30 基线采用：

```text
0x51 device sample timestamp
    + per-device clock offset/skew correction
    + software time alignment
```

两只 IMU 使用固定 `/dev/serial/by-id` 身份、独立 ROS2 数据流和只读串口采集。`0x51` 用于设备采样时间基准，host monotonic time 用于离线或实时 ClockSession 建模；当前不直接把两路原始设备时间相减作为同步结果。

实测软件同步误差约为亚毫秒至 1 ms 级。在当前研究阶段，这一精度足以继续双 IMU 状态估计、相对姿态和抗振算法的开发，但不应视为最终高精度防抖同步指标。

## 未来 Hardware SYNC

Hardware SYNC 是后续高精度防抖阶段的重要升级方向。目标是在物理采样源头同步 Base IMU 与 Platform IMU，减少独立时钟、采样相位和 USB/Linux 到达延迟对同步精度的影响。

后续需要审查并实测：

- H30/YIS106 SYNC 输入和输出功能；
- 主从或触发模式，以及触发源选择；
- SYNC 电平、电气兼容性、脉宽、边沿和接地要求；
- 200 Hz 下两只 H30 是否能够稳定同步采样；
- SYNC 模式下 `0x51` Sample Timestamp 和 `0x52` DataReady Timestamp 的物理语义；
- 实际接线、线缆、隔离和 NUC 侧接口资源。

Hardware SYNC 启用后，需要重新验证：

1. 两路实际采样边沿和 `0x51` 增量；
2. `0x52-0x51` 延迟及其抖动；
3. TID、checksum、丢帧和触发异常；
4. 跨事件、长时间运行的同步误差；
5. 双 IMU 抗振实验中的相位一致性和最终防抖性能。

## 其他未来优化

- 最终安装外参和 Base/Platform 坐标变换；
- 杆臂补偿及旋转引起的平动加速度补偿；
- 温漂模型、陀螺 bias 和加速度 bias 在线估计；
- 实时 `ClockSession`，包括 offset、skew、质量指标和重连处理；
- 基于双 IMU 的 ESKF、相对姿态和状态估计；
- 在真实安装和硬件同步条件下重新评估实时性、鲁棒性和抗振性能。

上述项目暂不改变当前 J5 软件同步基线，待硬件 SYNC 能力和最终安装条件明确后分阶段实施。

## 当前阶段边界（非当前开发任务）

本文件是未来优化/备忘，不改变当前冻结状态：J5-A、J5-B1、J5-B3、J5-C1、J5-C2-SW、J5-C2-RT-SW 均为 **PASS / FROZEN**；J5-B2 = **DEFERRED**，J5-B4 = **PENDING**。J4 控制算法与安全策略保持冻结，UART transport 已应用 batch-atomicity hotfix。

完整 6PUS 机构尚未装配，两个 IMU 仍固定在同一块测试板。最终外参、lever arm、实机 FK、Dual-IMU/FK 融合和 ESKF 只有在机构装配并通过 `q_actual → Fixed-Lag → FK → 实际 T_BP` 验证后才能启动；本备忘录不表示这些工作已经开始。
