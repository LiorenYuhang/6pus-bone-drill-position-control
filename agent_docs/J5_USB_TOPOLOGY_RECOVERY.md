# J5 USB 拓扑恢复报告：Base H30 时间戳输出

日期：2026-08-21
阶段：J5（不进入 J6）
结论：**PASS；J5 冻结基线保持有效**

## 根因

USB 枚举和串口 by-id 路径没有变化。Base H30 设备端 Class `0x04` 输出掩码被读出为 `0x00D8`，Bit12=0，因此采样时间戳 `0x51` 和 DataReady 时间戳 `0x52` 均未输出。该问题是 Base H30 的设备端输出配置回退，不是 ROS2、串口路径或 USB 拓扑解析问题。

Base 查询：

```text
TX 59 53 04 00 00 04 0c
RX 59 53 04 10 00 d8 00 ec 04
Class 0x04 payload = d8 00 = 0x00D8
```

## 恢复操作

仅对 Base H30 执行设备端 Class `0x04` 配置恢复，未修改 ROS2 源码、ROS2 配置、USB 路径或任何 J5 冻结文档。

1. RAM 测试写入目标掩码 `0x80D8`（Bit12 开启 `0x51/0x52`）：

   ```text
   TX 59 53 04 11 00 d8 80 6d 88
   RX 59 53 04 09 00 00 0d 2b    # 成功
   ```

2. RAM 回读为 `0x80D8`，随后运行 `h30_dual.launch.py` 验证通过。
3. 验证通过后写入 Flash：

   ```text
   TX 59 53 04 12 00 d8 80 6e 8c
   RX 59 53 04 0a 00 00 0e 2e    # 成功
   ```

4. Flash 写入后再次查询，仍回读 `0x80D8`。

## 验证结果

### 双 H30 原始流

使用既有命令：

```text
source /opt/ros/jazzy/setup.bash
source /home/lyapunov/bone_drill/up/sensor/IMU/install/setup.bash
ros2 launch h30_imu h30_dual.launch.py \
  base_csv_path:=/tmp/j5_recovery_dual2_base.csv \
  platform_csv_path:=/tmp/j5_recovery_dual2_platform.csv
```

Flash 固化后的采样窗口结果：

| 项目 | Base | Platform |
|---|---:|---:|
| 解码帧数 | 4,168 | 4,169 |
| `0x51` 覆盖率 | 100% | 100% |
| `0x52` 覆盖率 | 100% | 100% |
| 正常间隔 | 4,166 / 4,166 | 4,167 / 4,167 |
| 时间戳步长 | 5,000 us | 5,000 us |
| 运行期 checksum/malformed/unknown | 0 新增 | 0 新增 |

每次启动的首个间隔可能包含打开串口前残留缓存中的旧帧；从有效当前流开始，时间戳步长稳定为 5,000 us。两路节点均稳定约 200 Hz。节点日志中观察到的单次 checksum/TID 初始计数和前置 TID 跳变均发生在串口缓存清理窗口，此后计数不再增长；有效当前流没有异常间隔或持续丢帧。

### 相对姿态与自动零偏

使用既有命令：

```text
ros2 launch h30_imu h30_relative.launch.py
```

日志确认：

```text
Auto gyro bias init COMPLETE (1200 samples, 6.00 s, 5% trimmed mean)
```

诊断话题 `/pus6/imu_relative/diagnostics` 实测：

```text
message: propagating
processed_frames: 5576
processed_rate_hz: 200.003273
base_received: 6778
platform_received: 6778
interpolation_misses: 0
dt_resets: 0
invalid_base: 0
invalid_platform: 0
last_dt_us: 4999.926869
bias_init_state: COMPLETE
bias_samples: 1200/1200
```

相对四元数、相对角速度和诊断输出均正常发布；`processed_frames` 已不再为零。

## J5 范围与冻结状态

- 未修改任何 ROS2 源代码或配置文件。
- 未修改 `J5_PRE_ASSEMBLY_BASELINE`、`J5_FROZEN_MANIFEST` 等冻结文档。
- 未修改 USB by-id 路径。
- 未向电机 RS485 发送命令，未产生电机运动或意外写入。
- Timed IMU -> 相对观测链已恢复；J5 冻结文档中既有 TimedJointFeedback / FixedLag / FK 软件基线未被改动。
- 本次恢复不创建新 J 阶段；J5 Frozen Baseline 继续有效。
