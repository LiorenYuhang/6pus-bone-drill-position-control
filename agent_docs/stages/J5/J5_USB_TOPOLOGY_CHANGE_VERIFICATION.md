# J5 USB 拓扑变更验证报告

**日期：** 2026-08-21
**范围：** 仅验证 J5 PRE-ASSEMBLY FROZEN 基线。未修改代码或架构，未进入 J6。

## 验证结论

**总体结论：FAIL / 在当前观测到的硬件状态下，基线不能发布。**

本次 USB 拓扑变更没有改变任何持久化设备身份，也不需要更新串口路径。电机 RS485 转接器和两只 H30 设备均已被检测到。电机通信和 READ_ONLY 安全行为通过，工作空间软件/Mock 回归也通过。但是，现场 Base H30 数据流不包含 0x51/0x52 硬件时间戳，因此双路时间戳契约失败。由此，`h30_relative.launch.py` 无法处理配对帧、不发布相对姿态，并且自动陀螺仪零偏初始化最终进入 `FALLBACK_TIMEOUT`。

未修改任何冻结文件；本报告是唯一新增文件。

## 1. USB / 串口映射

执行的只读命令：

```text
lsusb
ls -la /dev/serial/by-id/
udevadm info -q property -n <by-id-path>
```

相关 `lsusb` 设备：

```text
1a86:55d4 QinHeng Electronics USB Single Serial   （2 个设备：两只 H30 IMU）
1a86:7523 QinHeng Electronics CH340 serial converter（电机 RS485 转接器）
05e3:0610 / 05e3:0626 Genesys Logic hubs          （USB 集线器）
```

持久化链接及现场目标设备：

| 设备 | By-ID 路径 | 实际目标 | USB 位置 | 结果 |
|---|---|---|---|---|
| Base H30 | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00` | `/dev/ttyACM0` | `3-7.4.4` | PASS，已检测 |
| Platform H30 | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00` | `/dev/ttyACM1` | `3-7.4.3` | PASS，已检测 |
| 电机 RS485 | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` | `/dev/ttyUSB0` | `3-2` | PASS，已检测 |

两只 H30 均位于 USB 集线器下游路径（`3-7.4.x`）；电机转接器仍位于 NUC 原生 USB 路径（`3-2`）。三条生产 By-ID 字符串均与冻结 J5 映射一致。**不需要更新配置路径。**

但在重新接受基线之前，必须修复 Base H30 的输出配置：其数据流必须重新包含 0x51/0x52。本次验证没有下发任何修复写命令。

## 2. 双 H30 回归验证

### `h30_dual.launch.py`

使用两条 By-ID 串口路径和 CSV 采集运行现有 J5 流程约 17.6 秒。两个节点均成功打开串口，并以约 200 Hz 发布数据。每个设备连续采集到 3,514 个已解码帧。

| 检查项 | Base | Platform | 结果 |
|---|---:|---:|---|
| 已解码帧数 | 3,514 | 3,514 | PASS |
| 具备 0x51 采样时间戳 | 0 / 3,514（0%） | 3,514 / 3,514（100%） | **FAIL** |
| 具备 0x52 数据就绪时间戳 | 0 / 3,514（0%） | 3,514 / 3,514（100%） | **FAIL** |
| 0x51 时间戳步长 | N/A | 5,000 us，3,513 个间隔 | Platform PASS；Base FAIL |
| 发布频率 | 约 200 Hz | 约 200 Hz | PASS |
| 连续运行中的 TID/解析错误 | 未报告 | 未报告 | PASS |
| IMU 数据 / 姿态字段 | 存在 | 存在 | PASS |

Base 日志反复报告 `sample_us=- ready_us=-`；Platform 报告有效时间戳。这是硬件输出配置回归，不是 Linux 串口路径缺失。

### `h30_relative.launch.py`

使用 ROS 话题观测运行现有 J5 流程。两只 IMU 串口均成功打开。诊断消息如下：

```text
processed_frames:       0
base_received:          0
platform_received:      7402
invalid_base:           7380
interpolation_misses:   0
dt_resets:              0
bias_init_state:        FALLBACK_TIMEOUT
bias_samples:           0/1200
message:                waiting_or_reset
```

因此：

- 两个原始 IMU 读取节点都在发布，但由于缺少所需硬件时间戳，Base 的定时样本被拒绝；
- 没有产生相对姿态输出（`processed_frames=0`）；
- 自动陀螺仪零偏初始化未完成，并在超时后回退；
- 相对姿态验证项为 **FAIL**。

独立双路连续采集中的 TID 丢帧、校验和错误及解析错误均为零。重新启动 relative 流程时，由于两次采集之间没有进程读取设备，日志显示了累计 TID 间隔；该启动间隔已单独记录，不作为连续运行期间的异常丢帧。

## 3. 电机通信回归验证

使用冻结的电机 By-ID 路径、921600 baud，运行现有 J2-A READ_ONLY 审计 10.02 秒。

```text
verdict:                 J2-A COMPLETE / PASS
snapshots:               100
joint_states:            99 (9.88 Hz)
fault codes:             0x00 on all six axes
hardware mode:           read_only
startup writes:          0
runtime writes:          0
shutdown writes:         0
total hardware writes:   0
```

遥测通信耗时均值为 18.26 ms，P95 为 19.83 ms，ID1 到 ID6 的扫描跨度均值为 15.26 ms。各轴位置均保持在审计包络内。**电机通信 PASS；READ_ONLY 行为未改变；硬件写入计数 = 0。**

## 4. J5 全链路冒烟 / 软件回归

在加载 ROS 2 Jazzy 和已构建工作空间后重新运行源码测试：

```text
pytest /home/lyapunov/bone_drill/bone_ws/src -q
162 passed in 38.58s
```

该测试覆盖 Mock 电机安全路径、Latest-Value-Wins / 陈旧目标行为、Timed 遥测契约、Fixed-Lag 零外推同步，以及 FK 观测节点测试。**软件/Mock 管线 PASS，hardware_write_count = 0。**

由于 Base H30 时间戳输入无效且 relative 链路处理帧数为零，现场 Timed IMU -> 配对观测 -> Fixed-Lag -> FK 冒烟项不能判定为 PASS。本次故障状态下，不宣称现场 FK 观测链路通过。

## 5. 所需变更

| 项目 | 所需动作 | 是否修改代码/架构 |
|---|---|---|
| 持久化 USB 路径 | 无；所有冻结 By-ID 链接均存在且正确 | 否 |
| 电机配置 | 无 | 否 |
| Base H30 0x51/0x52 | 恢复并确认 Base H30 输出配置，使两个字段重新输出，然后重新运行 `h30_dual.launch.py` 和 `h30_relative.launch.py` | 仅硬件/设备配置；未修改仓库 |
| 冻结文件 | 在 H30 回归清除前保持不变 | 否 |

本报告不进入 J6，也不下发 H30 配置写命令或任何电机运动命令。

## 6. J5 基线状态

**J5 Frozen Baseline 的结构保持不变，但当前 USB 拓扑下尚未重新验证通过。** USB 枚举和电机安全行为与冻结基线兼容；Base H30 的 0x51/0x52 故障使 J5 双 IMU、相对姿态、自动零偏以及现场观测链验收项失效。恢复 Base 时间戳输出后，必须重新执行相同的两个 launch 流程，并要求两台设备均达到 100% 时间戳覆盖、5,000 us 步长、连续运行零丢帧、相对姿态正常发布，以及 `Auto gyro bias init COMPLETE`，之后才能重新确认冻结基线有效。
