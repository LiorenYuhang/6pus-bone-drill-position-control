# J6 Phase 3 首次运动观测记录器准备说明

## 1. 文档状态与边界

| 项目 | 状态 |
| --- | --- |
| 目的 | 为总装 6PUS 首次运动建立同步证据记录格式 |
| 当前工作 | 离线准备、字段定义、人工记录模板 |
| 是否执行硬件 | 否 |
| 是否执行运动 | 否 |
| 是否实现闭环控制 | 否 |
| 是否修改控制或运动代码 | 否 |
| 是否修改 `motor_driver_node.py` / `pus6_control` | 否 |
| 是否修改 J5 frozen 文件或接口 | 否 |
| 是否修改 `geometry.yaml` 或 offset | 否 |

本记录器是被动观测和证据归档准备，不向电机、IMU 或机构发送命令，不发布控制目标，不启用 IK、Cartesian motion、HOME、PARK 或任何闭环控制。实际首次运动仍由已批准的 Phase 3 运动工具和现场安全门控负责。

## 2. 观测目标

在一次明确编号的 Phase 3 试验会话中，以共同时间轴归档以下数据：

1. 六个电机的 target、actual、raw steps、current、fault、temperature 和反馈时间戳。
2. 安装在基座侧和动平台侧的两只 H30 IMU 的原始时间字段、姿态四元数和可用的角速度/加速度。
3. 基于两只 IMU 的离线相对平台/基座姿态观测；该字段只用于记录和分析，不反馈控制。
4. 可选相机快照的主机单调时间戳、文件路径或帧编号。
5. 操作员对平台运动、球铰、干涉和异响的人工观察。

## 3. 数据来源与冻结接口边界

### 3.1 六轴电机反馈

六轴反馈沿用现有 `TimedJointFeedback` 语义，不复制或改写 J5 冻结消息定义。每个采样周期应保留：

| 字段 | 含义 |
| --- | --- |
| `snapshot_start_monotonic_ns` | 六轴扫描开始的主机 `CLOCK_MONOTONIC` 时间 |
| `snapshot_end_monotonic_ns` | 六轴扫描结束的主机 `CLOCK_MONOTONIC` 时间 |
| `per_axis_rx_done_monotonic_ns[6]` | ID1-ID6 各反馈帧接收完成时间；不是电机内部采样时间 |
| `raw_steps[6]` | ID1-ID6 原始整步位置 |
| `positions_meters[6]` | 已转换的 SI 位置 |
| `currents_ma[6]` | 六轴电流 |
| `temperatures_c[6]` | 六轴温度 |
| `fault_codes[6]` | 六轴故障码 |
| `uart_cycle_sequence` | 共享 UART 周期序号 |

记录器必须保留原始整组扫描时间和逐轴时间，不能只保存一个“平均时间”而丢失串行轮询跨度。

### 3.2 双 H30 IMU

离线记录按 J6 Phase 8 的双流约定准备：

| 角色 | 预期话题 | 预期 frame ID |
| --- | --- | --- |
| 基座 IMU | `/pus6/imu_base/timed` | `pus6_base_imu_link` |
| 平台 IMU | `/pus6/imu_platform/timed` | `pus6_platform_imu_link` |

每个 IMU 样本应保留以下时间和姿态字段：

- `tid`；
- `sample_timestamp_us` 及 `has_sample_timestamp`；
- `data_ready_timestamp_us` 及 `has_data_ready_timestamp`；
- `host_monotonic_arrival_time_ns`；
- ROS header 时间和 `frame_id`（如消息提供）；
- `quaternion_xyzw`，以及可用的 `roll/pitch/yaw`；
- `angular_velocity_rad_s[3]`、`linear_acceleration_m_s2[3]`（如消息提供）；
- 话题、设备角色、采样频率和丢帧/校验错误统计。

原始 H30 四元数、采样时间和主机到达时间必须保留；离线相对姿态不得覆盖原始 IMU 样本。

### 3.3 相对平台 / 基座姿态

相对姿态作为离线观测字段记录，推荐明确写出约定：

```text
q_base_to_platform_xyzw
R_base_to_platform
```

该命名表示“基座坐标系到平台坐标系”的旋转，具体乘法方向和四元数顺序必须在会话元数据中声明。每个相对姿态样本还应包含：

- 使用的 base/platform IMU 样本序号或时间戳；
- 采用的外参/杆臂记录 ID；
- 时间配对偏差 `pairing_error_ns`；
- 是否为原始节点输出、离线重放结果或人工计算；
- 有效性、缺样、插值/外推状态和失败原因。

相对姿态只支持“观测到的姿态关系”结论，不支持闭环控制、IK 修正、零位补偿或几何参数修改。J5-B3 的共刚体验证结果不能直接当作最终 Base/Platform 安装精度；最终装配外参和杆臂仍需单独确认。

### 3.4 可选相机证据

相机不是必需通道。启用时，每帧至少记录：

- `camera_snapshot_timestamp_monotonic_ns`；
- 相机编号、文件路径或帧 ID；
- 相机时间戳来源（主机单调时钟、设备时钟或仅墙钟）；
- 与最近电机/IMU采样的时间差；
- 图像是否完整、是否遮挡、是否可用于观察平台和球铰。

只有主机单调时间戳可直接参与同步排序；仅有墙钟时间的相机帧必须标记为低可信可选证据，不得伪装成严格同步数据。

## 4. 统一时间轴与配对规则

### 4.1 主时间基准

会话统一使用主机 `CLOCK_MONOTONIC` 纳秒时间作为排序基准：

```text
t_observation_ns = host_monotonic_arrival_time_ns
```

不得将 ROS header 时间、IMU 设备微秒计数和相机墙钟时间直接混加。所有非主机时钟字段必须原样保存，并注明时钟域。

### 4.2 电机周期时间

每个六轴反馈周期保留：

```text
t_motor_start_ns
t_motor_end_ns
t_motor_axis_done_ns[1..6]
```

离线配对时可以额外计算周期中点或最近逐轴时间，但不得删除原始时间。若某个 IMU 样本落在六轴扫描跨度内，应记录它与整组起止时间及每轴完成时间的关系。

### 4.3 IMU 配对

推荐优先使用 IMU 样本的主机到达时间参与离线配对，同时保留设备 `sample_timestamp_us` 和 `data_ready_timestamp_us`。配对输出必须记录：

```text
imu_pairing_method
base_sample_index / platform_sample_index
base_host_time_ns / platform_host_time_ns
pairing_error_ns
interpolation_used = false | true
extrapolation_used = false | true
```

首次运动证据禁止无记录外推。若两路 IMU 没有满足时间窗口的样本，保留缺样状态，不生成伪造的相对姿态。

### 4.4 建议的证据时间窗

一次会话至少分成四类事件段：

1. `baseline_static`：运动前静止基线；
2. `forward_perturbation`：一次批准的首动扰动；
3. `observation_hold`：前进后保持观察，不自动回程；
4. `return_authorized`：仅在现场明确授权后发生的回程。

每段都使用同一 `session_id` 和递增 `sample_index`。记录器不得自行发起回程或重试。

## 5. 离线记录流程

### 5.1 会话开始前

- 创建唯一 `session_id`、操作者、审查人和会话日期；
- 记录软件仓库版本、J6 运动报告路径、观测记录器版本和配置哈希；
- 确认 Phase 3 运动授权由外部门控提供，记录器本身不提供授权；
- 核对两路 IMU 的角色、frame ID、话题和时钟字段；
- 核对电机反馈使用 J5 冻结 `TimedJointFeedback` 字段；
- 确认相机若启用则能输出主机单调时间戳；
- 预先创建只追加、不覆盖的原始 JSONL/CSV 文件和会话 manifest。

### 5.2 采集期间

- 先记录静态基线，再标记外部运动工具的前进、保持和回程事件；
- 每个电机反馈周期记录六轴完整数组，不只记录被选轴；
- 两路 IMU 原始样本分别保存，不能只保存相对姿态；
- 相对姿态、配对误差和相机时间戳写入同一会话索引或可追溯引用；
- 发生丢帧、时间回退、校验错误、frame ID 变化或相机遮挡时记录事件，不删除异常样本；
- 记录操作员的即时观察，但不把人工描述改写成软件 PASS。

### 5.3 会话结束后

- 关闭追加流并写入文件校验和；
- 统计电机、两路 IMU 和相机的样本数量、时间范围、重复/回退/缺口；
- 计算但不覆盖原始数据：电机扫描跨度、IMU 配对偏差、相机最近邻时间差；
- 检查 forward、hold、return 事件是否与运动报告的时间段相符；
- 生成 `PASS` / `HOLD` / `INVALID` 的证据完整性判断；
- 将异常原因和未完成字段保留在报告中，不通过补值或重采样伪造完整性。

## 6. 操作员人工观察字段

每个事件段应支持以下人工字段，取值不得由软件自动推断：

| 字段 | 建议取值 |
| --- | --- |
| `platform_motion` | `unrecorded` / `expected` / `unexpected` |
| `ball_joint_condition` | `unrecorded` / `clear` / `abnormal` |
| `interference` | `unrecorded` / `none` / `present` |
| `abnormal_sound` | `unrecorded` / `none` / `present` |
| `observer_notes` | 自由文本，记录位置、时间和见证人 |

只要任一字段为 `unexpected`、`abnormal` 或 `present`，该事件段应保持 `HOLD`/`STOP`，不得由记录器触发回程、重试或控制恢复。

## 7. 离线报告门控

### 7.1 证据完整性条件

| 条件 | 要求 |
| --- | --- |
| 六轴反馈完整 | 每条电机观测含 ID1-ID6 的 target/actual/current/fault/temperature 和时间戳 |
| 电机时间可追溯 | 保留整组起止及逐轴接收完成时间 |
| 双 IMU 完整 | base/platform 角色、frame、tid、设备时间和主机到达时间可追溯 |
| 相对姿态可解释 | 旋转方向、四元数顺序、外参来源和配对误差已声明 |
| 相机证据可分级 | 可选；时间域和文件路径明确，不把墙钟当严格同步 |
| 无控制副作用 | 记录器无目标发布、无寄存器写入、无 HOME/PARK/IK/Cartesian 调用 |
| 原始数据可恢复 | 输入 JSONL/CSV 不被覆盖，manifest 和校验和存在 |

### 7.2 结论分类

- `PASS`：只表示观测证据完整、时间顺序有效、人工字段已填写；不表示机构运动安全或控制通过。
- `HOLD`：存在待补样本、未填写人工观察、配对窗口不足或外参来源未确认。
- `INVALID`：字段结构错误、时间回退/重复、frame/topic 不一致或原始证据不可追溯。

观测记录器的 `PASS` 不能替代 Phase 3 机械 PASS，也不能改变当前 J6 的运动门控。

## 8. 建议文件组织

```text
bone_ws/log/j6_phase3_observation_<session_id>/
  session_manifest.json
  motor_feedback.jsonl
  imu_base.jsonl
  imu_platform.jsonl
  relative_attitude.jsonl
  camera_index.jsonl              # 可选
  operator_observations.jsonl
  observation_report.json
```

原始流按来源分文件，`observation_report.json` 只保存索引、统计、配对结果和证据门控，不重写原始样本。所有文件应使用不可覆盖的会话目录，并在 manifest 中记录创建时间、工具版本和 SHA-256。

## 9. 当前 J6 处置

Phase 3 首次运动观测记录器目前只完成离线字段和流程准备。它不会解除 J6 Phase 2 的 `HOLD`，不会代替机械 bottom 确认，不会启动运动，也不会修改 J5 冻结链路。待 Phase 2、Phase 3 运动工具和现场安全授权分别满足后，才可把本模板作为被动证据采集输入。

## 10. 关联文件

- [J6 Phase 3 首次运动安全审查](J6_PHASE3_FIRST_MOTION_SAFETY_REVIEW_CN.md)
- [J6 Phase 2 人工确认记录](J6_PHASE2_MANUAL_VERIFICATION_RECORD_CN.md)
- [J6 统一测试报告 Schema](../stages/J6/J6_TEST_REPORT_SCHEMA.md)
- [J6 Phase 8 IMU 准备脚本（只读/离线验证框架）](../../bone_ws/scripts/run_j6_phase8_imu_preparation.py)
- `bone_ws/src/pus6_msgs/msg/TimedJointFeedback.msg`
