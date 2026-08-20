# J5 装配前阶段软件环境与硬件接口定义 (J5 Software Environment & Hardware Specification)

> **文档标识**: `ENV-6PUS-J5-20260820`  
> **适用版本**: `v1.0-j5-pre-assembly-frozen`  
> **适用阶段**: 6PUS J5 装配前基线环境复现与 J6 现场调试环境基准

---

## 1. 宿主机与操作系统环境

* **操作系统**: Ubuntu 24.04 LTS (Noble Numbat, 64-bit x86_64)
* **Linux 内核**: Linux 6.8.0-generic
* **系统依赖与库**:
  * `build-essential` (GCC 13.2.0, G++ 13.2.0, Make, CMake 3.28.3)
  * `libserial-dev` / 标准 Linux POSIX Termios 接口
  * `git`, `pdftotext` (poppler-utils)

---

## 2. ROS 2 中间件与通信环境

* **ROS 2 发行版**: **ROS 2 Jazzy Jalisco** (安装路径 `/opt/ros/jazzy`)
* **DDS 中间件**: CycloneDDS / eProsima FastDDS（默认 FastDDS）
* **ROS 2 环境 Source 脚本**:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source /home/lyapunov/bone_drill/bone_ws/install/setup.bash
  ```

---

## 3. Python 运行时与核心依赖

* **Python 解释器**: Python 3.12.3 (`/usr/bin/python3`)
* **核心 Python 依赖包**:
  * `pytest`: 7.4.4+ (单元测试与回归框架)
  * `numpy`: 1.26.4+ (矩阵与向量数值计算)
  * `scipy`: 1.13.0+ (优化器与李群/旋转库)
  * `pyserial`: 3.5+ (底层 RS485 串口通信)
  * `pyyaml`: 6.0.1+ (`6pus_geometry.yaml` 权威参数加载)
  * `rclpy`: ROS 2 Jazzy 官方 Python 客户端绑定库

---

## 4. 工作空间与功能包架构 (Workspace Structure)

工作空间根目录位于：`/home/lyapunov/bone_drill/bone_ws`

```
bone_ws/
├── src/
│   ├── pus6_msgs/                     # [ROS 2 C++ 接口包]
│   │   ├── msg/
│   │   │   ├── TimedJointFeedback.msg # 六轴微秒/纳秒接收完成时间戳遥测帧
│   │   │   ├── TimedJointCommand.msg  # 六轴发送开始/完成纳秒时间戳控制帧
│   │   │   └── FkStateStamped.msg     # 6PUS 数值正解末端位姿、残差与迭代状态
│   │   └── CMakeLists.txt
│   │
│   ├── pus6_kinematics/               # [运动学核心包 (Python)]
│   │   ├── config/
│   │   │   └── 6pus_geometry.yaml     # 生产环境唯一权威几何与安全参数真值源
│   │   ├── pus6_kinematics/
│   │   │   ├── geometry_config.py     # 权威 YAML 解析与类型安全配置类
│   │   │   ├── analytical_6pus.py     # 闭式解析逆运动学求解器与几何定义
│   │   │   ├── state_synchronizer.py  # FixedLagJointSynchronizer 零外推时钟对齐重构
│   │   │   └── fk_node.py             # Pus6FkNode (独立 SingleThreadedExecutor 实时解算)
│   │   └── test/                      # 35 项单元与实时消融测试
│   │
│   ├── pus6_motor_driver/             # [电机驱动与协议包 (Python)]
│   │   ├── config/
│   │   │   └── motors.yaml            # 电机串口与通信运行时参数
│   │   ├── pus6_motor_driver/
│   │   │   ├── protocol.py            # 0x29/0x30/0x32 串口协议编解码与 CRC 校验
│   │   │   ├── six_motor_controller.py# 六轴批次通信、重试、Batch 锁与伴随解析
│   │   │   ├── safety.py              # 5 级安全硬门禁、Slew 自愈与 Maintenance 隔离
│   │   │   └── motor_driver_node.py   # Pus6MotorDriverNode (Latest-Value-Wins 下发)
│   │   └── test/                      # 60 项驱动、协议、0 写入审计与回归测试
│   │
│   ├── pus6_control/                  # [控制与轨迹整形包 (Python)]
│   │   ├── pus6_control/
│   │   │   ├── shaper.py              # StatefulJerkLimitedShaper (三阶加加速度连续整形)
│   │   │   ├── joint_trajectory_controller.py # 关节控制器与 Delivery-Uncertain 自愈
│   │   │   └── control_node.py        # Pus6ControlNode (50Hz 笛卡尔/关节控制节点)
│   │   └── test/                      # 67 项控制器、Shaper、Recovery 测试
│   │
│   ├── pus6_bringup/                  # [启动与系统配置包]
│   │   ├── config/
│   │   │   └── system_params.yaml     # 节点命名空间与通用参数
│   │   └── launch/                    # 系统级 launch 文件
│   │
│   └── h30_imu/                       # [Yesense H30 IMU 驱动与相对姿态解算 (C++)]
│       ├── msg/
│       │   ├── TimedImu.msg           # 0x51/0x52 采样微秒时间戳与惯性数据
│       │   └── RelativeAttitude.msg   # 双 IMU 李群 SO(3) 相对四元数与相对角速度
│       ├── src/
│       │   ├── h30_protocol.cpp       # Yesense 标准 TLV 帧解析器 (含 0x51/0x52)
│       │   ├── h30_imu_node.cpp       # 200Hz 串口读取与 TimedImu 高频发布节点
│       │   └── h30_relative_attitude_node.cpp # 相对姿态解算与 6.0s 自动零偏估计节点
│       └── launch/
│           ├── h30_dual.launch.py     # 双 IMU 独立驱动启动文件
│           └── h30_relative.launch.py # 双 IMU 驱动 + 相对姿态解算总启动文件
│
└── scripts/                           # [可执行脚本与验收工具链]
    ├── run_j2a_readonly_verification.py # 60s 0 写入 READ_ONLY 硬件审计工具
    ├── run_j3_startup_homing.py        # PARK ↔ HOME 一键顶升/收拢 S 曲线执行器
    ├── run_j3_cartesian_motion.py      # J3 点到点单自由度/复合位姿验证工具
    ├── run_j4_f1b_real_cartesian_motion.py # J4 连续笛卡尔动态轨迹控制工具
    └── run_j5_c2_hw_acceptance.py      # J5-C2-HW 离线数据回放与 Synthetic 验收工具
```

---

## 5. 硬件接口与串口映射规范 (Hardware Interface Specification)

### 5.1 稳定 By-ID 设备映射表
为防止 Linux 重启或 USB 热插拔导致 `/dev/ttyUSB*` 与 `/dev/ttyACM*` 编号跳变，所有生产与测试脚本一律强制采用 `/dev/serial/by-id/` 硬件持久化符号链接：

| 硬件目标 | 物理总线与波特率 | 权威 By-ID 串口路径 | 默认备用路径 |
| :--- | :---: | :--- | :---: |
| **6 轴推杆电缸** | RS485 半双工 (921600 8N1) | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` | `/dev/ttyUSB0` |
| **Base H30 IMU** | USB-CDC (460800 8N1) | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00` | `/dev/ttyACM0` |
| **Platform H30 IMU**| USB-CDC (460800 8N1) | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B029000-if00` | `/dev/ttyACM1` |

### 5.2 核心电气与总线参数
* **推杆电缸工作电压**: DC 24V（峰值电流建议 $\ge 10\text{ A}$ 稳压电源）；
* **急停门禁 (E-Stop)**: 硬件紧急物理断电开关串联于 24V 供电母线；
* **RS485 终端电阻**: 建议在总线最远端电缸处配置 $120\ \Omega$ 匹配电阻；
* **H30 采样配置**: Class 0x04 输出掩码为 `0x80D8`（已固化至 Flash，强制开启 0x51 硬件采样微秒时间戳输出）。

---

## 6. 环境构建与一键验证命令

```bash
# 1. 编译整个 ROS 2 工作空间
cd /home/lyapunov/bone_drill/bone_ws
colcon build --symlink-install

# 2. 载入环境
source /opt/ros/jazzy/setup.bash
source /home/lyapunov/bone_drill/bone_ws/install/setup.bash

# 3. 运行全量核心单元测试 (162 项)
pytest /home/lyapunov/bone_drill/bone_ws/src -v
```
