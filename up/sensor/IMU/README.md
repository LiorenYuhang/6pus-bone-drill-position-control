# WHEELTEC H30 IMU — ROS 2 Jazzy

这是一个不依赖第三方串口库的 ROS 2 C++ 驱动，解析厂家 Yesense 标准二进制协议（帧头 `59 53`）。默认用于 H30 Type-C/USB 串口版。

## 编译

```bash
cd /home/lyapunov/bone_drill/up/sensor/IMU
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 连接和运行

接好 IMU 后先确认设备名：

```bash
ls -l /dev/wheeltec_IMU /dev/ttyCH343USB* /dev/ttyUSB* 2>/dev/null
```

已在这台 NUC 上确认 H30 的稳定设备路径是 `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32028000-if00`，默认波特率为 `460800`：

```bash
ros2 launch h30_imu h30.launch.py
```

若实际设备名不同：

```bash
ros2 launch h30_imu h30.launch.py port:=/dev/ttyUSB0
```

检查输出：

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

发布的话题：`/imu/data` (`sensor_msgs/Imu`)、`/imu/temperature`、`/imu/euler`（RPY，单位 rad）。角速度已由厂家协议的 deg/s 转成 rad/s；加速度原始单位就是 m/s²。H30 使用 FLU 载体系和 ENU 地理系，与 ROS REP-103 一致，不额外换轴。四元数按厂家 `q0,q1,q2,q3 = w,x,y,z` 映射并归一化。

参数在 `src/h30_imu/config/h30.yaml`。如果提示 Permission denied，请将当前用户加入串口组后重新登录：`sudo usermod -aG dialout $USER`。

注意：必须在 H30 上位机中启用加速度、角速度和四元数输出，`sensor_msgs/Imu` 才会包含相应字段。驱动遇到拔插会每秒自动重连。

## CSV 记录与实时曲线

Python 可视化节点订阅现有 C++ 驱动发布的 ROS 2 话题。每条 IMU 消息立即写入按日期时间命名的 CSV，桌面曲线每 1 秒刷新。启动方式请看 [fastreadme.md](fastreadme.md)。

```bash
./scripts/start_visualization.sh
```

CSV 默认保存在 `data/`。
