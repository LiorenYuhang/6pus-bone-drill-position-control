# H30 IMU 实时曲线与 CSV 快速运行说明

当前工作目录：

```bash
cd /home/lyapunov/bone_drill/up/sensor/IMU
```

## 1. 首次运行

依次执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
```

如果 `.venv` 已存在，无需重复创建和安装。

## 2. 启动 IMU 数据发布

如果已经安装并启动了后台服务：

```bash
systemctl --user status h30-imu.service
```

状态显示 `active (running)` 时，可以直接进行下一步。

如果没有运行后台服务，可在一个终端执行：

```bash
cd /home/lyapunov/bone_drill/up/sensor/IMU
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch h30_imu h30.launch.py
```

## 3. 启动实时曲线

当 `/imu/data` 已经在发布时，执行：

```bash
cd /home/lyapunov/bone_drill/up/sensor/IMU
./scripts/start_visualization.sh
```

程序会直接打开桌面曲线窗口，每 1 秒刷新一次。同时，每一条 `/imu/data` 消息都会立即写入 CSV。

## 4. 一次启动驱动和实时曲线

如果没有使用后台服务，也可以在一个终端同时启动：

```bash
cd /home/lyapunov/bone_drill/up/sensor/IMU
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
ros2 launch h30_imu h30_visual.launch.py
```

## 5. 检查数据

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

正常情况下，`ros2 topic hz` 会持续显示更新频率。

实时窗口和 CSV 中包含：

- 三轴线加速度和加速度模长
- 三轴角速度和角速度模长
- 姿态四元数 x/y/z/w
- Roll、Pitch、Yaw（rad 和 deg）
- IMU 温度

CSV 文件默认保存在：

```text
/home/lyapunov/bone_drill/up/sensor/IMU/data/
```

文件名使用启动日期时间，例如 `imu_20260727_143025.csv`。每次启动生成一个新文件，不会覆盖之前的数据。CSV 每条 IMU 消息记录一行，每 1 秒刷新到磁盘。

## 6. 停止运行

前台运行时按：

```text
Ctrl+C
```

停止后台 IMU 服务：

```bash
systemctl --user stop h30-imu.service
```

重新启动后台服务：

```bash
systemctl --user start h30-imu.service
```

## 最短启动方式

已经完成首次安装并启用后台服务后，每次只需要：

```bash
cd /home/lyapunov/bone_drill/up/sensor/IMU
./scripts/start_visualization.sh
```

随后会自动出现实时曲线窗口。
