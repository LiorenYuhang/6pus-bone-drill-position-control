#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
WORKSPACE=/home/lyapunov/bone_drill/up/sensor/IMU
if [[ ! -x "$WORKSPACE/.venv/bin/python" ]]; then
  echo "Python 虚拟环境不存在，请先执行：" >&2
  echo "  python3 -m venv --system-site-packages $WORKSPACE/.venv" >&2
  echo "  $WORKSPACE/.venv/bin/python -m pip install -r $WORKSPACE/requirements.txt" >&2
  exit 1
fi
source "$WORKSPACE/.venv/bin/activate"
source /home/lyapunov/bone_drill/up/sensor/IMU/install/setup.bash
set -u
echo "正在启动 IMU 实时曲线；CSV 文件保存在 $WORKSPACE/data"
exec ros2 launch h30_imu visualize.launch.py
