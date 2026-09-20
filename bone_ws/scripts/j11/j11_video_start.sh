#!/usr/bin/env bash
set -e
cd /home/lyapunov/bone_drill
source bone_ws/install/setup.bash
exec python3 bone_ws/src/h30_imu/scripts/j11_staged_launcher.py --execute-motion "$@"
