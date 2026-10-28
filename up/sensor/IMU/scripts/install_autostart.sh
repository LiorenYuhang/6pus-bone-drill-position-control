#!/usr/bin/env bash
set -e
unit_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
mkdir -p "$unit_dir"
cp /home/lyapunov/6pus_bone_drill_position_control/up/sensor/IMU/systemd/h30-imu.service "$unit_dir/h30-imu.service"
systemctl --user daemon-reload
systemctl --user enable --now h30-imu.service
echo "H30 已设为登录后自动连续读取。"
