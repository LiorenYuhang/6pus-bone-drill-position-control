#!/usr/bin/env python3
"""User-friendly CLI for moving 6PUS platform to arbitrary Cartesian coordinates.

Supports both relative offsets (mm & deg/rad from OPERATING_HOME)
and interactive prompts with safety envelope pre-checking.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from pathlib import Path

# Path setup
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
SRC_PATH = WORKSPACE_ROOT / "src"
for pkg in ["pus6_motor_driver", "pus6_kinematics", "pus6_control"]:
    p = str(SRC_PATH / pkg)
    if p not in sys.path:
        sys.path.insert(0, p)
SCRIPTS_PATH = str(WORKSPACE_ROOT / "scripts")
if SCRIPTS_PATH not in sys.path:
    sys.path.insert(0, SCRIPTS_PATH)

from pus6_control.joint_trajectory_controller import quintic_progress
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_motor_driver.fake_bus import FakeBus
from pus6_motor_driver.safety import (
    MAX_CURRENT_MA,
    MOTOR_IDS,
    OPERATING_HOME_RAW_STEPS,
    STORAGE_PARK_RAW_STEPS,
    HardwareMode,
    validate_target_vector,
)
from pus6_motor_driver.six_motor_controller import SixMotorController

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("move_cartesian")

# Safety limits (Validated J4 software envelope)
MAX_XY_MM = 15.0         # +/- 15 mm
MAX_Z_DELTA_MM = 20.0    # +/- 20 mm relative to nominal height
MAX_ANGLE_DEG = 5.73     # +/- 0.10 rad (~5.73 deg)
MAX_ANGLE_RAD = 0.10


def validate_user_pose(
    dx_mm: float,
    dy_mm: float,
    dz_mm: float,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> bool:
    """Check if Cartesian offset is within safe envelope."""
    if abs(dx_mm) > MAX_XY_MM or abs(dy_mm) > MAX_XY_MM:
        logger.error(f"❌ 平面平移越界: X={dx_mm:.1f}mm, Y={dy_mm:.1f}mm (安全上限: +/-{MAX_XY_MM:.1f}mm)")
        return False
    if abs(dz_mm) > MAX_Z_DELTA_MM:
        logger.error(f"❌ 垂直升降越界: dZ={dz_mm:.1f}mm (安全上限: +/-{MAX_Z_DELTA_MM:.1f}mm)")
        return False
    for name, ang in [("Roll", roll_rad), ("Pitch", pitch_rad), ("Yaw", yaw_rad)]:
        if abs(ang) > MAX_ANGLE_RAD:
            logger.error(f"❌ {name} 姿态角越界: {ang:.3f} rad ({math.degrees(ang):.2f}°) (安全上限: +/-{MAX_ANGLE_RAD:.2f} rad / {MAX_ANGLE_DEG:.1f}°)")
            return False
    return True


def execute_smooth_motion(
    controller: SixMotorController,
    start_steps: list[int],
    target_steps: list[int],
    duration: float,
    control_rate_hz: float = 10.0,
) -> bool:
    """Execute smooth quintic S-curve motion."""
    dt = 1.0 / control_rate_hz
    t = 0.0
    last_cmd = None

    while t <= duration + 0.02:
        t_loop_start = time.time()
        tau = min(1.0, max(0.0, t / max(0.001, duration)))
        s = quintic_progress(tau)

        cmd_steps = [
            int(round(float(q0) + s * float(q1 - q0)))
            for q0, q1 in zip(start_steps, target_steps)
        ]

        # Dispatch 6-axis write
        if cmd_steps != last_cmd:
            controller.command_positions(cmd_steps)
            last_cmd = list(cmd_steps)

        # In active mode, check telemetry safety invariants
        if controller.hardware_mode == HardwareMode.ACTIVE:
            snapshot = controller.read_state_snapshot()
            if any(f != 0 for f in snapshot.fault_codes):
                logger.error(f"🛑 电机硬件故障码: {snapshot.fault_codes}")
                controller.pause_all()
                return False
            if any(c > MAX_CURRENT_MA for c in snapshot.currents_mA):
                logger.error(f"🛑 电机异常过流: {snapshot.currents_mA} mA > {MAX_CURRENT_MA} mA")
                controller.pause_all()
                return False

        t += dt
        elapsed = time.time() - t_loop_start
        if elapsed < dt:
            time.sleep(dt - elapsed)

    return True


def move_to_cartesian_offset(
    dx_mm: float = 0.0,
    dy_mm: float = 0.0,
    dz_mm: float = 0.0,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    roll_rad: float | None = None,
    pitch_rad: float | None = None,
    yaw_rad: float | None = None,
    peak_speed: float = 200.0,
    hold_time: float = 2.0,
    return_home: bool = True,
    real_hardware: bool = False,
    confirm_motion: bool = False,
    validated_config: bool = False,
) -> bool:
    """Compute 6PUS IK and move real/mock platform to target Cartesian pose."""
    # Unit conversions
    r_rad = roll_rad if roll_rad is not None else math.radians(roll_deg)
    p_rad = pitch_rad if pitch_rad is not None else math.radians(pitch_deg)
    y_rad = yaw_rad if yaw_rad is not None else math.radians(yaw_deg)

    logger.info("=" * 65)
    logger.info("6PUS 笛卡尔空间运动规划器")
    logger.info("=" * 65)
    logger.info(f"目标相对中位位移: X={dx_mm:+.2f}mm, Y={dy_mm:+.2f}mm, Z={dz_mm:+.2f}mm")
    logger.info(f"目标姿态转角: Roll={math.degrees(r_rad):+.2f}°, Pitch={math.degrees(p_rad):+.2f}°, Yaw={math.degrees(y_rad):+.2f}°")
    logger.info(f"运行模式: {'真实硬件 (ACTIVE)' if real_hardware else '仿真模式 (MOCK)'}")
    logger.info(f"规划峰值速度: {peak_speed:.1f} step/s ({peak_speed*0.025:.2f} mm/s)")
    logger.info(f"到达后动作: {'停留后返回 OPERATING_HOME' if return_home else '保持在目标位置'}")

    # Validate Cartesian envelope
    if not validate_user_pose(dx_mm, dy_mm, dz_mm, r_rad, p_rad, y_rad):
        return False

    # Analytical IK
    ik = Analytical6PusKinematics(
        base_radius_m=0.040,
        platform_radius_m=0.0275,
        strut_length_m=0.084,
        nominal_height_m=0.132612268,
        steps_per_meter=40000.0,
    )
    z0 = ik.nominal_height
    ref_pose = [0.0, 0.0, z0, 0.0, 0.0, 0.0]
    target_pose = [
        dx_mm * 1e-3,
        dy_mm * 1e-3,
        z0 + dz_mm * 1e-3,
        r_rad,
        p_rad,
        y_rad,
    ]

    delta_steps = ik.differential_inverse_kinematics_steps(target_pose, ref_pose)
    target_steps = [1000 + ds for ds in delta_steps]

    logger.info(f"权威 6PUS 解析 IK 计算差分量: Δq = {delta_steps} step")
    logger.info(f"六轴绝对整步目标: q_target = {target_steps} step")

    # Joint limit validation
    try:
        validate_target_vector(target_steps)
    except Exception as exc:
        logger.error(f"❌ 关节行程软限超限: {exc}")
        return False

    # Hardware connection
    mode = HardwareMode.ACTIVE if real_hardware else HardwareMode.MOCK
    if real_hardware and not (confirm_motion and validated_config):
        logger.error("❌ 真实硬件运行必须携带 --confirm-motion 与 --validated-config 安全开关！")
        return False

    if not real_hardware:
        bus = FakeBus()
        bus.positions = {mid: 1000 for mid in MOTOR_IDS}
        bus.targets = dict(bus.positions)
        controller = SixMotorController(bus=bus, hardware_mode=HardwareMode.MOCK)
    else:
        controller = SixMotorController(
            port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            baudrate=921600,
            timeout=0.05,
            hardware_mode=HardwareMode.ACTIVE,
        )

    controller.connect()

    try:
        # Read current start position
        snap = controller.read_state_snapshot()
        start_steps = list(snap.positions_steps)
        logger.info(f"六轴当前起始位置: {start_steps} step")

        # Configure motor speed register
        reg_speed = max(50, int(round(peak_speed * 1.2)))
        controller.configure_all(speed=reg_speed)

        # Plan move duration
        max_disp = max(abs(tgt - cur) for tgt, cur in zip(target_steps, start_steps))
        duration = max(1.5, (1.875 * max_disp) / peak_speed)

        logger.info(f"▶ 阶段 1: 六轴协调运动至目标位置 (规划时长: {duration:.2f}s)...")
        if not execute_smooth_motion(controller, start_steps, target_steps, duration):
            return False

        time.sleep(0.5)
        snap_peak = controller.read_state_snapshot()
        logger.info(f"✔ 成功到达目标位置: 实测={snap_peak.positions_steps} step, 电流={snap_peak.currents_mA} mA")

        if hold_time > 0:
            logger.info(f"▶ 阶段 2: 目标位置保持观察 {hold_time:.1f} 秒...")
            time.sleep(hold_time)

        if return_home:
            logger.info(f"▶ 阶段 3: 六轴协调平滑返回 OPERATING_HOME [1000]*6...")
            snap_curr = controller.read_state_snapshot()
            ret_disp = max(abs(1000 - cur) for cur in snap_curr.positions_steps)
            ret_dur = max(1.5, (1.875 * ret_disp) / peak_speed)
            if not execute_smooth_motion(controller, snap_curr.positions_steps, [1000] * 6, ret_dur):
                return False

            time.sleep(1.0)
            snap_final = controller.read_state_snapshot()
            errs = [abs(p - 1000) for p in snap_final.positions_steps]
            logger.info(f"✔ 成功返回 OPERATING_HOME: 最终={snap_final.positions_steps}, 误差={errs} step (最大 {max(errs)} step)")

        # Lock and pause
        controller.pause_all()
        logger.info("✔ 运动全部完成，六轴已安全制动锁止。")
        return True

    finally:
        controller.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="6PUS 动平台笛卡尔空间位置控制工具")
    parser.add_argument("--x", type=float, default=0.0, help="X 轴相对位移 (前+/后-, 单位: mm, 上限 +/-15)")
    parser.add_argument("--y", type=float, default=0.0, help="Y 轴相对位移 (左+/右-, 单位: mm, 上限 +/-15)")
    parser.add_argument("--z", type=float, default=0.0, help="Z 轴相对位移 (升+/降-, 单位: mm, 上限 +/-20)")
    parser.add_argument("--roll", type=float, default=0.0, help="Roll 侧倾角 (右倾+/左倾-, 单位: 度 deg, 上限 +/-5.7°)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Pitch 俯仰角 (仰+/俯-, 单位: 度 deg, 上限 +/-5.7°)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw 偏航角 (顺时针+/逆时针-, 单位: 度 deg, 上限 +/-5.7°)")
    parser.add_argument("--roll-rad", type=float, default=None, help="Roll 侧倾角 (弧度 rad, 上限 +/-0.10)")
    parser.add_argument("--pitch-rad", type=float, default=None, help="Pitch 俯仰角 (弧度 rad, 上限 +/-0.10)")
    parser.add_argument("--yaw-rad", type=float, default=None, help="Yaw 偏航角 (弧度 rad, 上限 +/-0.10)")
    parser.add_argument("--speed", type=float, default=200.0, help="轨迹规划峰值速度 (step/s, 默认: 200, 即 5.0 mm/s)")
    parser.add_argument("--hold-time", type=float, default=2.0, help="在目标位置保持观察时间 (秒, 默认: 2.0)")
    parser.add_argument("--no-return", action="store_true", help="到达目标后保持在该位置，不自动返回 OPERATING_HOME")
    parser.add_argument("--real-hardware", action="store_true", help="连接真实六轴硬件")
    parser.add_argument("--confirm-motion", action="store_true", help="人工确认机构运动区域安全")
    parser.add_argument("--validated-config", action="store_true", help="人工确认参数配置已验证")

    args = parser.parse_args()

    success = move_to_cartesian_offset(
        dx_mm=args.x,
        dy_mm=args.y,
        dz_mm=args.z,
        roll_deg=args.roll,
        pitch_deg=args.pitch,
        yaw_deg=args.yaw,
        roll_rad=args.roll_rad,
        pitch_rad=args.pitch_rad,
        yaw_rad=args.yaw_rad,
        peak_speed=args.speed,
        hold_time=args.hold_time,
        return_home=(not args.no_return),
        real_hardware=args.real_hardware,
        confirm_motion=args.confirm_motion,
        validated_config=args.validated_config,
    )

    if not success:
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
