#!/usr/bin/env python3
"""J9.1 platform H30 static recorder.

This is a sidecar recorder for the existing J8 H30 ROS interface. It subscribes
to sensor_msgs/Imu and writes host timestamps, ROS header stamps, raw
orientation, HOME-relative RPY, and gyro to CSV. It does not publish commands or
touch the motor control path.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


CSV_FIELDS = [
    "host_log_time_ns",
    "host_monotonic_time_ns",
    "ros_stamp_sec",
    "ros_stamp_nanosec",
    "imu_source_timestamp",
    "frame_id",
    "qx",
    "qy",
    "qz",
    "qw",
    "relative_roll_deg",
    "relative_pitch_deg",
    "relative_yaw_deg",
    "gx_rad_s",
    "gy_rad_s",
    "gz_rad_s",
]


@dataclass(frozen=True)
class RecorderSummary:
    csv_path: str
    metadata_path: str
    topic: str
    duration_s: float
    init_duration_s: float
    sample_count: int
    recorded_count: int
    average_sample_rate_hz: float
    ros_timestamp_monotonic: bool
    host_timestamp_monotonic: bool
    quaternion_valid_count: int
    quaternion_invalid_count: int
    q0_xyzw: tuple[float, float, float, float] | None
    relative_roll_deg_mean: float | None
    relative_pitch_deg_mean: float | None
    relative_yaw_deg_mean: float | None
    relative_roll_deg_min: float | None
    relative_roll_deg_max: float | None
    relative_pitch_deg_min: float | None
    relative_pitch_deg_max: float | None
    relative_yaw_deg_min: float | None
    relative_yaw_deg_max: float | None
    gx_rad_s_mean: float | None
    gy_rad_s_mean: float | None
    gz_rad_s_mean: float | None
    gx_rad_s_min: float | None
    gx_rad_s_max: float | None
    gy_rad_s_min: float | None
    gy_rad_s_max: float | None
    gz_rad_s_min: float | None
    gz_rad_s_max: float | None


def normalize_quat_xyzw(q: Sequence[float]) -> tuple[float, float, float, float] | None:
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        return None
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1e-9:
        return None
    return tuple(float(v) / norm for v in q)  # type: ignore[return-value]


def quat_conjugate_xyzw(q: Sequence[float]) -> tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def quat_multiply_xyzw(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def average_quat_xyzw(samples: Sequence[Sequence[float]]) -> tuple[float, float, float, float] | None:
    normalized = [q for q in (normalize_quat_xyzw(s) for s in samples) if q is not None]
    if not normalized:
        return None
    ref = normalized[0]
    acc = [0.0, 0.0, 0.0, 0.0]
    for q in normalized:
        dot = sum(ref[i] * q[i] for i in range(4))
        sign = -1.0 if dot < 0.0 else 1.0
        for i in range(4):
            acc[i] += sign * q[i]
    return normalize_quat_xyzw(acc)


def relative_quat_xyzw(q0_xyzw: Sequence[float], q_xyzw: Sequence[float]) -> tuple[float, float, float, float] | None:
    q0 = normalize_quat_xyzw(q0_xyzw)
    q = normalize_quat_xyzw(q_xyzw)
    if q0 is None or q is None:
        return None
    return normalize_quat_xyzw(quat_multiply_xyzw(quat_conjugate_xyzw(q0), q))


def quat_to_rpy_deg_xyzw(q: Sequence[float]) -> tuple[float, float, float]:
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def _mean(values: Sequence[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _min(values: Sequence[float]) -> float | None:
    return min(values) if values else None


def _max(values: Sequence[float]) -> float | None:
    return max(values) if values else None


class PlatformImuRecorder(Node):
    def __init__(
        self,
        *,
        topic: str,
        csv_path: Path,
        metadata_path: Path,
        duration_s: float,
        init_duration_s: float,
        min_init_samples: int,
    ) -> None:
        super().__init__("j9_1_platform_imu_recorder")
        self.topic = topic
        self.csv_path = csv_path
        self.metadata_path = metadata_path
        self.duration_s = max(0.1, float(duration_s))
        self.init_duration_s = max(0.0, float(init_duration_s))
        self.min_init_samples = max(1, int(min_init_samples))

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()

        self._first_host_ns: int | None = None
        self._last_host_ns: int | None = None
        self._last_ros_ns: int | None = None
        self._host_monotonic = True
        self._ros_monotonic = True

        self._init_quats: list[tuple[float, float, float, float]] = []
        self._q0: tuple[float, float, float, float] | None = None

        self._sample_count = 0
        self._recorded_count = 0
        self._quat_valid = 0
        self._quat_invalid = 0
        self._rel_roll: list[float] = []
        self._rel_pitch: list[float] = []
        self._rel_yaw: list[float] = []
        self._gx: list[float] = []
        self._gy: list[float] = []
        self._gz: list[float] = []

        self._sub = self.create_subscription(Imu, topic, self._on_imu, qos_profile_sensor_data)
        self._timer = self.create_timer(0.05, self._check_done)
        self._done = False
        self.summary: RecorderSummary | None = None
        self.get_logger().info(
            f"Recording {topic} to {self.csv_path}; init_duration={self.init_duration_s:.3f}s"
        )

    def close(self) -> None:
        if not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()

    def _elapsed_s(self, host_ns: int) -> float:
        if self._first_host_ns is None:
            return 0.0
        return (host_ns - self._first_host_ns) / 1e9

    def _on_imu(self, msg: Imu) -> None:
        host_ns = time.time_ns()
        mono_ns = time.monotonic_ns()
        ros_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self._first_host_ns is None:
            self._first_host_ns = host_ns
        if self._last_host_ns is not None and host_ns <= self._last_host_ns:
            self._host_monotonic = False
        if self._last_ros_ns is not None and ros_ns <= self._last_ros_ns:
            self._ros_monotonic = False
        self._last_host_ns = host_ns
        self._last_ros_ns = ros_ns
        self._sample_count += 1

        q = normalize_quat_xyzw((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
        if q is None or msg.orientation_covariance[0] == -1.0:
            self._quat_invalid += 1
            return
        self._quat_valid += 1

        elapsed = self._elapsed_s(host_ns)
        if self._q0 is None:
            self._init_quats.append(q)
            enough_time = elapsed >= self.init_duration_s
            enough_samples = len(self._init_quats) >= self.min_init_samples
            if not (enough_time and enough_samples):
                return
            self._q0 = average_quat_xyzw(self._init_quats)
            if self._q0 is None:
                return
            self.get_logger().info(f"HOME relative zero initialized from {len(self._init_quats)} samples")

        q_rel = relative_quat_xyzw(self._q0, q)
        if q_rel is None:
            self._quat_invalid += 1
            return
        roll_deg, pitch_deg, yaw_deg = quat_to_rpy_deg_xyzw(q_rel)

        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)
        self._rel_roll.append(roll_deg)
        self._rel_pitch.append(pitch_deg)
        self._rel_yaw.append(yaw_deg)
        self._gx.append(gx)
        self._gy.append(gy)
        self._gz.append(gz)

        self._writer.writerow(
            {
                "host_log_time_ns": host_ns,
                "host_monotonic_time_ns": mono_ns,
                "ros_stamp_sec": int(msg.header.stamp.sec),
                "ros_stamp_nanosec": int(msg.header.stamp.nanosec),
                "imu_source_timestamp": "",
                "frame_id": msg.header.frame_id,
                "qx": q[0],
                "qy": q[1],
                "qz": q[2],
                "qw": q[3],
                "relative_roll_deg": roll_deg,
                "relative_pitch_deg": pitch_deg,
                "relative_yaw_deg": yaw_deg,
                "gx_rad_s": gx,
                "gy_rad_s": gy,
                "gz_rad_s": gz,
            }
        )
        self._recorded_count += 1

    def _check_done(self) -> None:
        if self._first_host_ns is None:
            return
        if self._elapsed_s(time.time_ns()) >= self.duration_s:
            self._done = True

    def done(self) -> bool:
        return self._done

    def build_summary(self) -> RecorderSummary:
        elapsed = 0.0
        if self._first_host_ns is not None and self._last_host_ns is not None:
            elapsed = max(0.0, (self._last_host_ns - self._first_host_ns) / 1e9)
        rate = (self._sample_count - 1) / elapsed if self._sample_count > 1 and elapsed > 0.0 else 0.0
        return RecorderSummary(
            csv_path=str(self.csv_path),
            metadata_path=str(self.metadata_path),
            topic=self.topic,
            duration_s=elapsed,
            init_duration_s=self.init_duration_s,
            sample_count=self._sample_count,
            recorded_count=self._recorded_count,
            average_sample_rate_hz=rate,
            ros_timestamp_monotonic=self._ros_monotonic,
            host_timestamp_monotonic=self._host_monotonic,
            quaternion_valid_count=self._quat_valid,
            quaternion_invalid_count=self._quat_invalid,
            q0_xyzw=self._q0,
            relative_roll_deg_mean=_mean(self._rel_roll),
            relative_pitch_deg_mean=_mean(self._rel_pitch),
            relative_yaw_deg_mean=_mean(self._rel_yaw),
            relative_roll_deg_min=_min(self._rel_roll),
            relative_roll_deg_max=_max(self._rel_roll),
            relative_pitch_deg_min=_min(self._rel_pitch),
            relative_pitch_deg_max=_max(self._rel_pitch),
            relative_yaw_deg_min=_min(self._rel_yaw),
            relative_yaw_deg_max=_max(self._rel_yaw),
            gx_rad_s_mean=_mean(self._gx),
            gy_rad_s_mean=_mean(self._gy),
            gz_rad_s_mean=_mean(self._gz),
            gx_rad_s_min=_min(self._gx),
            gx_rad_s_max=_max(self._gx),
            gy_rad_s_min=_min(self._gy),
            gy_rad_s_max=_max(self._gy),
            gz_rad_s_min=_min(self._gz),
            gz_rad_s_max=_max(self._gz),
        )

    def write_metadata(self) -> RecorderSummary:
        self.summary = self.build_summary()
        metadata = {
            **self.summary.__dict__,
            "orientation_quaternion_convention": (
                "sensor_msgs/Imu orientation xyzw; J8 H30 driver maps vendor q0,q1,q2,q3 "
                "to w,x,y,z and normalizes before publishing"
            ),
            "relative_orientation_definition": "q_rel = conjugate(q0_home) * q_imu(t), equivalent to R0^T * R_imu(t)",
            "gyro_units": "rad/s from sensor_msgs/Imu.angular_velocity; J8 H30 driver converts vendor deg/s to rad/s",
            "imu_source_timestamp": "unavailable in current J8 sensor_msgs/Imu interface",
            "motor_command_policy": "recorder subscribes to IMU only and sends no motor commands",
        }
        self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
        self.metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        return self.summary


def default_output_paths(output_dir: Path) -> tuple[Path, Path]:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return (
        output_dir / f"j9_1_platform_imu_{stamp}.csv",
        output_dir / f"j9_1_platform_imu_{stamp}_metadata.json",
    )


def print_summary(summary: RecorderSummary) -> None:
    print("\n================ J9.1 PLATFORM IMU RECORDER SUMMARY ================")
    print(f"CSV: {summary.csv_path}")
    print(f"Metadata: {summary.metadata_path}")
    print(f"Topic: {summary.topic}")
    print(f"Samples seen: {summary.sample_count}")
    print(f"Samples recorded after HOME zero: {summary.recorded_count}")
    print(f"Average sample rate: {summary.average_sample_rate_hz:.2f} Hz")
    print(f"ROS timestamp monotonic: {summary.ros_timestamp_monotonic}")
    print(f"Host timestamp monotonic: {summary.host_timestamp_monotonic}")
    print(f"Quaternion valid/invalid: {summary.quaternion_valid_count}/{summary.quaternion_invalid_count}")
    print(
        "Relative RPY mean deg: "
        f"roll={summary.relative_roll_deg_mean}, "
        f"pitch={summary.relative_pitch_deg_mean}, "
        f"yaw={summary.relative_yaw_deg_mean}"
    )
    print(
        "Gyro mean rad/s: "
        f"gx={summary.gx_rad_s_mean}, gy={summary.gy_rad_s_mean}, gz={summary.gz_rad_s_mean}"
    )
    print("Motor writes: 0")
    print("====================================================================")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="J9.1 Platform H30 sidecar CSV recorder")
    parser.add_argument("--topic", default="/imu/data", help="sensor_msgs/Imu topic to record")
    parser.add_argument("--duration", type=float, default=5.0, help="Total recording duration after first sample, seconds")
    parser.add_argument("--init-duration", type=float, default=1.0, help="Initial HOME-zero averaging window, seconds")
    parser.add_argument("--min-init-samples", type=int, default=10, help="Minimum samples used to initialize HOME zero")
    parser.add_argument("--output-dir", default="/tmp", help="Directory for CSV and metadata outputs")
    parser.add_argument("--output-csv", default="", help="Explicit CSV path")
    parser.add_argument("--metadata-json", default="", help="Explicit metadata JSON path")
    args = parser.parse_args(list(argv) if argv is not None else None)

    output_dir = Path(args.output_dir)
    default_csv, default_metadata = default_output_paths(output_dir)
    csv_path = Path(args.output_csv) if args.output_csv else default_csv
    metadata_path = Path(args.metadata_json) if args.metadata_json else default_metadata

    rclpy.init()
    node = PlatformImuRecorder(
        topic=args.topic,
        csv_path=csv_path,
        metadata_path=metadata_path,
        duration_s=args.duration,
        init_duration_s=args.init_duration,
        min_init_samples=args.min_init_samples,
    )
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by operator")
    finally:
        summary = node.write_metadata()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print_summary(summary)
    return 0 if summary.recorded_count > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
