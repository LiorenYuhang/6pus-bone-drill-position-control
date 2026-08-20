#!/usr/bin/env python3
"""Record ROS 2 IMU data to CSV and plot it with a one-second refresh."""
import csv
import math
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Temperature

CSV_FIELDS = [
    "datetime", "ros_time_sec", "sample",
    "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2", "accel_norm_m_s2",
    "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s", "gyro_norm_rad_s",
    "quat_x", "quat_y", "quat_z", "quat_w", "roll_rad", "pitch_rad",
    "yaw_rad", "roll_deg", "pitch_deg", "yaw_deg", "temperature_c",
]


class ImuCsvRecorder(Node):
    def __init__(self) -> None:
        super().__init__("imu_csv_visualizer")
        self.declare_parameter("output_dir", "~/bone_drill/up/sensor/IMU/data")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("euler_topic", "/imu/euler")
        self.declare_parameter("temperature_topic", "/imu/temperature")
        self.declare_parameter("history_seconds", 60.0)
        self.declare_parameter("expected_rate_hz", 200)
        output_dir = Path(self.get_parameter("output_dir").value).expanduser()
        output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = output_dir / datetime.now().strftime("imu_%Y%m%d_%H%M%S.csv")
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        capacity = max(100, int(
            float(self.get_parameter("history_seconds").value)
            * int(self.get_parameter("expected_rate_hz").value)
        ))
        self.lock = threading.Lock()
        self.sample = 0
        self.start_time = None
        self.latest_euler = [math.nan] * 3
        self.latest_temperature = math.nan
        self.times = deque(maxlen=capacity)
        self.acceleration = [deque(maxlen=capacity) for _ in range(4)]
        self.angular_velocity = [deque(maxlen=capacity) for _ in range(4)]
        self.quaternion = [deque(maxlen=capacity) for _ in range(4)]
        self.euler_deg = [deque(maxlen=capacity) for _ in range(3)]
        self.temperature = deque(maxlen=capacity)
        self.create_subscription(Imu, self.get_parameter("imu_topic").value,
                                 self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped,
                                 self.get_parameter("euler_topic").value,
                                 self.on_euler, qos_profile_sensor_data)
        self.create_subscription(Temperature,
                                 self.get_parameter("temperature_topic").value,
                                 self.on_temperature, qos_profile_sensor_data)
        self.create_timer(1.0, self.csv_file.flush)
        self.get_logger().info(f"CSV recording started: {self.csv_path}")

    def on_euler(self, msg: Vector3Stamped) -> None:
        with self.lock:
            self.latest_euler = [msg.vector.x, msg.vector.y, msg.vector.z]

    def on_temperature(self, msg: Temperature) -> None:
        with self.lock:
            self.latest_temperature = msg.temperature

    def on_imu(self, msg: Imu) -> None:
        ros_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if ros_time <= 0.0:
            ros_time = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = ros_time
        elapsed = ros_time - self.start_time
        accel = [msg.linear_acceleration.x, msg.linear_acceleration.y,
                 msg.linear_acceleration.z]
        accel.append(math.sqrt(sum(value * value for value in accel)))
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y,
                msg.angular_velocity.z]
        gyro.append(math.sqrt(sum(value * value for value in gyro)))
        quat = [msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w]
        with self.lock:
            self.sample += 1
            euler = list(self.latest_euler)
            temperature = self.latest_temperature
            self.times.append(elapsed)
            for series, value in zip(self.acceleration, accel):
                series.append(value)
            for series, value in zip(self.angular_velocity, gyro):
                series.append(value)
            for series, value in zip(self.quaternion, quat):
                series.append(value)
            for series, value in zip(self.euler_deg, euler):
                series.append(math.degrees(value))
            self.temperature.append(temperature)
            self.csv_writer.writerow({
                "datetime": datetime.now().isoformat(timespec="milliseconds"),
                "ros_time_sec": f"{ros_time:.9f}", "sample": self.sample,
                "accel_x_m_s2": accel[0], "accel_y_m_s2": accel[1],
                "accel_z_m_s2": accel[2], "accel_norm_m_s2": accel[3],
                "gyro_x_rad_s": gyro[0], "gyro_y_rad_s": gyro[1],
                "gyro_z_rad_s": gyro[2], "gyro_norm_rad_s": gyro[3],
                "quat_x": quat[0], "quat_y": quat[1], "quat_z": quat[2],
                "quat_w": quat[3], "roll_rad": euler[0],
                "pitch_rad": euler[1], "yaw_rad": euler[2],
                "roll_deg": math.degrees(euler[0]),
                "pitch_deg": math.degrees(euler[1]),
                "yaw_deg": math.degrees(euler[2]),
                "temperature_c": temperature,
            })

    def snapshot(self):
        with self.lock:
            return {
                "times": list(self.times),
                "acceleration": [list(x) for x in self.acceleration],
                "angular_velocity": [list(x) for x in self.angular_velocity],
                "quaternion": [list(x) for x in self.quaternion],
                "euler_deg": [list(x) for x in self.euler_deg],
                "temperature": list(self.temperature), "sample": self.sample,
            }

    def destroy_node(self) -> bool:
        self.csv_file.flush()
        self.csv_file.close()
        return super().destroy_node()


class RealtimePlot:
    def __init__(self, node: ImuCsvRecorder) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        plt.rcParams["axes.unicode_minus"] = False
        self.node = node
        self.plt = plt
        self.figure, self.axes = plt.subplots(5, 1, figsize=(13, 9), sharex=True)
        self.figure.canvas.manager.set_window_title("H30 IMU Real-time Data")
        self.figure.suptitle("H30 IMU Real-time Curves (1 s refresh)")
        self.lines = []
        specs = [
            ("Linear acceleration (m/s²)", ["X", "Y", "Z", "Norm"]),
            ("Angular velocity (rad/s)", ["X", "Y", "Z", "Norm"]),
            ("Orientation quaternion", ["X", "Y", "Z", "W"]),
            ("Euler angles (deg)", ["Roll", "Pitch", "Yaw"]),
            ("Temperature (°C)", ["Temperature"]),
        ]
        for axis, (ylabel, labels) in zip(self.axes, specs):
            axis.set_ylabel(ylabel)
            axis.grid(True, alpha=0.3)
            self.lines.append([axis.plot([], [], label=x)[0] for x in labels])
            axis.legend(loc="upper left", ncol=len(labels), fontsize=8)
        self.axes[-1].set_xlabel("Elapsed time (s)")
        self.status = self.figure.text(0.99, 0.01, "", ha="right", fontsize=9)
        self.animation = FuncAnimation(self.figure, self.update, interval=1000,
                                       cache_frame_data=False)

    def update(self, _frame):
        data = self.node.snapshot()
        times = data["times"]
        artists = [line for group in self.lines for line in group]
        if not times:
            self.status.set_text(f"Waiting for /imu/data | CSV: {self.node.csv_path}")
            return artists
        groups = [data["acceleration"], data["angular_velocity"],
                  data["quaternion"], data["euler_deg"], [data["temperature"]]]
        for axis, lines, values_group in zip(self.axes, self.lines, groups):
            for line, values in zip(lines, values_group):
                line.set_data(times, values)
            axis.relim()
            axis.autoscale_view()
        self.axes[-1].set_xlim(times[0], max(times[-1], times[0] + 1.0))
        self.status.set_text(f"Samples: {data['sample']} | CSV: {self.node.csv_path}")
        return artists

    def show(self) -> None:
        self.plt.tight_layout(rect=(0, 0.03, 1, 0.96))
        self.plt.show()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuCsvRecorder()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        RealtimePlot(node).show()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
