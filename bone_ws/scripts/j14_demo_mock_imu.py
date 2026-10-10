#!/usr/bin/env python3
"""Software-only dual IMU input. Never opens serial or publishes actuator commands."""
import argparse
import math
from pathlib import Path
import signal
import threading
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--amplitude-rad', type=float, default=.005)
    args = parser.parse_args()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('j14_demo_mock_dual_imu')
    pubs = [(node.create_publisher(Imu, f'/pus6/imu_{role}/data', 10),
             f'pus6_{role}_imu_link') for role in ('base', 'platform')]
    active_start = None
    try:
        while not stop.is_set():
            # Hold stationary through the frozen runner's reference capture.
            if active_start is None and (args.out_dir / 'controller_state.json').exists():
                import json
                if json.loads((args.out_dir / 'controller_state.json').read_text())['state'] == 'ACTIVE':
                    active_start = time.monotonic()
            elapsed = time.monotonic() - active_start if active_start is not None else 0.0
            angle = args.amplitude_rad * math.sin(2 * math.pi * 0.25 * elapsed)
            for index, (pub, frame) in enumerate(pubs):
                msg = Imu()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = frame
                # Base excitation + independent small platform error for residual paths.
                a = angle if index == 0 else angle * 0.3
                msg.orientation.x = math.sin(a / 2)
                msg.orientation.w = math.cos(a / 2)
                pub.publish(msg)
            stop.wait(0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
