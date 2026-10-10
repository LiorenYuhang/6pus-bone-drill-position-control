#!/usr/bin/env python3
"""Demo-owned relay signals are ready before importing ROS dependencies."""
import signal
import threading
stop=threading.Event()
for sig in (signal.SIGINT,signal.SIGTERM):
    signal.signal(sig,lambda *_:stop.set())
import argparse
from pathlib import Path
from j14_demo_controller import write_json
import time
parser=argparse.ArgumentParser()
parser.add_argument('--out-dir',type=Path,required=True)
args=parser.parse_args()
write_json(args.out_dir/'relay_signal_ready.json',dict(monotonic_ns=time.monotonic_ns()))
from j14_a2_q3_feedback_relay import run_relay
raise SystemExit(run_relay(args.out_dir,stop))
