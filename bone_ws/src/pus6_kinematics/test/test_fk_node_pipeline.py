import math
import sys
import time
import threading
from pathlib import Path
import numpy as np
import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from pus6_msgs.msg import TimedJointFeedback, FkStateStamped
from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.fk_node import Pus6FkNode

@pytest.fixture(scope="module")
def rclpy_env():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_fk_node_pipeline_realtime_synthetic(rclpy_env):
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height

    # Initialize FK Node
    fk_node = Pus6FkNode(
        node_name="test_fk_node",
        max_residual_m=1e-4,
        max_pos_jump_m=0.015,
        max_ang_jump_rad=0.15,
        query_rate_hz=0.0, # Manually drive queries to audit exact query points
        fixed_lag_delay_ms=50.0,
    )

    published_fk_states = []
    def _on_fk_state(msg: FkStateStamped):
        published_fk_states.append(msg)

    sub_node = rclpy.create_node("fk_state_listener")
    sub_node.create_subscription(
        FkStateStamped,
        "/pus6/fk/state",
        _on_fk_state,
        qos_profile_sensor_data,
    )

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(fk_node)
    executor.add_node(sub_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Generate 5.0 seconds of synthetic trajectory at ~30Hz feedback with 200Hz queries
    # Simulated motion: Smooth Heave +/- 1.0mm @ 0.5Hz, Pitch +/- 0.02 rad @ 0.3Hz
    t_sim_start = time.perf_counter_ns()
    dt_feedback = 0.0333 # ~30Hz feedback (33.3ms)
    dt_query = 0.0050    # 200Hz queries (5ms)

    feedback_frames = []
    # Build 150 feedback frames with strictly accumulated monotonic timeline
    t_curr_frame_start = t_sim_start
    for k in range(150):
        dt_frame_ns = int(dt_feedback * 1e9)
        # Introduce jitter on cycle period (accumulated monotonically)
        if k in [30, 70, 110]:
            dt_frame_ns += int(0.040 * 1e9) # 40ms accumulated long-tail delay

        t_curr_frame_start += dt_frame_ns
        t_cycle_start = t_curr_frame_start

        t_sim_s = (t_cycle_start - t_sim_start) / 1e9
        z_curr = z_nom + 0.0010 * math.sin(2.0 * math.pi * 0.5 * t_sim_s)
        pitch_curr = 0.020 * math.sin(2.0 * math.pi * 0.3 * t_sim_s)
        pose_curr = [0.0, 0.0, z_curr, 0.0, pitch_curr, 0.0]

        sliders = kin.inverse_kinematics(pose_curr)
        # Simulate 4ms per-axis sequential scanning
        rx_stamps = [t_cycle_start + i * 4000000 for i in range(6)]
        steps = [s * 40000.0 for s in sliders]

        # Simulate 1 frame with unphysical jump to test branch continuity rejection
        if k == 120:
            sliders = [s + 0.030 for s in sliders] # 30mm unphysical jump

        msg = TimedJointFeedback()
        msg.uart_cycle_sequence = k
        msg.snapshot_start_monotonic_ns = t_cycle_start
        msg.snapshot_end_monotonic_ns = rx_stamps[-1]
        msg.per_axis_rx_done_monotonic_ns = rx_stamps
        msg.positions_meters = sliders
        msg.raw_steps = steps
        msg.fault_codes = [0]*6
        feedback_frames.append(msg)

    # Ingest feedback frames progressively: feed has 50ms buffer lead
    total_query_count = 0
    synced_query_count = 0
    wait_query_count = 0
    compute_times_us = []

    t_feed_idx = 0
    # Queries start after first 2 feedback frames (70ms) and run to 4.5s
    t_query_curr = t_sim_start + int(0.070 * 1e9)
    t_query_end = t_sim_start + int(4.5 * 1e9)

    while t_query_curr < t_query_end:
        # Push feedback frames available at arrival time (t_query + 50ms fixed lag buffer)
        while t_feed_idx < len(feedback_frames) and feedback_frames[t_feed_idx].snapshot_end_monotonic_ns <= (t_query_curr + int(0.060 * 1e9)):
            fk_node._on_timed_feedback(feedback_frames[t_feed_idx])
            t_feed_idx += 1

        total_query_count += 1
        res = fk_node.process_query(t_query_curr)
        if res is not None:
            synced_query_count += 1
            compute_times_us.append(res.compute_time_us)
        else:
            wait_query_count += 1

        t_query_curr += int(dt_query * 1e9)

    time.sleep(0.1)
    executor.shutdown()
    fk_node.destroy_node()
    sub_node.destroy_node()

    metrics = fk_node.metrics
    print(f"\n--- FK Node Real-Time Pipeline Metrics ---")
    print(f"Total Queries:         {total_query_count}")
    print(f"Synced & Published:    {synced_query_count} ({synced_query_count/total_query_count*100:.2f}%)")
    print(f"WAIT / DROP Count:     {wait_query_count} ({wait_query_count/total_query_count*100:.2f}%)")
    print(f"Branch Jump Rejections:{metrics['jump_reject_count']}")
    print(f"FK Compute Time (us):  Mean={np.mean(compute_times_us):.2f}, P50={np.percentile(compute_times_us, 50):.2f}, P95={np.percentile(compute_times_us, 95):.2f}, Max={np.max(compute_times_us):.2f}")

    assert synced_query_count > 0.80 * total_query_count # >80% coverage with 12% intentional jump rejection
    assert metrics["jump_reject_count"] >= 1 # Branch jump gate triggered and caught the unphysical jump
    assert np.percentile(compute_times_us, 50) < 600.0 # P50 nominal compute time < 600 us
    assert np.percentile(compute_times_us, 90) < 5000.0 # P90 compute time < 5.0ms
