"""High-resolution timing, jitter, and statistical metrics utilities for Stage J2-F0."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import numpy as np


@dataclass
class CycleTimingStats:
    """Statistical summary of cycle timings (ms) and rates (Hz)."""
    sample_count: int
    duration_sec: float
    min_ms: float
    mean_ms: float
    median_p50_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float
    std_ms: float
    sustained_rate_hz: float
    theoretical_wire_capacity_hz: float
    theoretical_wire_time_ms: float


@dataclass
class RosEndToEndLatencyStats:
    """Statistical summary of ROS 2 Driver pub -> Auditor recv latency (ms)."""
    sample_count: int
    min_ms: float
    mean_ms: float
    median_p50_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float
    std_ms: float


@dataclass
class SweepPointStats:
    """Summary of a single sweep frequency point."""
    target_frequency_hz: float
    target_period_ms: float
    actual_publish_rate_hz: float
    auditor_receive_rate_hz: float
    auditor_delivery_ratio: float
    on_time_ratio: float
    deadline_miss_count: int
    max_consecutive_deadline_miss: int
    t_cycle: CycleTimingStats
    e2e_latency: RosEndToEndLatencyStats | None
    is_sustainable: bool
    fail_reasons: list[str]


def compute_cycle_timing_stats(
    durations_ns: list[int] | np.ndarray,
    total_elapsed_sec: float,
    theoretical_wire_time_ms: float = 1.823,
) -> CycleTimingStats:
    """Compute comprehensive timing statistics from nanosecond-precision sample durations."""
    if len(durations_ns) == 0:
        return CycleTimingStats(
            sample_count=0,
            duration_sec=total_elapsed_sec,
            min_ms=0.0,
            mean_ms=0.0,
            median_p50_ms=0.0,
            p95_ms=0.0,
            p99_ms=0.0,
            max_ms=0.0,
            std_ms=0.0,
            sustained_rate_hz=0.0,
            theoretical_wire_capacity_hz=1000.0 / theoretical_wire_time_ms if theoretical_wire_time_ms > 0 else 0.0,
            theoretical_wire_time_ms=theoretical_wire_time_ms,
        )

    dur_ms = np.array(durations_ns, dtype=np.float64) / 1e6
    count = len(dur_ms)
    mean_ms = float(np.mean(dur_ms))
    sustained_rate = float(count / total_elapsed_sec) if total_elapsed_sec > 0 else 0.0
    wire_cap = 1000.0 / theoretical_wire_time_ms if theoretical_wire_time_ms > 0 else 0.0

    return CycleTimingStats(
        sample_count=count,
        duration_sec=float(total_elapsed_sec),
        min_ms=float(np.min(dur_ms)),
        mean_ms=mean_ms,
        median_p50_ms=float(np.median(dur_ms)),
        p95_ms=float(np.percentile(dur_ms, 95)),
        p99_ms=float(np.percentile(dur_ms, 99)),
        max_ms=float(np.max(dur_ms)),
        std_ms=float(np.std(dur_ms)),
        sustained_rate_hz=sustained_rate,
        theoretical_wire_capacity_hz=wire_cap,
        theoretical_wire_time_ms=theoretical_wire_time_ms,
    )


def compute_e2e_latency_stats(latencies_ns: list[int] | np.ndarray) -> RosEndToEndLatencyStats | None:
    """Compute ROS 2 Driver -> Auditor latency statistics."""
    if len(latencies_ns) == 0:
        return None
    lat_ms = np.array(latencies_ns, dtype=np.float64) / 1e6
    return RosEndToEndLatencyStats(
        sample_count=len(lat_ms),
        min_ms=float(np.min(lat_ms)),
        mean_ms=float(np.mean(lat_ms)),
        median_p50_ms=float(np.median(lat_ms)),
        p95_ms=float(np.percentile(lat_ms, 95)),
        p99_ms=float(np.percentile(lat_ms, 99)),
        max_ms=float(np.max(lat_ms)),
        std_ms=float(np.std(lat_ms)),
    )


def evaluate_sweep_point(
    target_hz: float,
    duration_sec: float,
    cycle_durations_ns: list[int],
    driver_pub_stamps_ns: list[int],
    auditor_recv_stamps_ns: list[int],
    e2e_latencies_ns: list[int],
    wire_time_ms: float = 1.823,
    deadline_tolerance_ms: float = 2.0,
) -> SweepPointStats:
    """Evaluate sustainable ROS 2 rate criteria on a single sweep point."""
    target_period_ms = 1000.0 / target_hz
    cycle_stats = compute_cycle_timing_stats(cycle_durations_ns, duration_sec, wire_time_ms)
    e2e_stats = compute_e2e_latency_stats(e2e_latencies_ns)

    expected_msgs = int(round(target_hz * duration_sec))
    pub_count = len(driver_pub_stamps_ns)
    recv_count = len(auditor_recv_stamps_ns)

    delivery_ratio = (recv_count / pub_count) if pub_count > 0 else 0.0
    actual_pub_rate = (pub_count / duration_sec) if duration_sec > 0 else 0.0
    auditor_recv_rate = (recv_count / duration_sec) if duration_sec > 0 else 0.0

    # Calculate intervals and deadline misses between consecutive publishes
    fail_reasons: list[str] = []
    deadline_miss_count = 0
    max_consecutive_miss = 0
    current_consecutive_miss = 0

    if len(driver_pub_stamps_ns) > 1:
        intervals_ms = np.diff(np.array(driver_pub_stamps_ns, dtype=np.float64)) / 1e6
        for interval in intervals_ms:
            error = abs(interval - target_period_ms)
            if error >= deadline_tolerance_ms:
                deadline_miss_count += 1
                current_consecutive_miss += 1
                if current_consecutive_miss > max_consecutive_miss:
                    max_consecutive_miss = current_consecutive_miss
            else:
                current_consecutive_miss = 0

        on_time_ratio = 1.0 - (deadline_miss_count / len(intervals_ms))
    else:
        on_time_ratio = 0.0

    # Evaluate sustainability criteria
    if on_time_ratio < 0.98:
        fail_reasons.append(f"On-time ratio {on_time_ratio*100:.1f}% < 98.0%")
    if delivery_ratio < 0.99:
        fail_reasons.append(f"Auditor delivery ratio {delivery_ratio*100:.1f}% < 99.0%")
    if cycle_stats.p95_ms >= target_period_ms:
        fail_reasons.append(f"T_cycle P95 ({cycle_stats.p95_ms:.2f}ms) >= Target Period ({target_period_ms:.2f}ms)")

    is_sustainable = len(fail_reasons) == 0

    return SweepPointStats(
        target_frequency_hz=target_hz,
        target_period_ms=target_period_ms,
        actual_publish_rate_hz=actual_pub_rate,
        auditor_receive_rate_hz=auditor_recv_rate,
        auditor_delivery_ratio=float(delivery_ratio),
        on_time_ratio=float(on_time_ratio),
        deadline_miss_count=deadline_miss_count,
        max_consecutive_deadline_miss=max_consecutive_miss,
        t_cycle=cycle_stats,
        e2e_latency=e2e_stats,
        is_sustainable=is_sustainable,
        fail_reasons=fail_reasons,
    )
