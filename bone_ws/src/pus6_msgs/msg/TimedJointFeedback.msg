# Standard Header with ROS node clock publish time
std_msgs/Header header

# Host-side CLOCK_MONOTONIC timing in nanoseconds
int64 snapshot_start_monotonic_ns
int64 snapshot_end_monotonic_ns
# Host-side 0x29 feedback reception completion time for ID1..ID6 (not motor internal sample time)
int64[6] per_axis_rx_done_monotonic_ns

# 6-axis actual feedback measurements
float64[6] positions_meters
int32[6] raw_steps
int32[6] currents_ma
int32[6] forces_g_raw
int32[6] temperatures_c
int32[6] fault_codes

# Shared 1-to-1 UART cycle sequence counter
uint64 uart_cycle_sequence
