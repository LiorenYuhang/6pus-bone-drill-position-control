# Standard Header with ROS node clock publish time
std_msgs/Header header

# Host-side CLOCK_MONOTONIC timing in nanoseconds
int64 transaction_start_monotonic_ns
int64 transaction_end_monotonic_ns
# Host-side 0x29 write transaction initiation and completion times for ID1..ID6
int64[6] per_axis_tx_start_monotonic_ns
int64[6] per_axis_tx_done_monotonic_ns

# 6-axis dispatched commands
float64[6] target_positions_meters
int32[6] target_steps

# Shared 1-to-1 UART cycle sequence counter
uint64 uart_cycle_sequence
