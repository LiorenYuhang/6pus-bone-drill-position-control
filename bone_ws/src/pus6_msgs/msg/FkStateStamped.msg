# Standard ROS 2 header (stamp = ROS clock message publish time)
std_msgs/Header header

# Host CLOCK_MONOTONIC query timestamp for 6-axis synchronized configuration (ns)
int64 query_monotonic_timestamp_ns

# 6-axis synchronized slider positions [q1..q6] (meters)
float64[6] synced_positions_meters

# 6-axis synchronized raw steps
float64[6] synced_raw_steps

# 6DoF Pose: position (x, y, z in meters), orientation (quaternion x, y, z, w)
geometry_msgs/Pose pose

# ZYX Euler angles: [roll, pitch, yaw] in radians
float64[3] euler_angles_rad

# Convergence status
bool converged

# Number of LM solver iterations
int32 iterations

# L2 norm of the kinematic strut length constraint residual (meters)
float64 residual_norm_meters

# Branch continuity jump check passed
bool branch_continuity_passed

# FK calculation computation duration on host (microseconds)
float64 compute_time_us
