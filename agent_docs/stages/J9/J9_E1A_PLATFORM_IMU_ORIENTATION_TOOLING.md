# J9-E1A Platform IMU Orientation Extrinsic Calibration Tooling

## Scope

J9-E1A provides offline software tooling only. It estimates `R_P_IP`, the rotation from `pus6_platform_imu_link` into `platform_link`, using platform IMU gyro data and FK-derived platform angular velocity on the J9-D host `CLOCK_MONOTONIC` timeline.

Not included:

- platform IMU lever arm `p_P_IP`
- base IMU orientation or lever arm
- IMU/FK fusion
- stabilization control
- real-hardware calibration motion
- any change to J8 Servo, IK, geometry, motor safety, HOME/PARK, or Design-B feedback behavior

`robot_extrinsics_status` remains `UNCALIBRATED_UNSET` until authorized real-hardware calibration and independent validation pass.

## Equations

Frame convention:

- `R_P_IP`: maps vectors from platform IMU sensor coordinates into `platform_link`.
- `omega_IP_meas`: platform IMU gyro measurement in `pus6_platform_imu_link`, rad/s.
- `omega_P_FK`: platform angular velocity derived from FK rotation trajectory, expressed in `platform_link`, rad/s.
- `b_g_IP`: platform IMU gyro bias in sensor coordinates, rad/s.

The J9-E1A residual is:

```text
r_i = R_P_IP * (omega_IP_meas_i - b_g_IP) - omega_P_FK_i
```

Timing uses the J9-D affine clock model:

```text
t_host_ns = round(1000 * (a * unwrapped_0x51_sample_timestamp_us + b_us))
```

FK samples use:

```text
FkStateStamped.query_monotonic_timestamp_ns
```

ROS header stamps are not used as the calibration timeline. H30 0x52 ready timestamps are retained for latency diagnostics only, not measurement timing.

The offline tool accepts explicit `--affine-a/--affine-b-us`, can estimate the affine model from the platform `TimedImu` host-arrival timestamps, or can load the platform fit from a J9-D analysis artifact through `--j9d-analysis-json` and `--j9d-run-label`.

## FK Angular Velocity Smoothing

The tool derives `omega_P_FK` from FK orientation samples `R_B_P(t)` using local SO(3) polynomial smoothing:

```text
delta_phi_ij = Log(R_B_P(t_i)^-1 * R_B_P(t_j))
delta_phi_ij ~= c0 + c1 * (t_j - t_i) + c2 * (t_j - t_i)^2
omega_P_FK(t_i) = c1
```

This computes body angular velocity in `platform_link` coordinates and avoids noisy adjacent finite differences or Euler-angle differencing.

## Estimator

The estimator alternates two least-squares steps:

1. Given `b_g_IP`, solve Wahba/Kabsch alignment for `R_P_IP` so that `R_P_IP * (omega_IP_meas - b_g_IP)` matches `omega_P_FK`.
2. Given `R_P_IP`, solve gyro bias as:

```text
b_g_IP = mean(omega_IP_meas_i - R_P_IP^T * omega_P_FK_i)
```

The final observability Jacobian uses small orientation perturbations and bias parameters:

```text
J_i = [ -skew(R_P_IP * (omega_IP_meas_i - b_g_IP))   -R_P_IP ]
```

Reported diagnostics include rank, singular values, condition number, residual RMS/P95/max, and covariance when the rank is full.

## Observability Requirements

| Excitation | Orientation observability | Bias separation | Expected result |
| --- | --- | --- | --- |
| Stationary only | none | bias only, weak without reference | reject |
| Roll only | incomplete | partial | poor rank/conditioning |
| Pitch only | incomplete | partial | poor rank/conditioning |
| Yaw only | incomplete | partial | poor rank/conditioning |
| Roll + pitch + yaw | good if time aligned | good | candidate fit |
| Combined multi-axis | best conditioning | best | primary fit segment |
| Withheld validation | not used for fit | validates residuals | acceptance evidence |

FK remains joint-feedback-derived delayed observation, not external Cartesian ground truth. The calibration is relative to the current FK/geometry/timing model and inherits those uncertainties.

## Proposed J9-E1B Hardware Trajectory

No J9-E1B motion is authorized by this document. When authorized later, the planned sequence is:

1. HOME stationary segment: gyro bias, J9-D timing diagnostics, no excitation.
2. Roll excitation: low-amplitude sinusoid/chirp within existing J8 software envelope.
3. Pitch excitation: same safety limits.
4. Yaw excitation: same safety limits.
5. Combined multi-axis excitation: rank/conditioning improvement.
6. Independent validation trajectory: withheld from fitting.

The trajectory must keep the existing J8 motion envelope, watchdog, Design-B feedback, safety gates, HOME/PARK rules, and hardware authorization gates unchanged.

## Calibration Order

The later calibration order is:

```text
Platform orientation
-> Platform lever-arm observability
-> lock platform relative pose
-> move/rotate the whole mechanism
-> Base IMU orientation calibration
-> Base IMU lever-arm observability
```

Do not infer base yaw or base lever arm while the base is stationary.
