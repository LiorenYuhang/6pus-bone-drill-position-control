# J9.1-4E Translational Coupling Audit

Scope: offline analysis of frozen formal +/-X, +/-Y, +/-Z artifacts only. No hardware execution.
FK matrices are actuator-feedback-based empirical geometric responses, not ground truth Jacobians.
Platform IMU matrices are measurement-point empirical dynamic responses, not platform-center plant matrices.

## Decision
C now, then hybrid control architecture: refine measurement/frame/lever-arm separation before committing controller structure; use FK low-frequency centering plus Platform dynamic feedback once measurement channels are separated; keep future Base IMU disturbance feedforward pending Base disturbance characterization.

## M_FK mm_per_mm
- x: cmd_x=0.7302, cmd_y=0.1036, cmd_z=-0.0518
- y: cmd_x=0.0106, cmd_y=1.0141, cmd_z=0.1642
- z: cmd_x=0.0053, cmd_y=0.0031, cmd_z=0.9788

## M_accel odd RMS m_s2
- x: cmd_x=0.0393, cmd_y=0.0570, cmd_z=0.1235
- y: cmd_x=0.0227, cmd_y=0.0365, cmd_z=0.0699
- z: cmd_x=0.0064, cmd_y=0.0088, cmd_z=0.0188

## M_gyro odd RMS deg_s
- gx: cmd_x=0.1414, cmd_y=0.2146, cmd_z=0.7972
- gy: cmd_x=0.1844, cmd_y=0.0989, cmd_z=0.4172
- gz: cmd_x=0.4907, cmd_y=0.7270, cmd_z=2.3272

## Notes
- M_FK diagonal terms dominate and indicate near-decoupled actuator-feedback-based low-frequency geometry.
- Platform IMU dynamic matrices show large off-axis acceleration responses, especially during Y and Z commands.
- Large acceleration cross-axis responses co-occur with large gyro responses, so they should not be directly interpreted as translational plant coupling before frame/lever-arm refinement.
- 3 independent SISO loops are not yet justified from Platform IMU dynamics; full 3DOF MIMO translational control is also premature because the measurement is not proven platform-center translational acceleration.
