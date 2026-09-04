# J9.1-4F Platform IMU Lever-Arm Identifiability Audit

Scope: offline audit of frozen J9.1 attitude artifacts only. No hardware execution.
R_P_IP is frozen as Rz(+90 deg); Base IMU geometry is out of scope.

## Decision
NOT_RELIABLY_IDENTIFIABLE
Current frozen attitude datasets cannot separate Platform IMU lever-arm acceleration from Platform-origin translation because they lack acceleration telemetry. Candidate r_P_IP is not reliable and was not used to correct XYZ M_accel.

## Dataset Availability
- j9_1_open_loop_attitude_20260903_152326: usable_for_fit=no, accel=no, stages=roll_pos,roll_ret1,roll_neg,roll_ret2
- j9_1_open_loop_attitude_20260903_152450: usable_for_fit=no, accel=no, stages=roll_pos,roll_ret1,roll_neg,roll_ret2,pitch_pos,pitch_ret1,pitch_neg,pitch_ret2,yaw_pos,yaw_ret1,yaw_neg,yaw_ret2
- j9_1_open_loop_attitude_20260903_161349: usable_for_fit=no, accel=no, stages=yaw_pos,yaw_ret1,yaw_neg,yaw_ret2
- j9_1_3dof_orientation_servo_20260903_183743: usable_for_fit=no, accel=no, stages=roll_pos_closed_loop,roll_ret_home
- j9_1_3dof_orientation_servo_20260903_184719: usable_for_fit=no, accel=no, stages=pitch_pos_closed_loop,pitch_ret_home
- j9_1_3dof_orientation_servo_20260903_185428: usable_for_fit=no, accel=no, stages=yaw_pos_closed_loop,yaw_ret_home
- j9_1_pitch_closed_loop_20260903_172948: usable_for_fit=no, accel=no, stages=pitch_pos_closed_loop,pitch_ret_home
- j9_1_pitch_closed_loop_20260903_173709: usable_for_fit=no, accel=no, stages=pitch_neg_closed_loop,pitch_ret_home
- j9_1_roll_closed_loop_20260903_162702: usable_for_fit=no, accel=no, stages=roll_pos_closed_loop,roll_ret_home
- j9_1_roll_closed_loop_20260903_172228: usable_for_fit=no, accel=no, stages=roll_neg_closed_loop,roll_ret_home
- j9_1_yaw_closed_loop_20260903_174906: usable_for_fit=no, accel=no, stages=yaw_pos_closed_loop,yaw_ret_home
- j9_1_yaw_closed_loop_20260903_182631: usable_for_fit=no, accel=no, stages=yaw_neg_closed_loop,yaw_ret_home

## Estimates
No lever-arm least-squares estimate was accepted because b = a_I - a_P cannot be formed from these attitude artifacts.

## Next Step
Run a minimal dedicated pure-rotation characterization only after explicit hardware authorization: Roll +/-0.5 deg, Pitch +/-0.5 deg, Yaw +/-2 deg, logging Platform quaternion, gyro, raw accel, gravity-removed accel, stages, joint feedback, and FK. Then repeat identifiability before Platform-center acceleration reconstruction.
