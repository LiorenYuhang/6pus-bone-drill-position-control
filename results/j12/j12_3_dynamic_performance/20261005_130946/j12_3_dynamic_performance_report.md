# J12-3 Small-Angle Dynamic Performance Report

**Run Directory**: `20261005_130946`
**Dynamic Segments Evaluated**: 3

## 1. Dynamic Performance Summary Table

| Segment | Freq (Hz) | Peak Rate (°/s) | Base Amp (°) | Plat SO3 RMS (°) | Plat SO3 Peak (°) | Attenuation (dB) | P/D Sat Ratio (%) | Max Err (st) | Max Curr (mA) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Roll_Slow** | 0.00 | 17.2 | 15.10° | 1.99° | 4.41° | **-11.4 dB** | 0.0% | 36 st | 132 mA |
| **Roll_Medium** | 0.09 | 17.3 | 17.84° | 2.49° | 5.75° | **-12.1 dB** | 0.0% | 84 st | 174 mA |
| **Roll_Fast** | 0.15 | 34.3 | 16.88° | 9.72° | 19.24° | **+0.6 dB** | 34.0% | 111 st | 161 mA |

## 2. Dynamic Performance & Classical Bottleneck Audit

- **Max Base Angular Rate Tested**: 34.3°/s
- **Residual Saturation (±0.5°)**: FREQUENT SATURATION (>20% cycles)
- **Motor Current Boundary (400 mA / 500 mA)**: SAFE (All axes well below 400 mA)

## 3. Generated Figures

- [Figure 1: Base Attitude Progression](j12_3_figure_1_base_attitude_progression.png)
- [Figure 2: Base SO(3) Angular Rate Progression](j12_3_figure_2_base_angular_rate.png)
- [Figure 3: Platform Attitude Residuals](j12_3_figure_3_platform_attitude_residuals.png)
- [Figure 4: Base Rate vs Platform SO(3) Error](j12_3_figure_4_angular_rate_vs_so3_error.png)
- [Figure 5: PID Residuals & Saturation Boundary](j12_3_figure_5_pid_residuals_and_saturation.png)
- [Figure 6: Six-Axis Actuator Step Rates](j12_3_figure_6_actuator_step_rates.png)
- [Figure 7: Six-Axis Tracking Error vs Rate](j12_3_figure_7_tracking_error_vs_rate.png)
- [Figure 8: Motor Current vs Rate](j12_3_figure_8_motor_current_vs_rate.png)
- [Figure 9: Disturbance Rejection Comparison](j12_3_figure_9_disturbance_rejection_comparison.png)
