# J12-2 Combined Quasi-Static Attitude Screening Report

**Run Directory**: `20261005_122117`
**Detected Plateaus**: 7

## 1. Case Screening Summary Table

| Plateau | Case / Nom Target | Base R/P/Y (°) | Base SO3 | Plat Err SO3 | ID1 (mA) | ID2 (mA) | ID3 (mA) | ID4 (mA) | ID5 (mA) | ID6 (mA) | Max Err (st) | Min Stroke |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| P1 | Baseline 0 | -0.0 / -0.0 / -0.0 | 0.0° | 0.01° | 0.0 | 0.0 | **0.0** | **0.0** | **0.0** | 0.0 | 2 | 998 |
| P2 | Case A (+, +, +) | +13.8 / +7.1 / -14.2 | 21.6° | 2.39° | 12.5 | 12.4 | **8.0** | **6.7** | **7.2** | 12.6 | 9 | 768 |
| P3 | Return 0A | +0.5 / -2.3 / -3.7 | 4.5° | 0.45° | 15.2 | 18.9 | **12.9** | **6.6** | **21.9** | 9.7 | 9 | 956 |
| P4 | Case B (+, -, -) | +12.3 / -12.7 / +7.0 | 19.5° | 0.66° | 8.5 | 0.0 | **5.6** | **2.3** | **1.7** | 2.3 | 5 | 701 |
| P5 | Case C (-, +, -) | -18.6 / +3.6 / -6.3 | 19.8° | 0.78° | 10.3 | 6.6 | **3.0** | **2.2** | **7.9** | 10.5 | 9 | 673 |
| P6 | Case D (-, -, +) | -15.5 / -12.2 / +5.6 | 20.0° | 1.01° | 5.5 | 4.3 | **6.0** | **3.6** | **5.0** | 7.6 | 6 | 660 |
| P7 | Return 0D | -0.2 / -4.3 / -5.6 | 7.1° | 0.88° | 3.6 | 1.1 | **15.4** | **0.4** | **0.0** | 1.2 | 3 | 934 |

## 2. Multi-Axis Kinematic Coupling & Load Qualification

- **Global Minimum Actuator Stroke in Plateaus**: 660 steps (margin to 500 threshold: +160 steps)
- **Current Safety Limit (500 mA)**: PASS (No axis exceeded 500 mA)
- **Current Warning Limit (400 mA)**: PASS (All axes below 400 mA)
- **Tracking Error Boundary (25 steps)**: PASS (All axes tracking tightly)

**Screening Verdict**: **PASS** - 3-DOF combined attitude envelope is kinematically clean, loads are evenly distributed, and no non-linear current surge or tracking saturation occurred.

## 3. Generated Figures

- [Figure 1: 3-DOF Base Attitude Progression](j12_2_figure_1_base_attitude_progression.png)
- [Figure 2: 3-DOF Platform Tracking Residual](j12_2_figure_2_platform_error_progression.png)
- [Figure 3: Six-Axis Actuator Feedback Steps](j12_2_figure_3_actuator_steps_progression.png)
- [Figure 4: Six-Axis Tracking Error Progression](j12_2_figure_4_tracking_error_progression.png)
- [Figure 5: Six-Axis Motor Current Progression](j12_2_figure_5_motor_current_progression.png)
- [Figure 6: Per-Case Steady-State Current Comparison](j12_2_figure_6_per_case_current_comparison.png)
- [Figure 7: Per-Case Actuator Stroke & Error Envelope](j12_2_figure_7_stroke_and_error_envelope.png)
