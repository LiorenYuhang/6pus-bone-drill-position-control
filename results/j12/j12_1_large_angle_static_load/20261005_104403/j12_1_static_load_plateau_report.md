# J12-1 Large-Angle Static Load Boundary Qualification Report

**Run Date**: 20261005_104403
**Detected Plateaus**: 4

## 1. Plateau Summary Table

| Plateau | Nom Base Roll | Dur (s) | Cmd Roll (°) | ID1 curr (mA) | ID2 curr (mA) | ID3 curr (mA) | ID4 curr (mA) | ID5 curr (mA) | ID6 curr (mA) | Max Err (steps) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| P1 | +0.0° | 2.8s | -0.0° | 0.0 | 0.0 | **1.2** | **0.0** | **0.0** | 0.0 | 2 |
| P2 | -25.8° | 2.3s | +25.7° | 2.6 | 6.5 | **11.2** | **0.9** | **14.2** | 13.4 | 6 |
| P3 | -25.2° | 2.4s | +25.2° | 4.3 | 3.0 | **14.7** | **2.6** | **3.8** | 7.7 | 5 |
| P4 | +1.9° | 7.1s | -1.9° | 8.5 | 4.0 | **6.5** | **7.1** | **4.3** | 6.8 | 45 |

## 2. Multi-Axis Current Escalation & Inflection Analysis

- **Roll Progression**: [0.0, -25.8, -25.2, 1.9]
- **ID5 Current Trend**: [0.0, 14.201754385964913, 3.768595041322314, 4.264788732394366] mA
- **ID3 Current Trend**: [1.1830985915492958, 11.219298245614034, 14.735537190082646, 6.473239436619719] mA
- **Inflection Verdict**: **APPROXIMATELY SMOOTH / LINEAR TREND**.

## 3. Generated Figures

- [Figure A: Roll Angle vs Actuator Position](j12_1_figure_a_roll_vs_actuator_position.png)
- [Figure B: Roll Angle vs Steady-State Current](j12_1_figure_b_roll_vs_steady_state_current.png)
- [Figure C: Roll Angle vs Tracking Error](j12_1_figure_c_roll_vs_tracking_error.png)
