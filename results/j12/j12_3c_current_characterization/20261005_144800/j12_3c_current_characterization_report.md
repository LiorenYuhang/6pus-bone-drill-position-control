# J12-3C PARK ↔ HOME Dynamic Current Characterization Report

**Run Timestamp**: 2026-10-05T14:49:46.976439
**Target Directory**: `/home/lyapunov/bone_drill/results/j12/j12_3c_current_characterization/20261005_144800`

## 1. Post-Experiment Comparison Table

| Level | Speed | Peak measured velocity | Peak current | Axis | Phase | Max tracking error | Fault |
|---|---:|---:|---:|---|---|---:|---|
| Level 1 | 100.0 st/s | 722.7 st/s | 77 mA | ID5 | DECEL | 14 steps | NO |
| Level 2 | 200.0 st/s | 246.3 st/s | 119 mA | ID2 | CRUISE | 11 steps | NO |
| Level 3 | 300.0 st/s | 341.9 st/s | 161 mA | ID2 | CRUISE | 18 steps | NO |
| Level 4 | 400.0 st/s | 446.2 st/s | 184 mA | ID2 | CRUISE | 26 steps | NO |

## 2. Answers to the 8 Core Questions

1. **500 mA 是否在正常动态运行中被自然超过？**
   - 否 (实测最高峰值: 184 mA)
2. **800 mA 是否被超过？**
   - 否
3. **1000 mA 是否被超过？**
   - 否
4. **1200 mA 是否被超过？**
   - 否
5. **是否接近 1500 mA？**
   - 否
6. **电流峰值主要发生在：acceleration / cruise / deceleration / reversal？**
   - 实测峰值阶段分布: ['DECEL', 'CRUISE', 'CRUISE', 'CRUISE']
7. **最大合法速度下 tracking 是否仍正常？**
   - 正常 (Level 4 最大跟踪误差: 26 steps)
8. **六轴有没有任何 internal fault？**
   - 六轴完全无 internal fault (全部为0)

## 3. High Current Events (>500 mA)

Total events recorded: 0

No samples exceeded 500 mA during the entire characterization run.

## 4. Generated Artifact Figures

- [Figure 1: Current vs Time](figure_1_current_vs_time.png)
- [Figure 2: Current vs Velocity](figure_2_current_vs_velocity.png)
- [Figure 3: Tracking Error vs Velocity](figure_3_tracking_error_vs_velocity.png)
