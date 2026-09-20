# 6PUS Real-Hardware Positioning and Attitude Tracking Performance Summary

## 1. Scope

This report consolidates the currently available real-hardware evidence for positioning and attitude tracking near `OPERATING_HOME = [1000]*6`. Its scope is deliberately limited to small-range, primarily single-axis tests using:

- Cartesian command to actuator feedback to numerical forward kinematics (FK); or
- attitude command to Platform-IMU feedback.

It is not an absolute-accuracy qualification. No result in this report is a whole-workspace robot specification.

## 2. Measurement Interpretation

The following distinctions are mandatory when reading the evidence:

1. `1 step = 0.025 mm` is the actuator/joint position quantization defined by the current geometry configuration. It is not TCP Cartesian positioning accuracy, repeatability, or external-metrology accuracy.
2. A command-to-FK difference compares a software command with a pose reconstructed from the same mechanism's actuator feedback and kinematic model. It is not independent external-metrology absolute error.
3. An IMU attitude tracking error compares an attitude command with Platform-IMU feedback in a particular near-HOME run. It is not whole-workspace 3DOF absolute orientation accuracy.
4. A single favorable run or direction must not be promoted to a robot-level specification. Bidirectional results and cross-axis responses are materially different in the current evidence.

The X `+0.5 mm` result requires an additional traceability note. The tracked J10 synthesis records FK `+0.238 mm`; its underlying local metadata records `+0.238466 mm` and is ignored by the repository's current evidence-retention policy. The public number is therefore reported as approximately `+0.2385 mm`, with the tracked synthesis as the durable repository reference.

## 3. Verified Evidence Table

Differences below are absolute command-minus-measurement magnitudes calculated from the cited settled-window or FK-displacement values.

| Item | Command / condition | Measured result | Difference / performance | Current interpretation | Evidence / source |
| --- | --- | --- | --- | --- | --- |
| Actuator position resolution | 1 step | 0.025 mm | — | Actuator/joint quantization only; not TCP accuracy | [`6pus_geometry.yaml`](../../bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml) (`1 mm = 40 steps`) |
| X translation | +0.5 mm | FK approximately +0.2385 mm | approximately 0.2615 mm | Local command-to-FK result; not an absolute-position measurement | [`J10_A4_X_POS_LARGE_MOTION_20260909.json`](../../results/j10/J10_A4_X_POS_LARGE_MOTION_20260909.json), `summary_table[0]`; underlying local `results/j10_a2/j10_a2_workspace_x_pos05mm_20260908_122831_metadata.json` |
| Y translation | +0.5 mm | FK +0.518360 mm | 0.018360 mm | Good local primary-axis tracking in this run | [`j9_1_y_translation_pos05mm_20260904_101138_metadata.json`](../../results/j9_1/translation_characterization/j9_1_y_translation_pos05mm_20260904_101138_metadata.json) |
| Y translation | -0.5 mm | FK -0.495733 mm | 0.004267 mm | Good local primary-axis tracking in this run | [`j9_1_y_translation_neg05mm_20260904_120930_metadata.json`](../../results/j9_1/translation_characterization/j9_1_y_translation_neg05mm_20260904_120930_metadata.json) |
| Z translation | +0.5 mm | FK +0.476926 mm | 0.023074 mm | Good local primary-axis tracking, with material cross-axis response | [`j9_1_z_translation_pos05mm_20260904_123707_metadata.json`](../../results/j9_1/translation_characterization/j9_1_z_translation_pos05mm_20260904_123707_metadata.json) |
| Z translation | -0.5 mm | FK -0.501826 mm | 0.001826 mm | Good local primary-axis tracking in this run | [`j9_1_z_translation_neg05mm_20260904_131517_metadata.json`](../../results/j9_1/translation_characterization/j9_1_z_translation_neg05mm_20260904_131517_metadata.json) |
| Roll attitude | +0.5 deg | Platform-IMU settled mean +0.448645 deg | mean error 0.051355 deg | Near-HOME small-angle tracking; direction-dependent | [`j9_1_roll_closed_loop_20260903_162702_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_roll_closed_loop_20260903_162702_metadata.json) |
| Roll attitude | -0.5 deg | Platform-IMU settled mean -0.508347 deg | mean error 0.008347 deg | Near-HOME small-angle tracking; direction-dependent | [`j9_1_roll_closed_loop_20260903_172228_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_roll_closed_loop_20260903_172228_metadata.json) |
| Pitch attitude | +0.5 deg | Platform-IMU settled mean +0.499518 deg | mean error 0.000482 deg | Favorable single-run result; not a system specification | [`j9_1_pitch_closed_loop_20260903_172948_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_pitch_closed_loop_20260903_172948_metadata.json) |
| Pitch attitude | -0.5 deg | Platform-IMU settled mean -0.521312 deg | mean error 0.021312 deg | Near-HOME small-angle tracking; asymmetric with the positive run | [`j9_1_pitch_closed_loop_20260903_173709_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_pitch_closed_loop_20260903_173709_metadata.json) |
| Yaw attitude | +2.0 deg | Platform-IMU settled mean +1.482501 deg | mean residual 0.517499 deg | Materially weaker than Roll/Pitch in the available evidence | [`j9_1_yaw_closed_loop_20260903_174906_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_yaw_closed_loop_20260903_174906_metadata.json) |
| Whole-workspace absolute positioning accuracy | No dedicated external calibration | Unknown | Not qualified | J10 explicitly limits its claim to independent single-axis operational-envelope characterization: [`J10_FINAL_SUMMARY.md`](../stages/J10/J10_FINAL_SUMMARY.md) |
| Repeatability | No systematic repeated-point statistics | Unknown | Not qualified | No current repository evidence supports a sigma, 3-sigma, or equivalent repeatability value |

## 4. Cross-Axis Coupling

Cross-axis behavior prevents the primary-axis differences above from being treated as standalone Cartesian accuracy values.

| Excitation | Observed cross-axis response | Interpretation | Evidence / source |
| --- | --- | --- | --- |
| X +0.5 mm, J10 run | FK delta Y approximately -0.098736 mm; FK delta Z approximately -0.001472 mm | Run-specific coupling from the underlying J10 metadata; the durable tracked J10 synthesis retains only the primary X result | Local `results/j10_a2/j10_a2_workspace_x_pos05mm_20260908_122831_metadata.json`; tracked synthesis: [`J10_A4_X_POS_LARGE_MOTION_20260909.json`](../../results/j10/J10_A4_X_POS_LARGE_MOTION_20260909.json) |
| Bidirectional X, frozen J9.1 audit | X-command column: X response 0.730227, Y response 0.010611, Z response 0.005303 mm/mm | Frozen bidirectional aggregate is diagonal-dominant but differs from the individual J10 run, demonstrating run/protocol dependence | [`j9_1_4e_xyz_translation_coupling_analysis.json`](../../results/j9_1/translation_coupling_analysis/j9_1_4e_xyz_translation_coupling_analysis.json); [`audit report`](./j9_1/j9_1_4e_xyz_translation_coupling_analysis.md) |
| Y +0.5 mm | FK delta X +0.035113 mm; FK delta Z +0.017834 mm | Non-zero translational and rotational cross-axis response | [`j9_1_y_translation_pos05mm_20260904_101138_metadata.json`](../../results/j9_1/translation_characterization/j9_1_y_translation_pos05mm_20260904_101138_metadata.json) |
| Z +0.5 mm | FK delta X -0.077737 mm; FK delta Y +0.195832 mm | Material cross-axis response; primary-axis error alone is incomplete | [`j9_1_z_translation_pos05mm_20260904_123707_metadata.json`](../../results/j9_1/translation_characterization/j9_1_z_translation_pos05mm_20260904_123707_metadata.json) |

The frozen J9.1 audit describes its FK matrix as actuator-feedback-based empirical geometry, not a ground-truth Jacobian. It also prohibits interpreting Platform-IMU XYZ channels as proven platform-center translational acceleration.

## 5. Engineering Interpretation

- The representative primary-axis translation differences in the table span approximately `0.0018-0.2615 mm`. The spread and the observed cross-axis responses do not support a single local Cartesian accuracy number.
- The available Roll/Pitch settled-mean errors span approximately `0.0005-0.0514 deg`. This supports only a near-HOME small-angle tracking characterization with material axis/direction dependence, not a uniform `0.01 deg` attitude capability.
- The positive-Yaw residual of approximately `0.5175 deg` is materially larger than the Roll/Pitch results; a common three-axis attitude-accuracy statement would be misleading.
- The present evidence is useful for engineering comparison, controller development, and reproduction of the recorded command-to-feedback behavior. It is not sufficient for an absolute positioning or orientation specification.

## 6. Known Limitations

The current repository does not contain completed qualification for:

- external optical tracking calibration;
- laser-tracker or CMM-style absolute metrology;
- a whole-workspace absolute accuracy map;
- systematic repeated-point repeatability statistics;
- multi-axis Cartesian absolute-accuracy qualification;
- a whole-workspace 3DOF absolute orientation map.

Additional limitations include model dependence of FK, actuator quantization, direction-dependent behavior, cross-axis coupling, IMU frame/lever-arm uncertainty, host-time-based sensor pairing in J9.1, and non-identical protocols across J9.1 and J10. The best single result must not be generalized beyond its recorded condition.

## 7. Evidence Traceability

- **Actuator conversion**: [`bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml`](../../bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml).
- **X local translation**: tracked synthesis [`results/j10/J10_A4_X_POS_LARGE_MOTION_20260909.json`](../../results/j10/J10_A4_X_POS_LARGE_MOTION_20260909.json); underlying local metadata `results/j10_a2/j10_a2_workspace_x_pos05mm_20260908_122831_metadata.json`.
- **Y/Z local translation**: the four positive/negative metadata files linked in the verified evidence table under [`results/j9_1/translation_characterization/`](../../results/j9_1/translation_characterization/).
- **Translation coupling**: [`results/j9_1/translation_coupling_analysis/j9_1_4e_xyz_translation_coupling_analysis.json`](../../results/j9_1/translation_coupling_analysis/j9_1_4e_xyz_translation_coupling_analysis.json) and [`agent_docs/reports/j9_1/j9_1_4e_xyz_translation_coupling_analysis.md`](./j9_1/j9_1_4e_xyz_translation_coupling_analysis.md).
- **Roll/Pitch/Yaw tracking**: the five settled-window metadata files linked in the verified evidence table under [`results/j9_1/closed_loop_attitude/`](../../results/j9_1/closed_loop_attitude/).
- **Stage boundary and interpretation**: [`agent_docs/stages/J9/J9_FINAL_SUMMARY.md`](../stages/J9/J9_FINAL_SUMMARY.md), [`agent_docs/stages/J9/J9_TO_J10_HANDOFF.md`](../stages/J9/J9_TO_J10_HANDOFF.md), and [`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](../stages/J10/J10_FINAL_SUMMARY.md).

All cited tests are real-hardware evidence. No new hardware run, control-code change, or experimental-data modification was performed for this summary.
