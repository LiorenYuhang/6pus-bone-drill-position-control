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

The actuator value comes from `steps_per_meter: 40000.0` (`40 steps/mm`) and `meters_per_step: 0.000025`. The comparable J9.1 translation metric is the arithmetic mean of the last 100 action-stage FK samples minus the arithmetic mean of the last 100 pre-HOME FK samples. It is not a final sample or median. The comparable J9.1 attitude metric is the Platform-IMU arithmetic mean over the latter half of the action stage; `mean_error` is the signed reference-minus-measurement arithmetic mean over that same window, not a final sample, median, or controller residual.

## 3. Metric Audit Verdict

### X command and the historical J10 value

The J10 run did command an actual Cartesian X target increment of `+0.5 mm`. The tracked synthesis stores `summary_table[0].amplitude_mm = 0.5`. The underlying local metadata stores `target_amplitude = 0.5`, `target_units = "mm"`, and `plan.target_pose[0] = 0.0005 m`; the runner applies the signed amplitude to X and publishes that Cartesian pose.

The historical `+0.238466 mm` is a different statistic: local field `summary.candidate_delta_from_pre_home[0]`, calculated as the 412-sample legacy `candidate`-stage median FK X minus the 247-sample `pre_home`-stage median FK X. The tracked synthesis rounds it to `summary_table[0].fk_delta_x_mm = 0.238` and labels the run `legacy candidate stage; strict window unavailable`. It is not a final sample, endpoint displacement, or qualified steady-state mean. Therefore, `0.5 - 0.238466` must not be presented as directly comparable to J9.1 settled-window differences. The public comparable table instead uses the formal J9.1 X runs, whose metric matches Y and Z.

### Historical attitude-number discrepancies

- The matching repository values behind the historical Roll pair `approximately +0.4936 deg` / `approximately 0.0064 deg` are negative-Roll controller quantities `stage_metrics.roll_neg_closed_loop.max_cmd = -0.493579... deg` and `.max_residual = +0.006421... deg`. They are neither final nor median Platform-IMU measurements, and they do not form a settled response/error pair.
- Repository search found no negative-Pitch action-stage metric supporting a generic `approximately 0.0034 deg` tracking error. A nearby value belongs to the positive-run return-HOME condition (`stage_metrics.pitch_ret_home.mean_error = -0.003010... deg`), not negative-Pitch action tracking. The comparable negative-Pitch field is `stage_metrics.pitch_neg_closed_loop.mean_error = +0.021312... deg`; the exact origin of the historical statement is not established by authoritative evidence.

## 4. Verified Comparable Evidence Table

Translation differences below are absolute command-minus-FK-local-displacement magnitudes. Attitude errors preserve the signed command-minus-Platform-IMU convention stored in `mean_error`.

| Item | Command / condition | Measured result | Metric | Derived difference / signed error | Evidence / source field |
| --- | --- | --- | --- | --- | --- |
| Actuator position resolution | 40 steps/mm configuration mapping | 0.025 mm/step | Joint-command quantization | — | [`6pus_geometry.yaml`](../../bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml), `actuator.steps_per_meter`, `actuator.meters_per_step` |
| X translation | +0.5 mm | FK local X displacement +0.331142 mm | Last-100-sample action FK mean minus last-100-sample pre-HOME FK mean | 0.168858 mm absolute | [`j9_1_x_translation_pos05mm_20260903_203327_metadata.json`](../../results/j9_1/translation_characterization/j9_1_x_translation_pos05mm_20260903_203327_metadata.json), `target_dx_mm`, `metrics.fk_local_displacement.dx_mm` |
| X translation | -0.5 mm | FK local X displacement -0.399084 mm | Same last-100-sample mean difference | 0.100916 mm absolute | [`j9_1_x_translation_neg05mm_20260903_204626_metadata.json`](../../results/j9_1/translation_characterization/j9_1_x_translation_neg05mm_20260903_204626_metadata.json), `target_dx_mm`, `metrics.fk_local_displacement.dx_mm` |
| Y translation | +0.5 mm | FK local Y displacement +0.518360 mm | Same last-100-sample mean difference | 0.018360 mm absolute | [`j9_1_y_translation_pos05mm_20260904_101138_metadata.json`](../../results/j9_1/translation_characterization/j9_1_y_translation_pos05mm_20260904_101138_metadata.json), `target_dy_mm`, `metrics.fk_local_displacement.dy_mm` |
| Y translation | -0.5 mm | FK local Y displacement -0.495733 mm | Same last-100-sample mean difference | 0.004267 mm absolute | [`j9_1_y_translation_neg05mm_20260904_120930_metadata.json`](../../results/j9_1/translation_characterization/j9_1_y_translation_neg05mm_20260904_120930_metadata.json), `target_dy_mm`, `metrics.fk_local_displacement.dy_mm` |
| Z translation | +0.5 mm | FK local Z displacement +0.476926 mm | Same last-100-sample mean difference | 0.023074 mm absolute | [`j9_1_z_translation_pos05mm_20260904_123707_metadata.json`](../../results/j9_1/translation_characterization/j9_1_z_translation_pos05mm_20260904_123707_metadata.json), `target_dz_mm`, `metrics.fk_local_displacement.dz_mm` |
| Z translation | -0.5 mm | FK local Z displacement -0.501826 mm | Same last-100-sample mean difference | 0.001826 mm absolute | [`j9_1_z_translation_neg05mm_20260904_131517_metadata.json`](../../results/j9_1/translation_characterization/j9_1_z_translation_neg05mm_20260904_131517_metadata.json), `target_dz_mm`, `metrics.fk_local_displacement.dz_mm` |
| Roll attitude | +0.5 deg | Platform-IMU mean +0.448645 deg | Arithmetic mean over latter half of action stage | +0.051355 deg signed mean error | [`j9_1_roll_closed_loop_20260903_162702_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_roll_closed_loop_20260903_162702_metadata.json), `ref_angle_deg`, `stage_metrics.roll_pos_closed_loop.mean_plat_r`, `.mean_error` |
| Roll attitude | -0.5 deg | Platform-IMU mean -0.508347 deg | Same latter-half action-stage mean | +0.008347 deg signed mean error | [`j9_1_roll_closed_loop_20260903_172228_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_roll_closed_loop_20260903_172228_metadata.json), `ref_angle_deg`, `stage_metrics.roll_neg_closed_loop.mean_plat_r`, `.mean_error` |
| Pitch attitude | +0.5 deg | Platform-IMU mean +0.499518 deg | Same latter-half action-stage mean | +0.000482 deg signed mean error | [`j9_1_pitch_closed_loop_20260903_172948_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_pitch_closed_loop_20260903_172948_metadata.json), `ref_angle_deg`, `stage_metrics.pitch_pos_closed_loop.mean_plat_p`, `.mean_error` |
| Pitch attitude | -0.5 deg | Platform-IMU mean -0.521312 deg | Same latter-half action-stage mean | +0.021312 deg signed mean error | [`j9_1_pitch_closed_loop_20260903_173709_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_pitch_closed_loop_20260903_173709_metadata.json), `ref_angle_deg`, `stage_metrics.pitch_neg_closed_loop.mean_plat_p`, `.mean_error` |
| Yaw attitude | +2.0 deg | Platform-IMU mean +1.482501 deg | Same latter-half action-stage mean | +0.517499 deg signed mean error | [`j9_1_yaw_closed_loop_20260903_174906_metadata.json`](../../results/j9_1/closed_loop_attitude/j9_1_yaw_closed_loop_20260903_174906_metadata.json), `ref_angle_deg`, `stage_metrics.yaw_pos_closed_loop.mean_plat_y`, `.mean_error` |
| Whole-workspace absolute positioning accuracy | No dedicated external calibration | Unknown | Not measured | Not qualified | J10 explicitly limits its claim to independent single-axis operational-envelope characterization: [`J10_FINAL_SUMMARY.md`](../stages/J10/J10_FINAL_SUMMARY.md) |
| Repeatability | No systematic repeated-point statistics | Unknown | Not measured | Not qualified | No current repository evidence supports a sigma, 3-sigma, or equivalent repeatability value |

## 5. Cross-Axis Coupling

Cross-axis behavior prevents the primary-axis differences above from being treated as standalone Cartesian accuracy values. The individual-run values below use the same J9.1 last-100-sample mean-difference metric.

| Excitation | Observed cross-axis response | Interpretation | Evidence / source |
| --- | --- | --- | --- |
| X +0.5 mm | FK local delta Y +0.000332 mm; delta Z +0.001486 mm | Small cross-axis response in this run | [`j9_1_x_translation_pos05mm_20260903_203327_metadata.json`](../../results/j9_1/translation_characterization/j9_1_x_translation_pos05mm_20260903_203327_metadata.json) |
| X -0.5 mm | FK local delta Y -0.010279 mm; delta Z -0.003816 mm | Direction-dependent cross-axis response | [`j9_1_x_translation_neg05mm_20260903_204626_metadata.json`](../../results/j9_1/translation_characterization/j9_1_x_translation_neg05mm_20260903_204626_metadata.json) |
| Bidirectional X, frozen J9.1 audit | X-command column: X response 0.730227, Y response 0.010611, Z response 0.005303 mm/mm | Bidirectional aggregate of the same formal runs; diagonal dominance does not make it an external ground-truth Jacobian | [`j9_1_4e_xyz_translation_coupling_analysis.json`](../../results/j9_1/translation_coupling_analysis/j9_1_4e_xyz_translation_coupling_analysis.json); [`audit report`](./j9_1/j9_1_4e_xyz_translation_coupling_analysis.md) |
| Y +0.5 mm | FK delta X +0.035113 mm; FK delta Z +0.017834 mm | Non-zero translational and rotational cross-axis response | [`j9_1_y_translation_pos05mm_20260904_101138_metadata.json`](../../results/j9_1/translation_characterization/j9_1_y_translation_pos05mm_20260904_101138_metadata.json) |
| Z +0.5 mm | FK delta X -0.077737 mm; FK delta Y +0.195832 mm | Material cross-axis response; primary-axis error alone is incomplete | [`j9_1_z_translation_pos05mm_20260904_123707_metadata.json`](../../results/j9_1/translation_characterization/j9_1_z_translation_pos05mm_20260904_123707_metadata.json) |

The frozen J9.1 audit describes its FK matrix as actuator-feedback-based empirical geometry, not a ground-truth Jacobian. It also prohibits interpreting Platform-IMU XYZ channels as proven platform-center translational acceleration.

## 6. Engineering Interpretation

- X/Y/Z are directly comparable within the translation family because all six formal J9.1 runs use the same last-100-sample mean-difference definition. Their absolute command-response differences span approximately `0.0018-0.1689 mm`, but that spread and the cross-axis responses do not support a single local Cartesian accuracy number.
- The available Roll/Pitch settled-mean errors span approximately `0.0005-0.0514 deg`. This supports only a near-HOME small-angle tracking characterization with material axis/direction dependence, not a uniform `0.01 deg` attitude capability.
- The positive-Yaw signed mean tracking error of approximately `0.5175 deg` is materially larger than the Roll/Pitch results; a common three-axis attitude-accuracy statement would be misleading.
- The present evidence is useful for engineering comparison, controller development, and reproduction of the recorded command-to-feedback behavior. It is not sufficient for an absolute positioning or orientation specification.

## 7. Known Limitations

The current repository does not contain completed qualification for:

- external optical tracking calibration;
- laser-tracker or CMM-style absolute metrology;
- a whole-workspace absolute accuracy map;
- systematic repeated-point repeatability statistics;
- multi-axis Cartesian absolute-accuracy qualification;
- a whole-workspace 3DOF absolute orientation map.

Additional limitations include model dependence of FK, actuator quantization, direction-dependent behavior, cross-axis coupling, IMU frame/lever-arm uncertainty, host-time-based sensor pairing in J9.1, and non-identical protocols across J9.1 and J10. The best single result must not be generalized beyond its recorded condition.

## 8. Evidence Traceability

- **Actuator conversion**: [`bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml`](../../bone_ws/src/pus6_kinematics/config/6pus_geometry.yaml).
- **Comparable X translation**: formal metadata files linked in the table; [`j9_1_4e_xyz_translation_coupling_analysis.json`](../../results/j9_1/translation_coupling_analysis/j9_1_4e_xyz_translation_coupling_analysis.json) identifies `203327` and `204626` as frozen formal X artifacts.
- **Historical X J10 audit**: tracked synthesis [`results/j10/J10_A4_X_POS_LARGE_MOTION_20260909.json`](../../results/j10/J10_A4_X_POS_LARGE_MOTION_20260909.json), [`J10 A4 report`](./j10/J10_A4_X_POS_LARGE_MOTION_20260909.md), and [`J10_FINAL_SUMMARY.md`](../stages/J10/J10_FINAL_SUMMARY.md); underlying local legacy metadata remains unmodified.
- **Y/Z local translation**: the four positive/negative metadata files linked in the verified evidence table under [`results/j9_1/translation_characterization/`](../../results/j9_1/translation_characterization/).
- **Translation coupling**: [`results/j9_1/translation_coupling_analysis/j9_1_4e_xyz_translation_coupling_analysis.json`](../../results/j9_1/translation_coupling_analysis/j9_1_4e_xyz_translation_coupling_analysis.json) and [`agent_docs/reports/j9_1/j9_1_4e_xyz_translation_coupling_analysis.md`](./j9_1/j9_1_4e_xyz_translation_coupling_analysis.md).
- **Roll/Pitch/Yaw tracking**: the five settled-window metadata files linked in the verified evidence table under [`results/j9_1/closed_loop_attitude/`](../../results/j9_1/closed_loop_attitude/); each corresponding runner's `compute_stage_settled_metrics()` averages the latter half of the action stage.
- **Stage boundary and interpretation**: [`agent_docs/stages/J9/J9_FINAL_SUMMARY.md`](../stages/J9/J9_FINAL_SUMMARY.md), [`agent_docs/stages/J9/J9_TO_J10_HANDOFF.md`](../stages/J9/J9_TO_J10_HANDOFF.md), and [`agent_docs/stages/J10/J10_FINAL_SUMMARY.md`](../stages/J10/J10_FINAL_SUMMARY.md).

All cited tests are real-hardware evidence. No new hardware run, control-code change, or experimental-data modification was performed for this summary.
