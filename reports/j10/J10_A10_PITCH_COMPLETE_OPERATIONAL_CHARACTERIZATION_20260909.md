# J10-A10 Pitch +/- Operational Characterization

Date: 2026-09-09

J10-only Pitch gate was extended to +/-55 deg for this experiment. The default
J7/J8/J9 +/-0.1 rad behavior was preserved; Yaw was not extended.

## Offline projection

| Pitch | Predicted steps (ID1..6) | Min absolute margin | Limiter | IK |
|---:|---|---:|---|---|
| +20 deg | [1187,1386,1194,818,633,811] | 614 | ID2 | valid |
| +35 deg | [1313,1664,1335,704,402,682] | 336 | ID2 | valid |
| +40 deg | [1351,1752,1379,672,338,644] | 248 | ID2 | valid |
| +45 deg | [1386,1837,1421,644,282,608] | 163 | ID2 | valid |
| +50 deg | [1419,1920,1462,619,235,576] | 80 | ID2 | valid |
| +55 deg | [1449,1999,1500,599,197,548] | 1 | ID2 | valid |
| +60 deg | [1476,2075,1537,584,170,524] | -75 | ID2 | valid, absolute LIMIT |
| -20 deg | [811,633,818,1194,1386,1187] | 614 | ID5 | valid |
| -30 deg | [723,473,739,1289,1573,1273] | 427 | ID5 | valid |
| -35 deg | [682,402,704,1335,1664,1313] | 336 | ID5 | valid |
| -40 deg | [644,338,672,1379,1752,1351] | 248 | ID5 | valid |
| -45 deg | [608,282,644,1421,1837,1386] | 163 | ID5 | valid |
| -50 deg | [576,235,619,1462,1920,1419] | 80 | ID5 | valid |
| -55 deg | [548,197,599,1500,1999,1449] | 1 | ID5 | valid |
| -60 deg | [524,170,584,1537,2075,1476] | -75 | ID5 | valid, absolute LIMIT |

## Hardware results

| Candidate | Result | Candidate / HOME | Margin / limiter | Traffic |
|---:|---|---|---|---:|
| +20 deg | PASS | 6.149 s / 6.080 s confirmed | 614 / ID2 | 368 cycles, 2208 frames |
| +35 deg | PASS | 10.474 s / 10.168 s confirmed | 336 / ID2 | 396 cycles, 2376 frames |
| +40 deg | FAIL/STOP | timeout 14.033 s; HOME unconfirmed | 248 / ID2 | 180 cycles, 1080 frames |
| -20 deg | PASS | 7.174 s / 6.469 s confirmed | 614 / ID5 | 391 cycles, 2346 frames |
| -30 deg | PASS | 8.645 s / 8.619 s confirmed | 427 / ID5 | 451 cycles, 2706 frames |
| -35 deg | FAIL/STOP | timeout 12.567 s; HOME unconfirmed | 336 / ID5 | 177 cycles, 1062 frames |

The +40 deg failure reported `overcurrent_ID5`; the candidate did not reach
feedback target and no measurement window was valid. The -35 deg failure
reported `overcurrent_ID6` with the same outcome. CSV artifacts contain no
current telemetry; current evidence is from the driver fatal logs. Neither
event supplied direct mechanical interference evidence.

Valid FK measurement medians and H30 relative Roll/Pitch/Yaw medians were:

- +20 deg: FK Pitch `+19.968 deg`; H30 Pitch `+19.628 deg`.
- +35 deg: FK Pitch `+34.925 deg`; H30 Pitch `+33.388 deg`.
- -20 deg: FK Pitch `-19.953 deg`; H30 Pitch `-19.999 deg`.
- -30 deg: FK Pitch `-29.935 deg`; H30 Pitch `-29.364 deg`.

Translations and the other attitude channels remained small in the valid
windows (largest reported translational coupling was below 0.23 mm). The
FK/H30 differences are calibration/frame discrepancies, not qualification
limits. No IMU fusion, lever-arm refinement, or controller tuning was used.

Artifacts are in `results/J10_A10/`:

- `J10_workspace_pitch_pos200deg_20260909_183347_{metadata.json,csv}`
- `J10_workspace_pitch_pos350deg_20260909_183651_{metadata.json,csv}`
- `J10_workspace_pitch_pos400deg_20260909_183858_{metadata.json,csv}`
- `J10_workspace_pitch_neg200deg_20260909_184155_{metadata.json,csv}`
- `J10_workspace_pitch_neg300deg_20260909_185919_{metadata.json,csv}`
- `J10_workspace_pitch_neg350deg_20260909_185049_{metadata.json,csv}`

## Boundary conclusion

- Pitch+ last fully-qualified PASS: **+35 deg**.
- Pitch+ first observed FAIL: **+40 deg**, `overcurrent_ID5`.
- Pitch+ operational bracket: **+35 to +40 deg**.
- Pitch- last fully-qualified PASS: **-30 deg**.
- Pitch- first observed FAIL: **-35 deg**, `overcurrent_ID6`.
- Pitch- operational bracket: **-30 to -35 deg**.

The Pitch boundaries are operational current/load failures before the predicted
absolute actuator endpoint. They are not actuator stroke limits and are not
direct proof of mechanical limits. `py_compile` passed and the J10 regression
suite passed (`26 passed`). No J7/J8/J9 production behavior was modified and
no commit was created.
