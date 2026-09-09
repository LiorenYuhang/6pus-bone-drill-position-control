# J10-A9.1 Roll Operational Boundary Refinement

Date: 2026-09-09

## A. +50 deg audit

The original +50 deg run commanded `[1809,1000,306,306,1000,1809]`. At the
driver fatal event (`16:39:32.139`), the last feedback was approximately
`[1650,998,424,426,998,1645]`; ID4 therefore remained about 120 steps from
the 306-step target. ID4 had been moving before the event, but the final
feedback plateaued around 422-426 steps. The driver reported
`overcurrent_ID4` and latched STOP. The CSV records position/feedback but no
current telemetry, so current magnitude cannot be reconstructed from the
artifact. No direct mechanical interference evidence was recorded.

The +50 deg candidate never reached feedback target, established no
measurement window, and did not confirm HOME. It was not rerun.

## B. -30 deg audit and timeout correction

The original -30 deg run started candidate transition with stale non-HOME
feedback (about ID1/6 = 1530 and ID3/4 = 530) left by the failed +50 deg run.
Feedback continued toward the target at approximately 57-64 steps/s, but the
9.6 s timeout was computed as if the mechanism had started at HOME. The
failure was therefore an invalid precondition plus an optimistic distance
assumption, not a workspace limit or a demonstrated stall.

The J10 runner was minimally changed to require five fresh HOME-band samples
before candidate transition. The pre-candidate HOME timeout is derived from
the feedback actually observed. The existing candidate timeout model and the
J7/J8 trajectory/control path were unchanged.

## C. Hardware results

| Candidate | Predicted steps | Absolute margin | Candidate | Measurement | HOME | Qualification |
|---:|---|---:|---|---|---|---|
| Roll +30 deg | [1502,1000,533,533,1000,1502] | 498 (ID1/6) | 7.602 s | valid, 0 invalid | 7.189 s, confirmed | PASS |
| Roll +40 deg | [1660,1000,407,407,1000,1660] | 340 (ID1/6) | timeout at 12.5 s; `overcurrent_ID4` at 17:09:17.620 | not established | not confirmed | FAIL/STOP |
| Roll -30 deg (corrected run) | [549,1000,1486,1486,1000,549] | 514 (ID3/4) | 7.198 s | valid, 0 invalid | 7.955 s, confirmed | PASS |
| Roll -50 deg | [350,1000,1766,1766,1000,350] | 234 (ID3/4) | timeout at 14.267 s; `overcurrent_ID3` at 17:11:54.284 | not established | not confirmed | FAIL/STOP |

The corrected -30 deg run required 8.123 s to establish pre-candidate HOME
from the prior run's state, then reached target normally. Its measurement FK
median was `[-0.1396 mm, 0.1179 mm, 132.6097 mm, -29.9235 deg, 0.0659 deg,
0.0563 deg]`. Platform H30 Roll, computed from the stable post-HOME relative
zero using the existing Rz(+90 deg) transform, was approximately -29.45 deg.
The +30 deg run measured FK +29.9622 deg and Platform H30 +29.50 deg. These
are consistent in direction and magnitude; no H30 fusion or controller change
was made.

The +40 deg and -50 deg CSV traces show continued approach before their
overcurrent events, but neither candidate satisfied feedback-reached and
therefore neither has FK/H30 measurement-window data. No mechanical anomaly
was directly observed in the logged evidence.

Traffic counts were 353/2118 (UART cycles/0x29 frames) for +30 deg,
175/1050 for +40 deg, 435/2610 for corrected -30 deg, and 171/1026 for
-50 deg. The runs emitted the known post-run context-invalid teardown warning;
the runner metadata reports no separate motion-stop/fault-stop flag.

Artifacts:

- [corrected -30 metadata](../../results/J10_A9_1/J10_workspace_roll_neg300deg_20260909_170957_metadata.json)
- [+40 metadata](../../results/J10_A9_1/J10_workspace_roll_pos400deg_20260909_170902_metadata.json)
- [-50 metadata](../../results/J10_A9_1/J10_workspace_roll_neg500deg_20260909_171143_metadata.json)
- Previous [+30 metadata](../../results/J10_A9/J10_workspace_roll_pos300deg_20260909_163601_metadata.json)

## D. Boundary conclusion

- Maximum fully-qualified observed Roll+: **+30 deg**.
- First Roll+ operational failure: **+40 deg**, `overcurrent_ID4` with
  candidate timeout. +50 deg remains a separate prior failure and was not
  rerun.
- Maximum fully-qualified observed Roll-: **-30 deg**.
- First Roll- operational failure: **-50 deg**, `overcurrent_ID3` with
  candidate timeout.

The current Roll boundary classification is **operational failure limited
before the predicted absolute actuator endpoint**. The overcurrent events are
real operational failures, but are not by themselves proof of a mechanical
limit. The original -30 deg timeout was corrected as a pre-HOME sequencing
issue and subsequently passed.

J10 experiment-layer changes are limited to the pre-candidate HOME
confirmation and its unit test. `py_compile` passed and the J10 regression
suite passed (`26 passed`). No J7/J8/J9 production executable logic was
modified, and no commit was created.
