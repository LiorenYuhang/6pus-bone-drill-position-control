# J6 Phase Gate Checklist

This checklist is the compact gate-control companion to the post-assembly
commissioning plan. Gates are strictly sequential. A phase may be marked PASS
only when its entry conditions, safety controls, evidence, and PASS criteria are
all satisfied and signed.

| Phase | Entry condition | Safety restriction | Required evidence | PASS gate |
| --- | --- | --- | --- | --- |
| 0 Mechanical static | Complete mechanical assembly; 24 V and RS485 disconnected | No power, no motor energization, gentle manual checks only | Rod/anchor/travel measurements, joint/rigidity checklist, photos, signed record | No binding/interference/loose structure; measured mechanical baseline accepted |
| 1 Electrical/UART READ_ONLY | Phase 0 PASS; current-limited 24 V; E-stop ready; by-ID path known | READ_ONLY only; no `0x25`, `0x29`, `0x1A`, HOME, PARK, J3, or J4 | Connection report, 30 s READ_ONLY report, supply current, six-axis telemetry | All six axes online; faults/timeouts/current/temperature acceptable; total writes exactly zero |
| 2 ID/feedback/zero | Phase 1 PASS; mechanism stationary near low position | Read-only queries; no motion or multi-axis writes | ID topology map, static feedback, measured `q_bottom`, mapping review | ID1..ID6 match physical anchor order and measured zero mapping is accepted |
| 3 Single-axis micro-jog | Phase 2 PASS; no load/interference; E-stop operator present | One axis only; `20..40` steps; `50 step/s`; no full HOME | Per-axis JSON, direction/clearance observation, residual/current/fault data | Physical `+Z` direction and monotonic feedback confirmed; return <= 1 step; observers untouched |
| 4 PARK/HOME | Phase 3 PASS for all six axes; two-person observation | `50 step/s`; no skipped direction gate; maintenance authorization for raw PARK | HOME/PARK logs, final positions, height/level/clearance/current records | HOME/PARK mechanically reachable, settled, level, and interference-free |
| 5 Coordinated small motion | Phase 4 PASS; platform at HOME; approved supply limit | Low-speed small DOF tests only; no full-range/J4 dynamic trajectory | J3 logs, command/feedback waveforms, current and coordination records | Small motions stable, coordinated, return accurately, with zero safety violations |
| 6 Geometry freeze | Phase 5 PASS; powered off on measurement platform; calibrated tools | Measured data only; no CAD guesses; regression before advance | Raw repeated measurements, uncertainty analysis, updated config review/hash, pytest log | Post-assembly geometry confirmed; full regression and 1000-sample IK/FK pass |
| 7 Real FK | Phase 6 PASS; external truth instrument; isolated FK executor | Observation-only FK; no J4 feedback; no algorithm/gate changes | Timed feedback, FK JSON/CSV, external ground truth, residual analysis | Sync/FK/branch gates and external physical accuracy pass; signed acceptance true |
| 8 Final IMU/B2 | Phase 7 signed PASS; final brackets; sensors removed from fixture | No same-board substitution; no B4/ESKF/closed-loop fusion; lever arm first | Mounting record, lever arm/uncertainty, final extrinsics/bias, IMU/FK curves | Rigid installation, measured lever arm/extrinsics, stable sync, coherent slow-tilt comparison |

## Universal stop rules

- Any unexpected motion, wrong direction, binding, interference, abnormal current,
  nonzero fault, communication loss, or failed stop confirmation: stop the test,
  use the E-stop, remove power as required, and record the failure.
- Never bypass a failed gate by changing limits, geometry, or mode in an
  unreviewed source file.
- Never convert Mock, synthetic, replay, or same-board evidence into a physical
  mechanism claim.

## Sign-off block

| Session | Highest PASS phase | Open deviations | Operator | Reviewer | Date |
| --- | --- | --- | --- | --- | --- |
|  |  |  |  |  |  |

