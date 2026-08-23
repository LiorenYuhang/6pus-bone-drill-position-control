# J6 Post-Assembly Commissioning Log

This is the operator record for the J6 post-assembly sequence. Complete one
copy per commissioning session and attach the immutable JSON/CSV evidence
files. This template does not replace the authoritative procedure in
`agent_docs/stages/J6/6PUS_POST_ASSEMBLY_COMMISSIONING_PLAN.md`.

## Session

| Field | Value |
| --- | --- |
| Session ID |  |
| Date/time (local and UTC) |  |
| Operator |  |
| Reviewer / sign-off |  |
| Hardware serials / assembly ID |  |
| Firmware / software revision |  |
| Frozen J5 tag | `v1.0-j5-pre-assembly-frozen` |
| Host / ROS 2 environment |  |
| E-stop tested before work | `PENDING` |
| 24 V supply current limit |  |
| Evidence directory |  |

### Global safety declaration

- [ ] No J4 continuous 6DoF trajectory was run before the required gates.
- [ ] No unapproved ACTIVE hardware command was run.
- [ ] Every write-capable action had an operator, an E-stop holder, and a
      recorded authorization.
- [ ] Mock/read-only evidence is not represented as physical-mechanism PASS.
- [ ] `6pus_geometry.yaml` was not changed until Phase 6 and only from measured
      data.
- [ ] J5-B4, ESKF, and Dual-IMU/FK closed-loop work remained disabled through
      Phase 8.

## Phase records

Use one block for each phase. A phase is not PASS until its evidence paths and
reviewer sign-off are complete. `N/A` is not a PASS substitute.

### Phase 0: Mechanical Static Inspection

**Entry conditions**

- Complete mechanism assembled: base, platform, six rods, joints, and guides.
- 24 V physically disconnected; RS485 disconnected from the host.

**Safety restrictions**

- No electrical power or motor energization.
- Only gentle manual movement; no prying or overload of joints.

**Measurements / observations**

| Item | Value / observation | Instrument / uncertainty |
| --- | --- | --- |
| Rod lengths `L0_1..L0_6` (mm) |  |  |
| Rod length range (mm) |  |  |
| Base radius `c` (mm) |  |  |
| Platform radius `d` (mm) |  |  |
| Slider lower/upper travel |  |  |
| Joint freedom / play |  |  |
| Fastener and base rigidity |  |  |
| Clearance observations |  |  |

**Evidence**

- Checklist / signed record: `________________`
- Photos or measurement file: `________________`

**PASS criteria**

- No binding, dead spots, abnormal friction, loose structure, or interference.
- Six rods and joint condition meet the measured baseline; the `0.05 mm` rod
  range is a design target, not an unmeasured fact.

**Gate**: `PENDING`  **Reviewer**: `____________`  **Timestamp**: `____________`

### Phase 1: Electrical and UART READ_ONLY Audit

**Entry conditions**: Phase 0 PASS; protected 24 V supply and E-stop ready;
RS485 by-ID path identified.

**Safety restrictions**: READ_ONLY only; no `0x29`, `0x1A`, `0x25`, HOME,
PARK, J3, or J4 command.

**Evidence**

- Connection report JSON: `________________`
- Phase 1 wrapper JSON/log: `________________`
- Supply voltage/current record: `________________`
- Six-axis snapshots / fault and timeout summary: `________________`

**PASS criteria**

- Six axes online for the required observation interval.
- Fault codes are zero; communication failures/timeouts meet the procedure.
- Static current and temperature are recorded and within the approved baseline.
- `hardware_mode == read_only` and lifecycle `hardware_write_count == 0`.

**Gate**: `PENDING`  **Reviewer**: `____________`  **Timestamp**: `____________`

### Phase 2: Axis ID, Feedback, and Zero Mapping

**Entry conditions**: Phase 1 PASS; mechanism stationary near the low parking
region; E-stop ready.

**Safety restrictions**: Read-only queries only; no motion or multi-axis command.

**Evidence**

- Physical ID/topology drawing: `________________`
- Static feedback JSON: `________________`
- `q_bottom_1..q_bottom_6` record: `________________`
- Mapping review: `________________`

**PASS criteria**

- ID1..ID6 physical positions match geometry anchor order.
- Bottom-position readings and zero mapping are measured and reviewed.

**Gate**: `PENDING`  **Reviewer**: `____________`  **Timestamp**: `____________`

### Phase 3: Single-Axis Micro-Jog Direction and Clearance

**Entry conditions**: Phase 2 PASS; no load or external interference above the
platform; E-stop held by a dedicated operator.

**Safety restrictions**

- One axis at a time; signed displacement `20..40` steps (`0.5..1.0 mm`).
- Speed exactly `50 step/s`; no full-range HOME; no multi-axis motion.
- Stop immediately for reverse direction, binding, current spike, or fault.

**Evidence**

- One JSON report per axis: `________________`
- Direction observation (`+Z` / reverse): `________________`
- Clearance and joint observation: `________________`
- Return error and peak-current record: `________________`

**PASS criteria**

- Command increase produces physical `+Z` motion and monotonic feedback.
- Return residual is at most one step; all faults remain zero.
- Other five axes receive zero writes and show no abnormal response.
- Mechanical clearance is explicitly observed and recorded.

**Gate**: `PENDING`  **Reviewer**: `____________`  **Timestamp**: `____________`

### Phase 4: PARK/HOME Reachability and Baseline Homing

**Entry conditions**: Phase 3 PASS for all six axes; no platform load; two-person
observation with E-stop.

**Safety restrictions**: `50 step/s` only; do not skip direction confirmation;
raw PARK requires the approved maintenance path.

**Evidence**

- HOME/PARK runner JSON/log: `________________`
- `q_home_1..q_home_6`, `q_park_1..q_park_6`: `________________`
- Height, level, current, and minimum-clearance measurements: `________________`

**PASS criteria**

- HOME settles in the procedure's approved `[998,1002]` step target band.
- PARK/HOME are mechanically reachable without interference or dead-stop contact.
- Current, level, and clearance observations meet approved targets or recorded
  measured baselines.

**Gate**: `PENDING`  **Reviewer**: `____________`  **Timestamp**: `____________`

### Phase 5: Coordinated Small Cartesian Motion

**Entry conditions**: Phase 4 PASS; platform at `OPERATING_HOME`; 24 V limit set
per the procedure.

**Safety restrictions**: low-speed single-DOF tests only; no full-range or J4
F1E trajectory; return to HOME after every test.

**Evidence**

- J3 Cartesian logs: `________________`
- Command/feedback waveforms: `________________`
- Current, synchronization, level, and direction observations: `________________`

**PASS criteria**

- Heave, Surge, Sway, Pitch, and Roll small motions are coordinated and stable.
- Return error, peak current, and preflight-intercept counts meet the approved
  criteria; no interference or abnormal vibration occurs.

**Gate**: `PENDING`  **Reviewer**: `____________`  **Timestamp**: `____________`

### Phase 6: Final Geometry Measurement and YAML Freeze

**Entry conditions**: Phase 5 PASS; mechanism powered off and fixed on a stable
measurement platform; calibrated measurement tools available.

**Safety restrictions**

- Use repeated physical measurements, not CAD guesses.
- Update the authoritative geometry only after measurement review.
- Do not advance until workspace tests and IK/FK regression pass.

**Evidence**

- Raw measurement table and uncertainty analysis: `________________`
- Reviewed geometry change record: `________________`
- Updated configuration hash / review: `________________`
- `pytest bone_ws/src -v` log: `________________`

**PASS criteria**

- All assembly-dependent dimensions are measured, repeatable, and reviewed.
- Configuration status reflects post-assembly confirmation.
- Full workspace regression and 1000-sample numerical IK/FK checks pass.

**Gate**: `PENDING`  **Reviewer**: `____________`  **Timestamp**: `____________`

### Phase 7: J5-C2-HW Real-Mechanism FK Acceptance

**Entry conditions**: Phase 6 PASS; external high-accuracy platform reference
installed; FK node has an independent `SingleThreadedExecutor`.

**Safety restrictions**

- FK is observation-only and cannot feed J4 or any motion control loop.
- Numerical residual alone cannot establish physical accuracy.
- Do not alter frozen FK algorithms or jump gates during acceptance.

**Evidence**

- Timed feedback recording: `________________`
- J5-C2-HW JSON/CSV report: `________________`
- External ground-truth measurements: `________________`
- FK-vs-ground-truth residual analysis: `________________`

**PASS criteria**

- Fixed-Lag synchronization, FK convergence, branch continuity, and residual
  thresholds satisfy the approved procedure.
- External translation and orientation errors meet the reviewed engineering
  target, with uncertainty documented.
- Acceptance report is signed with `hardware_acceptance_passed == true`.

**Gate**: `PENDING`  **Reviewer**: `____________`  **Timestamp**: `____________`

### Phase 8: Final Dual-IMU Installation, B2 Extrinsics, and Lever Arm

**Entry conditions**: Phase 7 signed PASS; sensors removed from same-board fixture;
final base/platform brackets available.

**Safety restrictions**

- Do not reuse same-board calibration as final mechanism calibration.
- Do not start B4, ESKF, or closed-loop fusion.
- Measure lever arm before using IMU angular rates for dynamic pose updates.

**Evidence**

- Final mounting photographs and fixture record: `________________`
- Lever-arm measurements and uncertainty: `________________`
- Final bias/frequency and extrinsic calibration output: `________________`
- IMU/FK residual curves: `________________`

**PASS criteria**

- Both sensors are rigidly mounted and coordinate frames are documented.
- Lever arm and final extrinsics are measured, recorded, and reviewed.
- Time synchronization is stable; slow tilt IMU/FK trend comparison is coherent.

**Gate**: `PENDING`  **Reviewer**: `____________`  **Timestamp**: `____________`

## Session disposition

| Result | Value |
| --- | --- |
| Highest completed phase |  |
| Overall disposition | `NOT_STARTED / HOLD / PASS / FAIL` |
| Open deviations |  |
| Corrective action owner/date |  |
| Final reviewer signature |  |

