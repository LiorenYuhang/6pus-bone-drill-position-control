# J6 Phase 0 First Hardware Inspection Record

This is the operator record for the first physical commissioning session after
the J6 assembly is complete. Complete one copy per session. Record measured
observations and attach the referenced photographs or measurement files.

Phase 0 is a **no-power, no-motion gate**. It does not authorize 24 V power,
motor communication, encoder reads, motor writes, HOME/PARK, or any Cartesian
motion. Encoder-based zero mapping is performed only after the later READ_ONLY
gate. Final IMU extrinsics and lever-arm acceptance remain a later commissioning
phase.

## 1. Session and authorization

| Field | Entry |
| --- | --- |
| Session ID |  |
| Date / local time |  |
| UTC time |  |
| Assembly ID / hardware serials |  |
| Operator |  |
| Safety observer / E-stop holder |  |
| Reviewer |  |
| Work area / fixture |  |
| Ambient temperature |  |
| Evidence directory / record link |  |
| Overall status (`PENDING / PASS / FAIL / HOLD`) |  |

### Phase 0 entry conditions

- [ ] Complete mechanical assembly is present: base, six actuators, six rods,
      joints, guides, platform, and required retainers.
- [ ] 24 V power is physically disconnected from the actuators and cannot be
      energized accidentally during this inspection.
- [ ] Host, USB-to-RS485 adapter, and RS485 cable are disconnected from the
      mechanism/bus.
- [ ] Platform is unloaded and the work area is clear of tools and loose parts.
- [ ] No hardware command or commissioning script has been run for this record.

**Entry gate:** `PASS / FAIL / HOLD` __________  **Initials:** __________

## 2. Mechanical inspection

Use only visual checks, calibrated hand measurement, and gentle manual motion.
Do not pry, preload, or force a joint or slider. Mark each item `PASS`, `FAIL`,
or `N/A` only with a written explanation for `N/A`.

### 2.1 Assembly, fasteners, and structure

| Check | Observation / measurement | Status | Photo or evidence ID |
| --- | --- | --- | --- |
| Base frame seated and rigid on fixture |  |  |  |
| Six actuator bodies and lower mounts fully seated |  |  |  |
| Six rods installed in the intended axis positions |  |  |  |
| Upper and lower joints retainers/clips installed |  |  |  |
| Platform plate and all six upper joints secure |  |  |  |
| Fasteners tightened; witness marks intact or newly applied |  |  |  |
| No cracked, bent, loose, or damaged structural part |  |  |  |
| No loose washers, spacers, tools, or foreign objects |  |  |  |

### 2.2 Rod and geometry observations

Measure from the defined mechanical datums. These are commissioning
observations, not a replacement for the later measured geometry freeze.

| Axis / item | Rod or dimension ID | Measured value (mm) | Instrument / uncertainty | Status |
| --- | --- | ---: | --- | --- |
| 1 | `L0_1` |  |  |  |
| 2 | `L0_2` |  |  |  |
| 3 | `L0_3` |  |  |  |
| 4 | `L0_4` |  |  |  |
| 5 | `L0_5` |  |  |  |
| 6 | `L0_6` |  |  |  |
| Rod range (`max - min`) |  |  |  |  |
| Base anchor radius / datum |  |  |  |  |
| Platform anchor radius / datum |  |  |  |  |

### 2.3 Manual travel, joints, and clearances

| Check | Observation | Status | Deviation / evidence ID |
| --- | --- | --- | --- |
| Each slider can be moved gently through the permitted unpowered check range |  |  |  |
| No binding, dead spot, scraping, or abnormal friction |  |  |  |
| Each spherical/universal joint articulates freely without a hard spot |  |  |  |
| Joint play is absent or recorded as a measured baseline |  |  |  |
| Platform can be gently tilted within the safe hand-check envelope |  |  |  |
| Rods, guides, mounts, and platform have clearance throughout the hand-check envelope |  |  |  |
| Harness channels and moving parts have clearance and strain relief |  |  |  |
| No contact with the fixture, floor, covers, or neighboring actuators |  |  |  |

**Mechanical inspection status:** `PASS / FAIL / HOLD` __________

**Required failure action:** stop the session, keep power disconnected, and
return the assembly for mechanical correction. Do not compensate for a defect
with software limits or geometry edits.

## 3. Electrical preparation (still unpowered)

These checks prepare the first power-on. They do not include energizing the
24 V supply or communicating with a motor.

| Check | Observation / value | Status | Evidence ID |
| --- | --- | --- | --- |
| 24 V supply model, polarity, and rating identified |  |  |  |
| Current limit target recorded before connection |  |  |  |
| Fuse / protective device present and rated |  |  |  |
| E-stop is in the power path and physically accessible |  |  |  |
| E-stop mechanical action and reset action checked without energizing load |  |  |  |
| Motor power connectors keyed, latched, and polarity-checked |  |  |  |
| No exposed conductor, crushed cable, or damaged insulation |  |  |  |
| RS485 A/B, signal ground, shield, and termination plan identified |  |  |  |
| Cable routing is separated from pinch points and moving joints |  |  |  |
| Continuity / short checks completed with all power sources disconnected |  |  |  |
| Protective earth / chassis bonding checked where applicable |  |  |  |
| First power-on mode is documented as READ_ONLY with motion disabled |  |  |  |
| Pre-power software state is known; expected `hardware_write_count = 0` |  |  |  |

**Electrical preparation status:** `PASS / FAIL / HOLD` __________

**Electrical notes / unresolved items:**

______________________________________________________________________________

______________________________________________________________________________

## 4. IMU installation record

Record physical installation status only. If an IMU is not installed at this
session, mark the row `DEFERRED` and record the reason. Do not claim final
extrinsics, bias calibration, synchronization, or lever-arm acceptance from
this record.

| Field | Base IMU | Platform IMU |
| --- | --- | --- |
| Installed / deferred |  |  |
| Manufacturer / model |  |  |
| Serial number |  |  |
| Mount / bracket ID |  |  |
| Mechanical datum / reference face |  |  |
| Sensor axes orientation recorded |  |  |
| Connector and cable strain relief secure |  |  |
| Fastener / clamp condition |  |  |
| Installation date / operator |  |  |
| Photo IDs |  |  |
| Installation status (`PASS / FAIL / DEFERRED`) |  |  |

### IMU orientation and datum observations

| Observation | Base IMU | Platform IMU |
| --- | --- | --- |
| X-axis direction relative to documented frame |  |  |
| Y-axis direction relative to documented frame |  |  |
| Z-axis direction relative to documented frame |  |  |
| Contact / seating faces free of rocking |  |  |
| Any bracket flex, cable pull, or obstruction |  |  |

**IMU record status:** `PASS / FAIL / HOLD / DEFERRED` __________

**Deferred or damaged installation disposition:**

______________________________________________________________________________

## 5. Motor ID mapping preparation

This is a physical label and topology preparation record. It is not an
electronic ID confirmation; that confirmation requires the later READ_ONLY
and axis-mapping gates.

| Physical anchor / branch | Motor label seen | Expected software ID | Cable / connector ID | Photo ID | Checked by |
| --- | --- | ---: | --- | --- | --- |
| `A1` / branch 1 |  | `ID1` |  |  |  |
| `A2` / branch 2 |  | `ID2` |  |  |  |
| `A3` / branch 3 |  | `ID3` |  |  |  |
| `A4` / branch 4 |  | `ID4` |  |  |  |
| `A5` / branch 5 |  | `ID5` |  |  |  |
| `A6` / branch 6 |  | `ID6` |  |  |  |

- [ ] Mapping uses the approved anchor order and coordinate convention.
- [ ] Labels are legible from the operator position and are not duplicated.
- [ ] A top-view or isometric topology photograph/drawing is attached.
- [ ] Any suspected swap or ambiguous label is recorded as `FAIL`, not guessed.

**Mapping preparation status:** `PASS / FAIL / HOLD` __________

## 6. Zero-position observation (unpowered)

Observe and record the current mechanical state only. Do not infer encoder
values while unpowered. `STORAGE_PARK` is the mechanical storage position;
`OPERATING_HOME` is the software operating reference and must not be assumed
from this observation.

| Axis | Current mechanical position / witness mark | Lower stop visible and intact | Rod extension (mm) | Interference / clearance note | Status |
| --- | --- | --- | ---: | --- | --- |
| 1 |  |  |  |  |  |
| 2 |  |  |  |  |  |
| 3 |  |  |  |  |  |
| 4 |  |  |  |  |  |
| 5 |  |  |  |  |  |
| 6 |  |  |  |  |  |

| Global observation | Entry |
| --- | --- |
| Assembly appears mechanically near `STORAGE_PARK`? |  |
| Mechanical reference marks installed / legible? |  |
| Platform level and supported without actuator power? |  |
| Encoder `q_bottom_1..q_bottom_6` read later in Phase 2 | `PENDING` |

**Zero-position observation status:** `PASS / FAIL / HOLD` __________

## 7. Safety checks before first power-on

All items below must be `PASS` before leaving Phase 0. A `FAIL` or unknown
condition is a stop condition; do not proceed by accepting a verbal override.

| Safety check | Result / evidence | Status |
| --- | --- | --- |
| Phase 0 mechanical inspection signed PASS |  |  |
| Electrical preparation signed PASS |  |  |
| IMU installation status and any deferral reviewed |  |  |
| Physical motor ID map reviewed by a second person |  |  |
| E-stop location demonstrated to all personnel |  |  |
| E-stop holder has an unobstructed view and can reach it immediately |  |  |
| 24 V source remains OFF and disconnected until the next gate is authorized |  |  |
| Supply current limit, fuse, and polarity are independently verified |  |  |
| Platform is unloaded; hands, tools, and clothing are outside moving envelope |  |  |
| Covers / guards and mechanical stops are installed or deviation is approved |  |  |
| Emergency power-removal procedure and responsible person agreed |  |  |
| First power-on is restricted to READ_ONLY; no motion command is authorized |  |  |
| Expected hardware write count before power-on is exactly zero |  |  |
| Stop rules reviewed: unexpected motion, noise, heat, odor, current, fault, or loss of comms |  |  |

**Pre-power safety gate:** `PASS / FAIL / HOLD` __________

## 8. Disposition and sign-off

### Open deviations and corrective actions

| ID | Description | Immediate containment | Owner | Due / disposition | Closed / reviewer |
| --- | --- | --- | --- | --- | --- |
|  |  |  |  |  |  |
|  |  |  |  |  |  |
|  |  |  |  |  |  |

### Phase 0 decision

Select exactly one:

- [ ] `PASS`: all required sections pass, evidence is attached, and Phase 1
      READ_ONLY entry may be reviewed.
- [ ] `FAIL`: at least one required item failed; power remains disconnected.
- [ ] `HOLD`: inspection is incomplete or evidence/reviewer is missing; power
      remains disconnected.

| Sign-off | Name / signature | Date / time |
| --- | --- | --- |
| Operator |  |  |
| Safety observer |  |  |
| Reviewer / commissioning authority |  |  |

**Final hardware write count for this record:** `0` (no hardware execution was
performed by preparing or completing this document).
