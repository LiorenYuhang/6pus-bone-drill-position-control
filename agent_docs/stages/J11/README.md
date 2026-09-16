# Stage J11: PID 3DOF World-Attitude Stabilization / Anti-Vibration

> **Authoritative Baseline**: Frozen J8 (`v1.0-j8-active-servo-hardware-qualified`) + J9 Dual IMU Integration + J10 Axis-Wise Operational Safe Workspace Facts  
> **Repository Path**: `/home/lyapunov/bone_drill`  
> **Workspace Path**: `/home/lyapunov/bone_drill/bone_ws`  
> **Status**: **PRE-MOTION QUALIFIED (J11-1C Production-Mapping Correction + Shadow Integration + Pre-Motion Freeze; hardware_write_count = 0)**

---

## 1. Goal and Mission Definition

Stage J11 establishes the baseline for **active world-attitude stabilization and anti-vibration** on the physical 6PUS parallel manipulator.

- **Primary Goal**: When the Base platform is subjected to manually applied angular disturbances across Roll, Pitch, and Yaw, the moving Platform shall maintain its established initial orientation in the world coordinate frame ($W$) as close as possible to the reference orientation established at PID enable time ($R_{WP, \text{des}}$).
- **Single-Axis and Combined Attitude Scope**: J11 focuses exclusively on the 3 rotational degrees of freedom (Roll, Pitch, Yaw).
- **XYZ Translational Scope**: Linear translation stabilization ($X, Y, Z$) is explicitly out of scope for J11.
- **Actuator & Kinematic Calibration Status**: **DEFERRED**. The physical calibration of anchor geometry, link lengths, and slider zero offsets is deferred to a later dedicated calibration stage and is **not** part of J11.

---

## 2. Frame Contract: $R_{WP}$ vs $R_{BP}$ and Command Composition

Understanding and strictly separating the world attitude frame and the robot base-relative kinematics frame is the highest priority mathematical contract in J11.

### 2.1 Definitions of Frames and Orientations

| Symbol | Coordinate Frame | Physical Meaning & Source |
| :--- | :--- | :--- |
| $W$ | World Frame (Initial Inertial Reference) | In the H30 driver implementation, upon initialization the sensor establishes an initial orientation reference $q_0$, and subsequent outputs represent rotation relative to this initial reference frame. Unless an explicit external geodetic gravity/leveling alignment or absolute magnetic yaw reset is performed, this "World frame" is specifically the **H30 initial inertial reference / captured initial attitude frame**. For J11 anti-vibration / stabilization, holding $\Delta R_P(t) \to I$ relative to this captured frame is mathematically rigorous, physically self-consistent, and sufficient; it must not be conflated with geodetic or geocentric coordinates. |
| $B$ | Base Frame | Rigidly attached to the 6PUS manipulator base plate. |
| $P$ | Platform Frame | Attached to the moving platform (mechanical coordinate frame). |
| $IP$ | Platform IMU Frame | Sensor chip frame of the Platform H30 IMU mounted on the platform. |
| $IB$ | Base IMU Frame | Sensor chip frame of the Base H30 IMU mounted on the base plate. |
| $R_{WP}$ | $W \to P$ | Platform orientation in World coordinates, measured by Platform H30. |
| $R_{WB}$ | $W \to B$ | Base orientation in World coordinates, measured by Base H30 (logging only). |
| $R_{BP}$ | $B \to P$ | **Base-to-Platform relative orientation**, required by J8 Inverse Kinematics. |

### 2.2 Proof: Nominal Manipulator HOME Orientation $R_{BP, \text{home}} \equiv I_{3\times 3}$ and Dual-Reference Mapping

In J8 kinematics (`pus6_kinematics/pus6_kinematics/analytical_6pus.py`) and authoritative configuration (`pus6_kinematics/config/6pus_geometry.yaml`):
1. The forward and inverse kinematics models represent platform pose as:
   147159\mathbf{x} = [x, y, z, \alpha, \beta, \gamma]^\top147159
   where $x, y, z$ are platform center position in meters relative to Base frame $B$, and $\alpha, \beta, \gamma$ are extrinsic XYZ (Tait-Bryan ZYX) angles defining the rotation matrix $R_{BP} = R_z(\gamma) R_y(\beta) R_x(\alpha)$.
2. At nominal neutral height ($z = z_0 = \text{DEFAULT\_NOMINAL\_HEIGHT\_M} \approx 0.132612268\text{ m}$):
   147159\mathbf{x}_{\text{home}} = [0.0, 0.0, z_0, 0.0, 0.0, 0.0]^\top147159
3. Evaluating $R_{BP}$ at $\alpha=0, \beta=0, \gamma=0$:
   $$R_{BP}(0, 0, 0) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} = I_{3\times 3}$$
4. **Authoritative Dual-Reference Mapping**:
   - **Analytical Geometric Layer**:
     The absolute analytical strut extension coordinate at HOME is $q_{\text{abs}} = 0.050\text{ m}$ (50.0 mm coordinate in analytical kinematics model).
   - **Production Hardware / Differential IK Layer**:
     The nominal reference extension is:
     147159\mathbf{q}_{\text{ref}} = [0.050, 0.050, 0.050, 0.050, 0.050, 0.050]^\top\text{ m}147159
     The differential extension from HOME is:
     147159\Delta \mathbf{q}_m = \mathbf{q}_{\text{abs}} - \mathbf{q}_{\text{ref}} = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]^\top\text{ m}147159
     The differential step displacement is:
     147159\Delta \mathbf{s} = \text{round}(\Delta \mathbf{q}_m \times 40000\text{ steps/m}) = [0, 0, 0, 0, 0, 0]^\top\text{ steps}147159
     The physical hardware slider raw step coordinate for OPERATING_HOME is:
     147159\mathbf{s}_{\text{operating\_home}} = [1000, 1000, 1000, 1000, 1000, 1000]^\top\text{ steps}147159
     (Storage park is $[0, 0, 0, 0, 0, 0]^\top\text{ steps}$; mechanical slider travel capacity is $0 \sim 2000\text{ steps} = 0 \sim 50\text{ mm}$).
     The actual target raw steps sent to motors by J8 control node are:
     147159\mathbf{s}_{\text{target}} = \mathbf{s}_{\text{operating\_home}} + \Delta \mathbf{s}147159
   - **Crucial Clarification**: Directly multiplying $q_{\text{abs}} = 0.050\text{ m} \times 40000 = 2000\text{ steps}$ confused the mechanical slider travel upper capacity ($50\text{ mm}$ above park) with the operating home position. OPERATING_HOME is located at slider mid-stroke ($25\text{ mm}$ above park $= 1000\text{ steps}$). At OPERATING_HOME, $\Delta \mathbf{s} \equiv \mathbf{0}$, so $\mathbf{s}_{\text{target}} = [1000, 1000, 1000, 1000, 1000, 1000]^\top\text{ steps}$.
5. **Conclusion**: $R_{BP, \text{home}} \equiv I_{3\times 3}$ and $\mathbf{s}_{\text{operating\_home}} = [1000]*6$ are rigorously proven from geometry, config, and authoritative code.

### 2.3 Exact Command Composition Equation: $R_{BP, \text{cmd}}$

The J8 topic `/pus6/target_cartesian_pose` carries `PoseStamped` with `frame_id = "pus6_base"`. It represents a **Base-relative** target!
- **Strict Prohibition**: Never write $R_{BP, \text{cmd}} = R_{WP, \text{des}} \dots$ without explicit proof of frame transport.
- When Base is disturbed by $R_{WB}$, Platform initially tilts with the Base:
  $$R_{WP, \text{meas}} = R_{WB} R_{BP}$$
- At reference capture time, desired world attitude is $R_{WP, \text{des}}$ (where relative attitude from initial capture is identity $I$).
- The SO(3) attitude tracking error in World frame is:
  $$E_W = R_{WP, \text{des}} R_{WP, \text{meas}}^\top$$
  $$\mathbf{e}_W = \text{Log}(E_W)^\vee \in \mathbb{R}^3$$
- Transporting this error to reference platform body frame:
  $$\mathbf{e}_{\text{ref}} = R_{WP, \text{des}}^\top \mathbf{e}_W$$
- Task-space PID generates attitude correction residual vector $\delta\boldsymbol{\phi}_{\text{feedback}} \in \mathbb{R}^3$:
  $$\delta\boldsymbol{\phi}_{\text{feedback}} = K_p \mathbf{e}_{\text{ref}} + K_i \int \mathbf{e}_{\text{ref}} dt - K_d \boldsymbol{\omega}_P$$
- The commanded Base-relative Cartesian attitude sent to J8 is strictly:
  147159\boxed{R_{BP, \text{cmd}} = R_{BP, \text{home}} \exp([\delta\boldsymbol{\phi}_{\text{feedback}}]_\times) = \exp([\delta\boldsymbol{\phi}_{\text{feedback}}]_\times)}147159
- Quaternion representation for `PoseStamped`:
  $$\mathbf{q}_{\text{cmd}} = \text{rot\_matrix\_to\_quat\_xyzw}(R_{BP, \text{cmd}})$$

### 2.4 Disturbance-Rejection Sign Verification
- Suppose an external disturbance tilts Base around $+X$ (Roll) by $+0.5^\circ$.
- Platform initially tilts $+0.5^\circ$ in World: $R_{WP, \text{meas}} = R_x(+0.5^\circ)$.
- Tracking error: $E_W = I R_x(-0.5^\circ) \implies \mathbf{e}_W = [-0.5^\circ, 0, 0]^\top \implies e_{\text{Roll}} = -0.5^\circ < 0$.
- With positive proportional gain $K_p > 0$: $\delta\phi_{\text{feedback}, \text{Roll}} < 0$.
- Commanded relative orientation: $R_{BP, \text{cmd}} = R_x(-0.5^\circ K_p)$.
- J8 commands Platform to tilt by $-0.5^\circ$ **relative to Base**.
- Resulting Platform World attitude: $R_{WP} \approx R_{WB} R_{BP} = R_x(+0.5^\circ) R_x(-0.5^\circ) = I = R_{WP, \text{des}}$.
- Disturbance is rejected! The sign convention is mathematically sound.

---

## 3. Controller Lifecycle and State Machine

The J11 execution runner implements a deterministic finite state machine:

```text
                  +-------------+
                  |  DISARMED   | (Initial state on launch)
                  +------+------+
                         | IMU topic connected & streaming
                         v
             +-----------+-----------+
             |    WAITING_FOR_IMU    |
             +-----------+-----------+
                         | Samples >= 50, dt valid
                         v
            +------------+------------+
            |    REFERENCE_CAPTURE    | (Average q0 over quiet window >= 1.0s)
            +------------+------------+
                         | Stability gate passed (var < threshold, rate < thresh)
                         | [FAIL -> abort to WAITING_FOR_IMU]
                         v
                  +-------------+
                  |    READY    | (Reference locked, waiting for user arming)
                  +------+------+
                         | User passes --execute-motion AND pre-flight check PASS
                         v
                 +-------+-------+
                 |  ARMED_HOLD   | (Hold HOME pose, verify 0x29 stream)
                 +-------+-------+
                         | Start stabilization command
                         v
                  +-------------+
                  |   ACTIVE    | (50Hz closed-loop task-space PID stabilization)
                  +------+------+
                         |
      +------------------+------------------+
      | Fault detected                      | User disarm / manual stop
      v                                     v
+-----+-----+                         +-----+-----+
|   FAULT   |                         | DISARMED  |
+-----------+                         +-----------+
```

### Safety Rule for J11 Pre-Motion Baseline:
- The execution runner is **DRY-RUN by default**.
- The runner **never transitions past READY** unless `--execute-motion` is explicitly provided.
- In dry-run, commands are generated and validated internally: **zero writes to `/pus6/target_cartesian_pose` or motor serial port (`hardware_write_count = 0`)**.

---

## 4. Safety Gates and Failure Protections

### 4.1 Reference Capture Qualification Gate
Platform initial attitude reference $R_{WP, \text{des}}$ must never be locked from a single frame:
1. **Minimum Sample Count**: $\ge 50$ consecutive valid IMU frames ($\ge 1.0\text{ s}$ at 50Hz, or $\ge 0.25\text{ s}$ at 200Hz).
2. **Quaternion Validity**: $\|q\| \in [0.99, 1.01]$, all components finite, no NaN/Inf.
3. **Stationary / Stability Gate**:
   - Maximum sample angular rate $\|\boldsymbol{\omega}_P\| < 0.05\text{ rad/s}$ ($ < 2.86^\circ/\text{s}$).
   - Orientation standard deviation across capture window $\sigma_{\text{quat}} < 0.002$.
   - If Platform moves noticeably during capture: **Reference Capture FAILS**, state reverts to `WAITING_FOR_IMU`.

### 4.2 Platform IMU Freshness and Validity Gate
At every 50Hz control tick, the runner verifies:
1. **Freshness**: $\text{age} = t_{\text{now}} - t_{\text{imu\_rx}} \le \text{max\_platform\_imu\_age\_s}$ (configurable, default $0.050\text{ s}$).
2. **Finite & Normalized**: Quaternion is finite and unit-norm.
3. **No Timestamp Regression**: Ingestion timestamps are strictly monotonically increasing.
4. **Valid Control Period**: $0.005\text{ s} \le dt \le 0.100\text{ s}$.
- **Trip Behavior**: If any validity check fails, the controller suspends active updates, holds the last safe target or zeros the residual, logs a diagnostic warning, and will not issue unvalidated commands.

### 4.3 Base IMU Role Gate
- Base IMU is an observer/logger only.
- If Base IMU drops packets or becomes stale, it logs a telemetry warning but **does not invalidate** the closed-loop attitude stabilization, unless dual-IMU logging is explicitly required by experiment configuration.

---

## 5. Execution Semantics of Disable and Fault

### 5.1 Real Behavior of J8 Control and Driver
- **Upstream Stops Publishing**:
  When J11 encounters a FAULT or Disable, the J11 runner stops publishing to `/pus6/target_cartesian_pose`.
  "Stopping target publication" is purely upstream software behavior.
- **Downstream J8 Smooth Settlement and Holding**:
  In J8 (`pus6_control_node`), when no new target pose arrives, the node retains its last accepted target position `self._target_positions`. The second-order trajectory shaper continues executing at 50Hz, smoothly decelerating and settling toward that last accepted target position, and then holding it.
  - **No Snap / No Instant Mechanical Freeze**: Stepper motors do not abruptly snap or experience infinite jerk.
  - **No Friction Brake**: There is no mechanical friction brake on the physical manipulator. The steppers hold position via electromagnetic holding current.
  - **No Automatic Return to HOME**: J8 does NOT automatically return to HOME on upstream target silence.
- **Motor Driver Watchdog (Tier 1 & Tier 2)**:
  - If `/pus6/command_joint_steps` stops arriving for $> 200\text{ ms}$ (`target_timeout_sec`), `pus6_motor_driver_node` enters `_stream_timed_out = True` and holds `_last_commanded_steps`.
  - Tier 1 watchdog trips on 4 consecutive missed feedback frames.
  - Tier 2 watchdog trips on scheduling latency $> 263.324\text{ ms}$.
  - On watchdog trip, `J7StopAdapter` issues `0x1A STOP` (writing `[1]` to pause register `0x1A` on all 6 motors).

### 5.2 Bounded Last-Target Safety Check
Why holding the last accepted target on FAULT is inherently safe:
1. J11 attitude correction is strictly clamped: $\|\delta\boldsymbol{\phi}_{\text{feedback}}\| \le \delta\phi_{\text{max}} = 1.0^\circ$ ($0.01745\text{ rad}$).
2. Upstream J8 kinematics node actively validates that incoming targets are within the temporary J4 software envelope ($\pm 0.10\text{ rad} \approx 5.73^\circ$, $\pm 15\text{ mm}$ XY, $\pm 20\text{ mm}$ Z).
3. At the maximum possible J11 attitude deflection ($\pm 1.0^\circ$ in any direction), the required differential slider displacement is at most $\pm 16\text{ steps}$ ($\pm 0.4\text{ mm}$), resulting in raw steps strictly between 984 and 1016 steps.
4. Physical hardware limits are $[200, 1800]$ steps.
5. Therefore, the last accepted target is always located deep within the safe operating envelope, and holding it poses zero collision or limit hazard.

### 5.3 Deterministic J11 Disable/Fault Policy
- **Strict Prohibition**: On Disable or Fault, the runner **must never** instantaneously command relative nominal HOME $[0, 0, z_0, 0, 0, 0]$ or snap steps. If Base is tilted, snapping relative orientation to identity would command the platform to tilt with the base, causing a dangerous sudden movement.
- **Deterministic Action on FAULT**:
  1. `pid.disable()`: Integrator and command residual are zeroed immediately in software.
  2. The runner ceases active publishing to `/pus6/target_cartesian_pose`.
  3. Downstream J8 smoothly completes deceleration to the last accepted target and holds position.

---

## 6. Single Topic Ownership Gate

To prevent conflicting Cartesian target commands from competing nodes (e.g. legacy J8 demo runners, J9 servo scripts, manual GUI publishers):
- Before transitioning to `ACTIVE`, the runner queries the ROS 2 node graph:
  ```python
  pubs = self.get_publishers_info_by_topic("/pus6/target_cartesian_pose")
  ```
- If any publisher exists other than the J11 runner itself:
  The runner logs a critical error and **refuses to activate**.

---

## 7. Frozen Production Boundary (Zero-Modification Mandate)

1. **Production Package Diff = 0**: Zero modifications in `pus6_motor_driver`, `pus6_control`, `pus6_kinematics`.
2. **Untampered Execution Black Box**: All Cartesian targets must flow through `/pus6/target_cartesian_pose`.
3. **Hardware Safety Envelopes**: J4 software envelope ($\pm 15\text{ mm}$ XY, $\pm 20\text{ mm}$ Z, $\pm 0.10\text{ rad} \approx 5.73^\circ$ attitude) actively enforced by `pus6_kinematics_node`.
4. **hardware_write_count = 0**: Strictly enforced during J11 pre-motion qualification.

---

## 8. Shadow Integration Qualification (J11-1C)

To guarantee software chain integrity without any physical motor interaction, a zero-hardware shadow integration harness (`bone_ws/src/h30_imu/scripts/j11_shadow_integration_harness.py`) executed live ROS 2 nodes:
- `Pus6KinematicsNode` (real production inverse kinematics node)
- `Pus6ControlNode` (real production 50Hz trajectory shaper node)
- `Pus6MotorDriverNode` (STRICTLY PROHIBITED; zero UART process running)

### Results Summary
All test cases exhibited exact bit-level step matches between offline differential kinematics and live ROS 2 pipeline outputs:

| Test Case | Commanded (R, P, Y) | Offline Delta Steps | ROS Kinematics Delta Steps | Offline Pred Raw Steps | ROS Control Final Raw Steps | Match Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **HOME** | $(0, 0, 0)^\circ$ | `[0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0, 0, 0]` | `[1000, 1000, 1000, 1000, 1000, 1000]` | `[1000, 1000, 1000, 1000, 1000, 1000]` | **MATCH_PASS** |
| **Roll $+0.25^\circ$** | $(+0.25, 0, 0)^\circ$ | `[4, 0, -4, -4, 0, 4]` | `[4, 0, -4, -4, 0, 4]` | `[1004, 1000, 996, 996, 1000, 1004]` | `[1004, 1000, 996, 996, 1000, 1004]` | **MATCH_PASS** |
| **Roll $-0.25^\circ$** | $(-0.25, 0, 0)^\circ$ | `[-4, 0, 4, 4, 0, -4]` | `[-4, 0, 4, 4, 0, -4]` | `[996, 1000, 1004, 1004, 1000, 996]` | `[996, 1000, 1004, 1004, 1000, 996]` | **MATCH_PASS** |
| **Roll $+0.50^\circ$** | $(+0.50, 0, 0)^\circ$ | `[8, 0, -8, -8, 0, 8]` | `[8, 0, -8, -8, 0, 8]` | `[1008, 1000, 992, 992, 1000, 1008]` | `[1008, 1000, 992, 992, 1000, 1008]` | **MATCH_PASS** |
| **Roll $-0.50^\circ$** | $(-0.50, 0, 0)^\circ$ | `[-8, 0, 8, 8, 0, -8]` | `[-8, 0, 8, 8, 0, -8]` | `[992, 1000, 1008, 1008, 1000, 992]` | `[992, 1000, 1008, 1008, 1000, 992]` | **MATCH_PASS** |

`hardware_write_count = 0` strictly verified throughout.
Report artifact saved at `results/j11/pre_motion/j11_shadow_integration_report.json`.
