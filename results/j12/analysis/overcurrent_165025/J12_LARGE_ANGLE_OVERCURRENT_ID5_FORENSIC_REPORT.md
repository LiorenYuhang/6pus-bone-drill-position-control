# J12 Large-Angle Overcurrent ID5 Forensic Report

**Experiment Run ID**: `20260923_165025`  
**Run Directory**: `/home/lyapunov/bone_drill/results/j11/m2_pid/20260923_165025/`  
**Audit Date**: 2026-09-23  
**Auditor**: Antigravity J12 Root-Cause Forensics  
**Controller Configuration**: Feedforward + Classical P/PD (`kp_roll=0.05, kp_pitch=0.05, kp_yaw=0.025, kd_roll=0.0, kd_pitch=0.0015, kd_yaw=0.0015`, LPF 5.0 Hz, max residual limit $\pm 0.5^\circ$)  
**Baseline Run for Comparison**: `/home/lyapunov/bone_drill/results/j11/m2_pid/20260920_095850/`  
**Generated Figures Directory**: `/home/lyapunov/bone_drill/results/j12/analysis/overcurrent_165025/`  

---

## 1. Executive Summary & Forensic Verdict

During a large-angle active attitude stabilization experiment (Run `20260923_165025`), the motion controller was driven by manual base tilting up to a Base Roll of **-38.11°** (Base $SO(3)$ magnitude of **38.33°**). At $t = 21.400\,\text{s}$ (Cycle 1071), the real-time driver aborted execution with a fatal safety latch:
```
[FATAL] [1790153454.783176076] [pus6_motor_driver_node]: J7 driver STOP latched: overcurrent_ID5
```

### Forensic Classification Summary
| Hypothesis | Verdict | Evidence Summary |
| :--- | :--- | :--- |
| **Large-Angle Geometric / Kinematic Load** | **STRONG EVIDENCE** | At Base Roll = -38.11°, the platform is tilted 8.1° beyond the J10 qualified boundary (+30.0°). Actuators ID3, ID4, and ID5 simultaneously suffered extreme current escalation (ID5=503 mA, ID3=415 mA, ID4=249 mA). Struts ID3/ID4 were compressed to 419–475 steps while ID1/ID6 were extended to 1581–1629 steps, generating severe mechanical strain and parasitic moments. |
| **Excessive Control / Motion Velocity** | **UNSUPPORTED** | Target step rate at fault was $\le 200\,\text{steps/s}$ (ID5 moved only 12 steps in 400 ms, i.e., $30\,\text{steps/s} = 0.75\,\text{mm/s}$). Baseline run survived speeds of $808\,\text{steps/s}$ with currents $\le 182\,\text{mA}$. |
| **P/PD Residual Causing the Overcurrent** | **UNSUPPORTED** | P/PD residual at fault was only $+0.17^\circ$ roll (well below $\pm 0.5^\circ$ limit; unsaturated). Feedforward contributed $>99.5\%$ of the total $+38.14^\circ$ command. |
| **Host Runtime / USB Scheduler Stall** | **UNSUPPORTED** | Loop $dt$ across the 2 seconds prior to fault was rock solid (mean $20.00\,\text{ms}$, min $19.74\,\text{ms}$, max $20.29\,\text{ms}$, 0 stalls $>30\,\text{ms}$). |
| **ID5 Local Mechanical Resistance** | **POSSIBLE** | ID5 was near mid-stroke ($1080\,\text{steps}$) with low tracking error ($15\,\text{steps} = 0.375\,\text{mm}$), yet drew more current than ID3 which was compressed to near lower stroke ($462\,\text{steps}$). However, without multi-axis force/torque load-cell data, parasitic kinematic force transfer vs. localized actuator friction cannot be separated. |

---

## 2. Catalog of Artifacts in Run `165025`

The directory `/home/lyapunov/bone_drill/results/j11/m2_pid/20260923_165025/` contains:
1. `j11_demo_feedforward_pd_summary_powered.json` (260 bytes):
   - `stage`: `"j11_demo_feedforward_pd"`
   - `mode`: `"POWERED REAL EXECUTION"`
   - `hardware_write_count`: 1016
   - `total_cycles`: 1071
   - `final_state`: `"FAULT"`
   - `fault_reason`: `"Driver fatal latch detected via /rosout: J7 driver STOP latched: overcurrent_ID5"`
2. `j11_demo_feedforward_pd_telemetry_powered.csv` (3,096,634 bytes):
   - 1,071 rows, 57 telemetry columns recorded at 50 Hz ($20.0\,\text{ms}$ interval).
   - Cycles 1–54: `WAITING_FOR_PREFLIGHT` and `CAPTURING_REFERENCE`.
   - Cycles 55–1070: `ACTIVE` closed-loop execution (1016 cycles, 20.30 s).
   - Cycle 1071: `FAULT` transition.
3. System logs:
   - `/tmp/j11_m2_robot.log`: captures vector slew recovery events and the fatal rosout line at $1790153454.783176076$.
   - Note on Diagnostics: `cycle_timing.csv`, `stall_diagnostic_summary.json`, and `fault_snapshot.json` were NOT generated because the run used default `enable_runtime_diagnostics=False` per the Runtime-Fix-1B safety specification.

---

## 3. Authoritative Driver Trigger Logic in Source Code

The fatal overcurrent shutoff is enforced at the driver feedback boundary:

- **Source File**: `bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_servo_feedback_scheduler.py`
- **Class**: [`J7ServoFeedbackScheduler`](file:///home/lyapunov/bone_drill/bone_ws/src/pus6_motor_driver/pus6_motor_driver/j7_servo_feedback_scheduler.py#L40)
- **Function**: `_validate_status(self, motor_id: int, status: Pus6FeedbackFrame)` (lines 636–642):
  ```python
  if status.current_mA > MAX_CURRENT_MA:
      self._stop(f"overcurrent_ID{motor_id}")
      return
  ```
- **Threshold Constant**: `MAX_CURRENT_MA: int = 500`
  - Defined in `bone_ws/src/pus6_motor_driver/pus6_motor_driver/safety.py:38`.
  - Unit: milliamperes ($\text{mA}$).
- **Hardware Feedback Field**: `status.current_mA`
  - Decoded in `bone_ws/src/pus6_motor_driver/pus6_motor_driver/protocol.py:295`:
    ```python
    current_mA = int.from_bytes(frame.data[4:6], "little", signed=False)
    ```
  - Unsigned 16-bit integer received from CAN/UART motor packet.
- **Sample Gate Policy**: **Single raw sample** exceeding $500\,\text{mA}$ triggers immediate `self._stop()`. There is **zero filtering**, **zero low-pass filter**, and **zero debounce window**.

---

## 4. Exact Fault Timeline Reconstruction

| Wall Time (s) | ROS Stamp / ns | Cycle Index | State | Base Roll (°) | Cmd Roll (°) | ID5 Current (mA) | Other Motor Currents (mA) [ID1, ID2, ID3, ID4, ID6] | Event Description |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **0.000** | 9717158034066 | 1 | WAITING | 0.00 | 0.00 | 0 | [0, 0, 0, 0, 0] | Session initialization. |
| **1.080** | 9718237785088 | 55 | ACTIVE | -0.00 | +0.00 | 0 | [0, 0, 0, 0, 0] | Reference captured; closed-loop ACTIVE begins. |
| **20.000** | 9737157834000 | 1001 | ACTIVE | -23.51 | +23.48 | 15 | [34, 18, 56, 42, 38] | Moderate tilt; normal currents. |
| **21.000** | 9738157791000 | 1051 | ACTIVE | -36.26 | +36.24 | 11 | [160, 39, 118, 0, 132] | Reaching steep roll tilt; current escalation starts on ID1/ID3. |
| **21.200** | 9738357774000 | 1061 | ACTIVE | -37.20 | +37.23 | 111 | [142, 15, 124, 86, 103] | ID5 current crosses 100 mA. |
| **21.260** | 9738417770000 | 1064 | ACTIVE | -37.59 | +37.62 | 195 | [111, 28, 167, 138, 105] | ID5 reaches 195 mA, ID3 reaches 167 mA. |
| **21.280** | 9738437768000 | 1065 | ACTIVE | -37.71 | +37.74 | 246 | [101, 40, 222, 145, 106] | ID5 reaches 246 mA, ID3 reaches 222 mA. |
| **21.320** | 9738477765000 | 1067 | ACTIVE | -37.91 | +37.94 | 326 | [0, 58, 272, 177, 119] | ID5 reaches 326 mA, ID3 reaches 272 mA. |
| **21.360** | 9738517762000 | 1069 | ACTIVE | -38.06 | +38.09 | 459 | [0, 78, 311, 206, 0] | ID5 surges to 459 mA, ID3 to 311 mA. |
| **21.380** | 9738537760000 | 1070 | ACTIVE | -38.11 | +38.14 | 487 | [0, 99, 400, 252, 0] | Last ACTIVE cycle: ID5=487 mA, ID3=400 mA, ID4=252 mA. |
| **21.400** | 9738557758000 | 1071 | FAULT | (hold -38.11) | (hold +38.14) | **503** | [0, 102, 415, 249, 15] | **Driver STOP latched: overcurrent_ID5** ($503 > 500\,\text{mA}$). Demo runner aborts. |

---

## 5. Window Reconstruction: 5s, 2s, and 1s Windows

### A. 5-Second Window ($t \in [16.4, 21.4]\,\text{s}$, Cycles 821 to 1071)
- **Base Roll Range**: $-14.2^\circ \to -38.11^\circ$ (continuous progressive tilting).
- **Base Angular Rate**: Mean $\approx 4.8^\circ/\text{s}$, peak $14.2^\circ/\text{s}$.
- **Loop Timing**: $dt \in [19.63, 20.65]\,\text{ms}$, mean $20.00\,\text{ms}$, zero stalls $>25\,\text{ms}$.
- **Current Behavior**: Up to $t=21.0\,\text{s}$, all motor currents remained below $160\,\text{mA}$.

### B. 2-Second Window ($t \in [19.4, 21.4]\,\text{s}$, Cycles 971 to 1071)
- **Base Roll Range**: $-18.5^\circ \to -38.11^\circ$.
- **Base Angular Rate**: $-6.87^\circ/\text{s}$ average roll rate.
- **Loop Timing**: $dt \in [19.74, 20.29]\,\text{ms}$, mean $20.00\,\text{ms}$, zero stalls $>21\,\text{ms}$.
- **PID Residuals**: Roll $+0.15^\circ \to +0.17^\circ$, Pitch $-0.32^\circ \to -0.28^\circ$, Yaw $-0.12^\circ \to -0.09^\circ$. Zero axis saturation.

### C. 1-Second Window ($t \in [20.4, 21.4]\,\text{s}$, Cycles 1021 to 1071)
- **Base Roll Range**: $-31.8^\circ \to -38.11^\circ$.
- **Current Ramp**:
  - $t = 20.4\,\text{s}$: ID5 = $0\,\text{mA}$, ID3 = $52\,\text{mA}$.
  - $t = 21.0\,\text{s}$: ID5 = $11\,\text{mA}$, ID3 = $118\,\text{mA}$.
  - $t = 21.2\,\text{s}$: ID5 = $111\,\text{mA}$, ID3 = $124\,\text{mA}$.
  - $t = 21.3\,\text{s}$: ID5 = $276\,\text{mA}$, ID3 = $208\,\text{mA}$.
  - $t = 21.4\,\text{s}$: ID5 = $503\,\text{mA}$, ID3 = $415\,\text{mA}$.
- **ID5 Current Waveform Shape**: A steep, continuous, monotonic ramp over $400\,\text{ms}$ ($20\,\text{cycles}$), exactly mirroring the mechanical load on the strut, **NOT a single-sample electrical spike**.

---

## 6. Detailed 6-Axis Current Comparison

At the instant of failure ($t = 21.400\,\text{s}$, Cycle 1071):
- **Motor ID1**: $0\,\text{mA}$ (commanded 1581, feedback 1562; fully extended, unloaded).
- **Motor ID2**: $102\,\text{mA}$ (commanded 906, feedback 914; near mid-stroke).
- **Motor ID3**: **$415\,\text{mA}$** (commanded 419, feedback 462; severely compressed, 83% of trip threshold).
- **Motor ID4**: **$249\,\text{mA}$** (commanded 475, feedback 492; severely compressed).
- **Motor ID5**: **$503\,\text{mA}$** (commanded 1095, feedback 1080; tripped overcurrent safety gate).
- **Motor ID6**: $15\,\text{mA}$ (commanded 1629, feedback 1586; fully extended, unloaded).

Notice that **ID3, ID4, and ID5 simultaneously experienced extreme current loads**, while ID1 and ID6 were nearly idle. This 3-motor load concentration is characteristic of high-angle Stewart platform asymmetric loading under pure roll.

---

## 7. Attitude Metrics & Operating Point

### Exact Maximum Attained Angles (Run 165025)
- **Base Maximum Roll**: **$-38.11^\circ$** (positive peak $+32.98^\circ$).
- **Base Maximum Pitch**: **$-20.32^\circ$** (positive peak $+11.01^\circ$).
- **Base Maximum Yaw**: **$-4.24^\circ$** (positive peak $+5.48^\circ$).
- **Base Maximum $SO(3)$ Norm**: **$38.33^\circ$**.
- **Platform Maximum $SO(3)$ Tracking Error**: **$27.29^\circ$**.

### Exact Operating Point at Fault (Cycle 1070–1071)
- **Base Orientation**: $\text{Roll} = -38.11^\circ$, $\text{Pitch} = +2.60^\circ$, $\text{Yaw} = -4.24^\circ$, $SO(3) = 38.33^\circ$.
- **Platform Tracking Error**: $\text{Roll} = -3.27^\circ$, $\text{Pitch} = +5.89^\circ$, $\text{Yaw} = +3.87^\circ$, $SO(3) = 7.85^\circ$.
- **Feedforward Attitude**: $\text{Roll} = +37.95^\circ$, $\text{Pitch} = -4.66^\circ$, $\text{Yaw} = +1.74^\circ$.
- **Commanded Attitude**: $\text{Roll} = +38.14^\circ$, $\text{Pitch} = -4.82^\circ$, $\text{Yaw} = +1.50^\circ$.
- **P/PD Residual**: $\Delta\text{Roll} = +0.17^\circ$, $\Delta\text{Pitch} = -0.28^\circ$, $\Delta\text{Yaw} = -0.10^\circ$.
- **P/PD Saturated Axes**: `[False, False, False]`.

The test achieved a true physical Base Roll of **$38.11^\circ$**, which is **$8.1^\circ$ beyond the J10 measured boundary**.

---

## 8. Six-Axis Actuator Positions vs. J10 Boundary

| Actuator ID | Commanded Steps | Feedback Steps | Tracking Error (Steps) | Tracking Error (mm) | J10 Roll+ Qualified Boundary | Deviation from Neutral (1000) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **ID1** | 1581 | 1562 | $+19$ | $+0.475\,\text{mm}$ | 1502 | $+581\,\text{steps}$ (Extended) |
| **ID2** | 906 | 914 | $-8$ | $-0.200\,\text{mm}$ | 1000 | $-94\,\text{steps}$ |
| **ID3** | 419 | 462 | $-43$ | $-1.075\,\text{mm}$ | 533 | $-581\,\text{steps}$ (Compressed) |
| **ID4** | 475 | 492 | $-17$ | $-0.425\,\text{mm}$ | 533 | $-525\,\text{steps}$ (Compressed) |
| **ID5** | 1095 | 1080 | $+15$ | $+0.375\,\text{mm}$ | 1000 | $+95\,\text{steps}$ |
| **ID6** | 1629 | 1586 | $+43$ | $+1.075\,\text{mm}$ | 1502 | $+629\,\text{steps}$ (Extended) |

### Findings:
1. **Actuator Stroke Limits**: Actuator hard stroke limits are $[0, 2000]\,\text{steps}$ ($[0, 50.0]\,\text{mm}$). Commanded values ($419$ to $1629$) are well within the physical limit $[0, 2000]$.
2. **J10 Task-Space Envelope**: In J10 workspace qualification (`J10_A9_ROLL_COMPLETE_CHARACTERIZATION_20260909.md`), the maximum qualified Roll was **$+30.0^\circ$** with boundary $[1502, 1000, 533, 533, 1000, 1502]$. At $+38.14^\circ$, struts ID3 and ID4 penetrated deep below the qualified lower stroke ($419$ vs $533$), and struts ID1 and ID6 pushed past the upper stroke ($1629$ vs $1502$).
3. **Singularity / Kinematic Conditioning**:
   - Analytical IK check indicates strut vertical inclination angles remain $\theta_z \in [10.2^\circ, 14.4^\circ]$ ($\cos\theta_z \ge 0.968$).
   - Mathematical singularity ($\text{radicand} \le 0$): **NOT ESTABLISHED / NONE**.
   - Kinematic disadvantage: **ESTABLISHED**. At high angles, the mechanical advantage of the linear actuators against external/parasitic moments degrades rapidly, increasing joint reactions.

---

## 9. Rate and Velocity Chain Analysis

- **Base Angular Rate at Fault**: $-6.87^\circ/\text{s}$ (Roll dot). Across the entire run, maximum angular rate was $30.85^\circ/\text{s}$.
- **Target Step Rates at Fault**:
  - ID1: $+100\,\text{steps/s}$ ($2.5\,\text{mm/s}$)
  - ID2: $-50\,\text{steps/s}$ ($1.25\,\text{mm/s}$)
  - ID3: $-200\,\text{steps/s}$ ($5.0\,\text{mm/s}$)
  - ID4: $-50\,\text{steps/s}$ ($1.25\,\text{mm/s}$)
  - ID5: $+30\,\text{steps/s}$ ($0.75\,\text{mm/s}$)
  - ID6: $+200\,\text{steps/s}$ ($5.0\,\text{mm/s}$)
- **Driver Velocity Limiter**: `pus6_motor_driver_node` clamps maximum velocity to $200\,\text{steps/s}$ ($5.0\,\text{mm/s}$).
- **Tracking Error**:
  - ID5 tracking error was only **$+15\,\text{steps}$ ($0.375\,\text{mm}$)**.
  - In comparison, ID3 tracking error was $-43\,\text{steps}$, and ID6 was $+43\,\text{steps}$.
- **Velocity Chain Verdict**: The hypothesis that *"the operator rotated the base too fast, causing lag and current surge"* is **UNSUPPORTED**. Motion at fault was slow ($0.75\,\text{mm/s}$ on ID5), and tracking error remained tiny ($0.375\,\text{mm}$).

---

## 10. Role of P/PD Residual

- **Configured Residual Limits**: Roll $\pm 0.5^\circ$, Pitch $\pm 0.5^\circ$, Yaw $\pm 0.5^\circ$.
- **Residual at Fault**:
  - $\Delta\text{Roll} = +0.173^\circ$ ($34.6\%$ of limit)
  - $\Delta\text{Pitch} = -0.281^\circ$ ($56.2\%$ of limit)
  - $\Delta\text{Yaw} = -0.087^\circ$ ($17.4\%$ of limit)
- **Saturation Status**: `[False, False, False]`. Zero axes were saturated.
- **Relative Contribution**:
  - Total Commanded Roll: $+38.14^\circ$.
  - Pure Feedforward Roll ($R_{WB}^T$): $+37.95^\circ$ ($99.50\%$).
  - P/PD Correction: $+0.17^\circ$ ($0.50\%$).
- **Verdict**: The P/PD residual added less than one-sixth of a degree to the command. The overcurrent was **almost exclusively driven by the large-angle Feedforward geometry**, not by P/PD feedback.

---

## 11. Comparison with Frozen J11 Qualified Baseline

| Metric | Baseline Run `20260920_095850` | Fault Run `20260923_165025` | Comparison / Significance |
| :--- | :---: | :---: | :--- |
| **Duration / Cycles** | 58.92 s / 2999 cycles | 20.30 s / 1071 cycles | Baseline completed full test; fault run tripped at 21.4 s. |
| **Final State** | `ACTIVE` (Clean Stop) | `FAULT` (`overcurrent_ID5`) | Direct safety latch abort. |
| **Max Base Roll** | $14.28^\circ$ | **$38.11^\circ$** | **2.67× larger roll angle**. |
| **Max Base $SO(3)$** | $19.49^\circ$ | **$38.33^\circ$** | Nearly 2× larger spatial rotation. |
| **Platform $SO(3)$ Error** | $5.97^\circ$ | **$27.29^\circ$** | 4.5× larger tracking error at extreme tilt. |
| **Actuator Stroke Span** | $452$ to $564\,\text{steps}$ | **$581$ to $1216\,\text{steps}$** | **2.2× larger mechanical displacement**. |
| **Peak Step Rate** | $807.7\,\text{steps/s}$ ($20.2\,\text{mm/s}$) | $410.5\,\text{steps/s}$ ($10.3\,\text{mm/s}$) | Baseline moved **2× faster** without any overcurrent! |
| **Peak Motor Current** | $182\,\text{mA}$ (ID1) | **$503\,\text{mA}$ (ID5), $415\,\text{mA}$ (ID3)** | **2.76× higher peak current** in fault run. |
| **P/PD Saturation** | 0 saturation cycles | 142 Roll sat cycles during dynamic swings | Residual limit reached earlier in dynamic peaks. |
| **Mean Loop $dt$** | $20.00\,\text{ms}$ | $20.00\,\text{ms}$ | Identical 50 Hz real-time timing. |

---

## 12. Answers to the 14 Mandatory Forensic Questions

1. **ID5 overcurrent 准确发生在什么姿态？**  
   发生在 Base Roll = **$-38.11^\circ$**, Pitch = **$+2.60^\circ$**, Yaw = **$-4.24^\circ$**, Base $SO(3)$ = **$38.33^\circ$**。此时控制器输出的平台目标姿态为 Command Roll = **$+38.14^\circ$**, Pitch = **$-4.82^\circ$**, Yaw = **$+1.50^\circ$**。

2. **当时六轴位置是多少？**  
   - Commanded Steps: `[1581, 906, 419, 475, 1095, 1629]`
   - Feedback Steps: `[1562, 914, 462, 492, 1080, 1586]`

3. **ID5 current 是缓慢上升还是瞬时尖峰？**  
   是**连续且快速的爬升（400 ms 渐进爬升）**，不是单点毛刺。从 $t=21.00\,\text{s}$ 的 $11\,\text{mA}$，逐步爬升到 $56\,\text{mA} \to 111\,\text{mA} \to 195\,\text{mA} \to 246\,\text{mA} \to 326\,\text{mA} \to 459\,\text{mA} \to 487\,\text{mA} \to 503\,\text{mA}$。

4. **其他五轴电流是多少？**  
   在故障时刻（Cycle 1071），六轴电流为：  
   `ID1 = 0 mA`, `ID2 = 102 mA`, `ID3 = 415 mA`, `ID4 = 249 mA`, `ID5 = 503 mA`, `ID6 = 15 mA`。  
   ID3（415 mA）与 ID4（249 mA）同样处于严重的高载荷状态。

5. **ID5 tracking error是多少？**  
   仅为 **$+15\,\text{steps}$ ($0.375\,\text{mm}$)**。在六个轴中属于极小误差（ID3 为 $-43\,\text{steps}$，ID6 为 $+43\,\text{steps}$），表明电机并未失步或卡死，而是推力负荷极大。

6. **当时六轴target rate是多少？**  
   `ID1 = +100 steps/s` ($2.5\,\text{mm/s}$), `ID2 = -50 steps/s` ($-1.25\,\text{mm/s}$), `ID3 = -200 steps/s` ($-5.0\,\text{mm/s}$), `ID4 = -50 steps/s` ($-1.25\,\text{mm/s}$), `ID5 = +30 steps/s` ($0.75\,\text{mm/s}$), `ID6 = +200 steps/s` ($5.0\,\text{mm/s}$)。

7. **Base angular rate是多少？**  
   故障时刻 Base Roll Rate 仅为 **$-6.87^\circ/\text{s}$**，属于平缓的人工倾斜操作。

8. **P/PD residual是否饱和？**  
   **未饱和**。`pid_saturated_axes = [False, False, False]`。Residual 输出为 Roll = $+0.17^\circ$, Pitch = $-0.28^\circ$, Yaw = $-0.10^\circ$，均在 $\pm 0.5^\circ$ 阈值以内。

9. **是否有runtime stall在fault前发生？**  
   **没有**。故障前 2 秒（101 个周期）内，循环 $dt$ 严格保持在 $19.74 \sim 20.29\,\text{ms}$（均值 $20.00\,\text{ms}$），0 次超过 $21\,\text{ms}$ 的抖动。

10. **是否有证据支持：大角度几何/负载问题？**  
    **STRONG EVIDENCE**。此时 Roll 达到 $38.14^\circ$，已超越此前硬件通过的极限（$+30^\circ$）。ID3、ID4 被压低至 $419 \sim 475$ steps，ID1、ID6 被拉升至 $1581 \sim 1629$ steps。ID3（415 mA）、ID4（249 mA）与 ID5（503 mA）呈现典型的多轴几何高负荷耦合。

11. **是否有证据支持：速度太快？**  
    **UNSUPPORTED**。故障时各轴指令速率 $\le 200\,\text{steps/s}$（$5\,\text{mm/s}$），ID5 仅 $30\,\text{steps/s}$（$0.75\,\text{mm/s}$）。基线实验在 $808\,\text{steps/s}$ 时电流仅 $182\,\text{mA}$。

12. **是否有证据支持：P/PD导致？**  
    **UNSUPPORTED**。P/PD 在 $+38.14^\circ$ 总指令中仅贡献了 $+0.17^\circ$（不到 $0.5\%$），前馈占比 $>99.5\%$。

13. **是否有证据支持：ID5机械局部阻力？**  
    **POSSIBLE**。ID5 处于行程中段（1080 steps），移动极其缓慢，但电流却略高于深度压缩的 ID3。可能存在铰链转角极限摩擦或 ID5 丝杠局部阻力，但在缺乏力矩传感器的情况下无法与空间几何载荷完全分离。

14. **下一次大角度实验 应该控制哪个唯一变量？**  
    应该进行**单变量逐级递增的纯 Roll 姿态硬件阶梯测试（Feedforward-only）**：从 $+25.0^\circ$ 开始，以 $+2.5^\circ$ 为阶梯，依次测试 $+27.5^\circ$、$30.0^\circ$、$32.5^\circ$、$35.0^\circ$。每次阶梯保持 $3\,\text{s}$ 静止，记录六轴静态电流分布，精确找出电流跃升的几何临界拐点。

---

## 13. Generated Figures Catalog

The following high-resolution figures have been generated and saved to `/home/lyapunov/bone_drill/results/j12/analysis/overcurrent_165025/`:

1. **Figure 1**: [`figure1_base_orientation_so3.png`](file:///home/lyapunov/bone_drill/results/j12/analysis/overcurrent_165025/figure1_base_orientation_so3.png)  
   *Base Roll, Pitch, Yaw, and $SO(3)$ norm vs time, highlighting the $-38.11^\circ$ fault point.*
2. **Figure 2**: [`figure2_platform_attitude_error_so3.png`](file:///home/lyapunov/bone_drill/results/j12/analysis/overcurrent_165025/figure2_platform_attitude_error_so3.png)  
   *Platform Roll, Pitch, Yaw tracking errors and $SO(3)$ error vs time.*
3. **Figure 3**: [`figure3_motor_currents_all_axes.png`](file:///home/lyapunov/bone_drill/results/j12/analysis/overcurrent_165025/figure3_motor_currents_all_axes.png)  
   *All 6 motor currents (ID1–ID6) vs time, clearly marking the 500 mA overcurrent threshold and the multi-axis surge in ID3, ID4, and ID5.*
4. **Figure 4**: [`figure4_target_vs_feedback_all_axes.png`](file:///home/lyapunov/bone_drill/results/j12/analysis/overcurrent_165025/figure4_target_vs_feedback_all_axes.png)  
   *Commanded vs feedback steps for each of the 6 axes across the 21.4 s run.*
5. **Figure 5**: [`figure5_id5_detailed_dynamics.png`](file:///home/lyapunov/bone_drill/results/j12/analysis/overcurrent_165025/figure5_id5_detailed_dynamics.png)  
   *ID5 commanded steps, feedback steps, tracking error (15 steps), and current on a synchronized timeline.*
6. **Figure 6**: [`figure6_pid_residual_saturation.png`](file:///home/lyapunov/bone_drill/results/j12/analysis/overcurrent_165025/figure6_pid_residual_saturation.png)  
   *P/PD residual components (Roll, Pitch, Yaw) vs the $\pm 0.5^\circ$ limits, confirming non-saturation.*

---

## 14. Recommended Next Single-Variable Experiment (J12 Preparation)

> [!IMPORTANT]
> **DO NOT EXECUTE NOW.** (Read-only execution policy strictly preserved.)

When ready for J12 large-angle boundary exploration:
1. **Mode**: Feedforward-only (`--controller-mode feedforward` or offline Cartesian generator), eliminating PID feedback dynamics.
2. **Axis**: Pure Roll rotation only (Pitch = 0, Yaw = 0).
3. **Staircase Protocol**:
   - Level 1: $+25.0^\circ$ (hold 3.0 s)
   - Level 2: $+27.5^\circ$ (hold 3.0 s)
   - Level 3: $+30.0^\circ$ (hold 3.0 s, J10 boundary)
   - Level 4: $+32.5^\circ$ (hold 3.0 s)
   - Level 5: $+35.0^\circ$ (hold 3.0 s)
4. **Current Observation**: Monitor steady-state currents of ID3, ID4, ID5 at each plateau to verify whether the load scales linearly or exhibits non-linear mechanical binding.
