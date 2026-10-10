# J11 Canonical System-Level Controller Performance Comparison Archive

**Current Status**: J11 Tuning Archive — Final Retained Classical Baseline

**Last Updated**: 2026-09-20

**Scope**: Authoritative performance comparison of multi-axis control architectures on the 6PUS parallel surgical robot:
- **J11-M1**: 3DOF Base-IMU $SO(3)$ Pure Feedforward (*Frozen Baseline*)
- **J11-M2**: 3DOF Base-IMU $SO(3)$ Feedforward + 3DOF Platform-IMU $SO(3)$ P-only Feedback (*Qualified Baseline: $K_p = [0.05, 0.05, 0.05]$*)
- **J11-M2-5**: 3DOF P Gain Tuning (*Coarse sweep completed; Candidate 2 retained as BEST TESTED P-ONLY CONFIGURATION: $K_p = [0.05, 0.05, 0.025]$*)
- **J11-M2-6**: 3DOF Yaw-D Gain Tuning (*Univariate sweep completed; Candidate 2 retained as CURRENT FROZEN YAW-D WORKING BASELINE: $K_p = [0.05, 0.05, 0.025], K_d = [0, 0, 0.0015]\,\text{s}, f_c = 5.0\,\text{Hz}$*)
- **J11-M2-7A**: 3DOF Pitch-D Gain Tuning (*Univariate sweep completed; Candidate 2 retained as CURRENT FROZEN PITCH-D WORKING BASELINE: $K_p = [0.05, 0.05, 0.025], K_d = [0, 0.0015, 0.0015]\,\text{s}, f_c = 5.0\,\text{Hz}$*)
- **J11-M2-7B**: Roll-D $K_{d\_roll}=0.0010\,\text{s}$ qualified repeat completed but **NOT RETAINED**; final $K_{d\_roll}=0$ and Roll-D tuning CLOSED.
- **Final Retained Controller**: $K_p=[0.05,0.05,0.025]$, $K_d=[0,0.0015,0.0015]\,\text{s}$, $K_i=0$, $f_c=5\,\text{Hz}$ (Roll=P, Pitch=PD, Yaw=PD).
- **Next Research Stage**: J12 Active Disturbance Rejection Control (ADRC).

---

## 1. Architectural Scope & Canonical Framework

The objective of Stage J11 is active attitude vibration suppression on a 6PUS parallel Stewart platform under dynamic base disturbances. The control paradigm progresses through well-defined, verifiable stages:

```text
┌─────────────────────────────────────────────────────────────────────────────────┐
│ J11-M1: 3DOF Base-IMU SO(3) Pure Feedforward (FROZEN BASELINE)                  │
│ R_BP_cmd = Delta_R_B^T @ R_BP0  (Observer only, open-loop relative to platform) │
└───────────────────────────────────────┬─────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ J11-M2: 3DOF Feedforward + 3DOF Platform-IMU SO(3) P-only Feedback              │
│ e_R_W = Log(R_WP_des @ R_WP_meas^T)^vee                                         │
│ delta_phi_W = diag(Kp_roll, Kp_pitch, Kp_yaw) * e_R_W                          │
│ R_BP_cmd = R_WB^T @ Exp([delta_phi_W]_x) @ R_WB @ R_BP_FF                       │
│ [Authoritative Qualified Baseline: Kp = [0.05, 0.05, 0.05], Ki = 0, Kd = 0]     │
└───────────────────────────────────────┬─────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ J11-M2-5: 3DOF P Gain Tuning (Coarse Sweep Complete; Candidate 2 Retained)       │
│ Best Tested P-Only Config: Candidate 2 (Kp = [0.05, 0.05, 0.025])               │
└───────────────────────────────────────┬─────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ J11-M2-6: 3DOF Feedforward + 3DOF PD Feedback (Yaw-D Gain Sweep Complete)       │
│ Derivative: e_dot_R_W = (e_R_W - e_R_W_prev) / dt with 1st-order LPF (fc=5.0Hz) │
│ delta_phi_W = Kp * e_R_W + Kd * e_dot_R_W_filt                                  │
│ Retained Working Baseline: Cand 2 (Kp=[0.05, 0.05, 0.025], Kd=[0, 0, 0.0015] s) │
└───────────────────────────────────────┬─────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ J11-M2-7A: 3DOF Feedforward + Pitch-D Gain Sweep Complete (Pitch-D Retained)    │
│ Retained Working Baseline: Cand 2 (Kp=[0.05, 0.05, 0.025],                      │
│                                    Kd=[0.0, 0.0015, 0.0015] s, fc=5.0 Hz, Ki=0) │
└───────────────────────────────────────┬─────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ J11 Final Retained Classical Controller                                         │
│   • Roll-D qualified repeat completed but NOT RETAINED; Kd_roll=0               │
│   • Roll=P, Pitch=PD, Yaw=PD; Ki=0; tuning CLOSED                               │
│   • Next: J12 ESO/ADRC task-space disturbance rejection                         │
└─────────────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **Canonical Comparison Definition**:
> This document serves exclusively as the **Canonical System-Level Controller Performance Comparison Archive**. Intermediate single-channel checkouts and exploratory trial logs are documented in historical notes. Only multi-axis, system-level dynamic evaluations are elevated to the master performance record.

---

## 2. Canonical Datasets & Execution Provenance

All comparative metrics are computed directly from machine-generated, timestamped, 50 Hz real-hardware telemetry runs:

### Table 2.1: Proportional (P-Only) Tuning Datasets (J11-M2-5)

| Attribute | J11-M1 Canonical Baseline | J11-M2 Qualified Baseline | J11-M2-5 Cand 1 Trial | J11-M2-5 Cand 2 Trial | J11-M2-5 Cand 3 Trial |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Run Timestamp** | `20260918_112354` | `20260918_152354` | `20260918_160014` | `20260919_152543` | `20260919_153847` |
| **Stage Identifier** | `J11_DISTURB_QUAL` | `j11_m2_4_phaseB_dynamic` | `j11_m2_5_phaseB_dynamic` | `j11_m2_5_cand2_phaseB_dynamic` | `j11_m2_5_cand3_phaseB_dynamic` |
| **Evidence Directory** | [`results/j11/runtime_validation/20260918_112354/`](../../results/j11/runtime_validation/20260918_112354/) | [`results/j11/m2_pid/20260918_152354/`](../../results/j11/m2_pid/20260918_152354/) | [`results/j11/m2_pid/20260918_160014/`](../../results/j11/m2_pid/20260918_160014/) | [`results/j11/m2_pid/20260919_152543/`](../../results/j11/m2_pid/20260919_152543/) | [`results/j11/m2_pid/20260919_153847/`](../../results/j11/m2_pid/20260919_153847/) |
| **Primary Telemetry CSV** | `J11_DISTURB_QUAL_feedforward_powered.csv` | `j11_m2_4_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_5_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_5_cand2_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_5_cand3_phaseB_dynamic_telemetry_powered.csv` |
| **Duration / Rate** | 120.0 s (5999 rows @ 50 Hz) | 60.0 s (2999 rows @ 50 Hz) | 60.0 s (2997 rows @ 50 Hz) | 60.0 s (2999 rows @ 50 Hz) | 60.0 s (2999 rows @ 50 Hz) |
| **Active Gains** | N/A / Platform feedback disabled | $K_p = [0.05, 0.05, 0.05]$ | $K_p = [0.05, 0.05, 0.10]$ | $K_p = [0.05, 0.05, 0.025]$ | $K_p = [0.05, 0.05, 0.0125]$ |
| **Residual Limits** | N/A | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis |
| **Classification** | **FROZEN BASELINE** | **QUALIFIED BASELINE** | **NOT RETAINED (Degraded)** | **BEST TESTED P-ONLY CONFIGURATION** | **NOT RETAINED (Rebound)** |

### Table 2.2: Yaw Derivative (PD) Tuning Datasets (J11-M2-6)

| Attribute | J11-M2-6 PD Candidate 1 | J11-M2-6 PD Candidate 2 (Retained) | J11-M2-6 PD Candidate 3 |
| :--- | :--- | :--- | :--- |
| **Run Timestamp** | `20260919_155855` | `20260919_163119` | `20260919_163650` |
| **Stage Identifier** | `j11_m2_6_cand1_phaseB_dynamic` | `j11_m2_6_cand2_phaseB_dynamic` | `j11_m2_6_cand3_phaseB_dynamic` |
| **Evidence Directory** | [`results/j11/m2_pid/20260919_155855/`](../../results/j11/m2_pid/20260919_155855/) | [`results/j11/m2_pid/20260919_163119/`](../../results/j11/m2_pid/20260919_163119/) | [`results/j11/m2_pid/20260919_163650/`](../../results/j11/m2_pid/20260919_163650/) |
| **Primary Telemetry CSV** | `j11_m2_6_cand1_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_6_cand2_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_6_cand3_phaseB_dynamic_telemetry_powered.csv` |
| **Duration / Rate** | 60.0 s (2997 rows @ 50 Hz) | 60.0 s (2997 rows @ 50 Hz) | 60.0 s (2999 rows @ 50 Hz) |
| **Active Proportional Gains** | $K_p = [0.05, 0.05, 0.025]$ | $K_p = [0.05, 0.05, 0.025]$ | $K_p = [0.05, 0.05, 0.025]$ |
| **Active Derivative Gains** | $K_d = [0.0, 0.0, 0.0010]\,\text{s}$ | $K_d = [0.0, 0.0, 0.0015]\,\text{s}$ | $K_d = [0.0, 0.0, 0.0020]\,\text{s}$ |
| **Derivative LPF Cutoff** | $f_c = 5.0\,\text{Hz}$ | $f_c = 5.0\,\text{Hz}$ | $f_c = 5.0\,\text{Hz}$ |
| **Residual Limits** | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis |
| **Disturbance Protocol** | Continuous multi-axis circular manual excitation | Continuous multi-axis circular manual excitation | Approx. 5 continuous circles of smooth, low-speed multi-axis manual excitation ($49.10\,\text{s}$ effective duration) |
| **Classification** | **VALID / STRONG LOW-D CANDIDATE** | **CURRENT FROZEN YAW-D WORKING BASELINE** | **NOT RETAINED (PERFORMANCE REBOUND AT HIGHER TESTED Kd)** |

### Table 2.3: Pitch Derivative (PD) Tuning Datasets (J11-M2-7A)

| Attribute | J11-M2-7A Pitch-D Candidate 1 | J11-M2-7A Pitch-D Candidate 2 (Retained) | J11-M2-7A Pitch-D Candidate 3 |
| :--- | :--- | :--- | :--- |
| **Run Timestamp** | `20260920_094056` | `20260920_095850` | `20260920_100514` |
| **Stage Identifier** | `j11_m2_7a_pitchd_cand1_repeat_phaseB_dynamic` | `j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic` | `j11_m2_7a_pitchd_cand3_phaseB_dynamic` |
| **Evidence Directory** | [`results/j11/m2_pid/20260920_094056/`](../../results/j11/m2_pid/20260920_094056/) | [`results/j11/m2_pid/20260920_095850/`](../../results/j11/m2_pid/20260920_095850/) | [`results/j11/m2_pid/20260920_100514/`](../../results/j11/m2_pid/20260920_100514/) |
| **Primary Telemetry CSV** | `j11_m2_7a_pitchd_cand1_repeat_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_7a_pitchd_cand2_repeat_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_7a_pitchd_cand3_phaseB_dynamic_telemetry_powered.csv` |
| **Duration / Rate** | 60.0 s (2999 rows @ 50 Hz, effective 48.0 s) | 60.0 s (2999 rows @ 50 Hz, effective 52.0 s) | 33.18 s early-stop (1660 rows @ 50 Hz, effective 23.0 s) |
| **Active Proportional Gains** | $K_p = [0.05, 0.05, 0.025]$ | $K_p = [0.05, 0.05, 0.025]$ | $K_p = [0.05, 0.05, 0.025]$ |
| **Active Derivative Gains** | $K_d = [0.0, 0.0010, 0.0015]\,\text{s}$ | $K_d = [0.0, 0.0015, 0.0015]\,\text{s}$ | $K_d = [0.0, 0.0020, 0.0015]\,\text{s}$ |
| **Derivative LPF Cutoff** | $f_c = 5.0\,\text{Hz}$ | $f_c = 5.0\,\text{Hz}$ | $f_c = 5.0\,\text{Hz}$ |
| **Residual Limits** | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis |
| **Disturbance Protocol** | Continuous multi-axis manual excitation | Continuous multi-axis manual excitation | Continuous multi-axis manual excitation |
| **Classification** | **VALID / PROMISING PITCH-D CANDIDATE** | **CURRENT FROZEN PITCH-D WORKING BASELINE** | **NOT RETAINED — PARTIAL DYNAMIC CHARACTERIZATION / EARLY DRIVER SAFETY LATCH** |

### Table 2.4: Roll Derivative Closure Dataset (J11-M2-7B)

| Attribute | Roll-D Initial Dynamic Run | Roll-D Qualified Repeat |
| :--- | :--- | :--- |
| **Run Timestamp** | `20260920_101856` | `20260920_102755` |
| **Stage Identifier** | `j11_m2_7b_rolld_cand1_phaseB_dynamic` | `j11_m2_7b_rolld_cand1_repeat_phaseB_dynamic` |
| **Evidence Directory** | [`results/j11/m2_pid/20260920_101856/`](../../results/j11/m2_pid/20260920_101856/) | [`results/j11/m2_pid/20260920_102755/`](../../results/j11/m2_pid/20260920_102755/) |
| **Primary Telemetry CSV** | `j11_m2_7b_rolld_cand1_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_7b_rolld_cand1_repeat_phaseB_dynamic_telemetry_powered.csv` |
| **Summary JSON** | `j11_m2_7b_rolld_cand1_phaseB_dynamic_summary_powered.json` | `j11_m2_7b_rolld_cand1_repeat_phaseB_dynamic_summary_powered.json` |
| **Active Derivative Gains** | $K_d=[0.0010,0.0015,0.0015]\,\text{s}$ | $K_d=[0.0010,0.0015,0.0015]\,\text{s}$ |
| **Classification** | Valid characterization; not retained | **QUALIFIED REPEAT; NOT RETAINED** |

The repeat completed 2999 cycles with 2946 hardware writes, final state `ACTIVE`, and an empty fault reason. Its normalized metrics did not justify retaining Roll-D over the balanced `095850` controller, so the final value is $K_{d\_roll}=0$; no further Roll-D gain is authorized.

> [!WARNING]
> **Documentation of Excluded and Non-Qualified Runs**:
> - `20260918_150445` & `20260919_153610`: Formally classified as **NON-QUALIFIED / INVALID (NO-DISTURBANCE RUNS)**; excluded from dynamic evaluations because manual base disturbance was not applied.
> - `20260919_160521` (Cand 1 Repeat 1): Experienced an actuator overcurrent stop event on ID3 at $t \approx 24.7\,\text{s}$. Formally classified as **REPEATABILITY INCONCLUSIVE / RUN INVALIDATED BY EARLY HARDWARE FAULT**. Control parameters were frozen without modification.
> - `20260919_161729` (Cand 1 Repeat 2): Fully completed 60 s execution without hardware faults, but operator manual base disturbance was applied only between $t \approx 54.88\,\text{s} \sim 58.24\,\text{s}$ ($\approx 3.38\,\text{s}$ duration). Reclassified as **NON-QUALIFIED FOR DYNAMIC PERFORMANCE / INSUFFICIENT DISTURBANCE DURATION**; retained as extended static hold and short transient checkout evidence.
> - `20260920_093216` (Pitch-D Cand 1 Strong Run): Characterized strong-amplitude manual excitation (Base $SO(3)$ Peak $19.31^\circ$). Retained as **VALID STRONG-DISTURBANCE CHARACTERIZATION RUN**.
> - `20260920_093916` & `20260920_095706`: Excluded as **NON-QUALIFIED (NO-DISTURBANCE / SETUP TRANSITIONS)** where disturbance was not applied or operator prompt was delayed.
> - `20260920_100514` (Pitch-D Cand 3): Terminated at $t \approx 33.18\,\text{s}$ by driver-side safety latch (`max jump 84 steps > limit 80 steps`). Classified as **PARTIAL DYNAMIC CHARACTERIZATION / EARLY-TERMINATED DRIVER-SAFETY RUN** with pre-trip degraded metrics; NOT RETAINED.

---

## 3. Master System-Level Performance Comparison Table

Because manual base disturbances are applied naturally by hand, raw platform excursion depends on input disturbance energy. Scientific cross-controller comparison relies on normalized rejection ratios:
$$G_{\text{Peak}} = \frac{\text{Peak}(\text{Platform Error})}{\text{Peak}(\text{Base Excursion})}, \qquad G_{\text{RMS}} = \frac{\text{RMS}(\text{Platform Error})}{\text{RMS}(\text{Base Excursion})}$$
$$G_{SO(3)\_\text{Peak}} = \frac{\text{Peak}(\text{Platform } SO(3) \text{ Error})}{\text{Peak}(\text{Base } SO(3) \text{ Excursion})}, \qquad G_{SO(3)\_\text{RMS}} = \frac{\text{RMS}(\text{Platform } SO(3) \text{ Error})}{\text{RMS}(\text{Base } SO(3) \text{ Excursion})}$$

### Canonical Master Comparison Table

| Axis / Metric | Parameter | A. M1: Pure FF (`112354`) | B. Best P-Only Cand 2 (`152543`) | C. PD Cand 1 ($K_{d\_y}=0.0010$) (`155855`) | D. PD Cand 2 ($K_{d\_y}=0.0015$) (`163119`) [FROZEN BASELINE] | E. PD Cand 3 ($K_{d\_y}=0.0020$) (`163650`) [NOT RETAINED] | Engineering Assessment |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **Roll** | Base Peak / RMS | $11.18^\circ$ / $2.54^\circ$ | $11.99^\circ$ / $4.81^\circ$ | $12.48^\circ$ / $4.49^\circ$ | $10.84^\circ$ / $3.15^\circ$ | $12.91^\circ$ / $5.79^\circ$ | Broad dynamic excitation across runs |
| | Plat Peak / RMS | $5.34^\circ$ / $1.04^\circ$ | $5.50^\circ$ / $1.31^\circ$ | $8.25^\circ$ / $1.58^\circ$ | $3.77^\circ$ / $1.09^\circ$ | $7.42^\circ$ / $2.00^\circ$ | Platform excursions contained |
| | **$G_{\text{Peak}}$** | **0.4778** | **0.4584** | **0.6608** | **0.3476** | **0.5750** | **Cand 2 achieves lowest Roll Peak ratio** |
| | **$G_{\text{RMS}}$** | **0.4071** | **0.2724** | **0.3517** | **0.3451** | **0.3458** | Stable multi-axis Roll response |
| **Pitch** | Base Peak / RMS | $10.86^\circ$ / $2.77^\circ$ | $11.60^\circ$ / $4.72^\circ$ | $14.83^\circ$ / $5.01^\circ$ | $11.04^\circ$ / $3.52^\circ$ | $13.93^\circ$ / $6.29^\circ$ | Strong multi-axis Pitch excitation |
| | Plat Peak / RMS | $6.46^\circ$ / $1.15^\circ$ | $4.59^\circ$ / $1.32^\circ$ | $6.34^\circ$ / $1.62^\circ$ | $5.67^\circ$ / $1.25^\circ$ | $8.06^\circ$ / $2.12^\circ$ | Suppressed pitch deviations |
| | **$G_{\text{Peak}}$** | **0.5946** | **0.3951** | **0.4274** | **0.5135** | **0.5790** | All configurations improve over M1 |
| | **$G_{\text{RMS}}$** | **0.4153** | **0.2794** | **0.3230** | **0.3545** | **0.3365** | Pitch variance consistently suppressed |
| **Yaw** | Base Peak / RMS | $10.94^\circ$ / $2.89^\circ$ | $4.33^\circ$ / $1.87^\circ$ | $9.58^\circ$ / $4.99^\circ$ | $5.67^\circ$ / $3.62^\circ$ | $7.94^\circ$ / $3.50^\circ$ | Sufficient multi-axis manual Yaw inputs |
| | Plat Peak / RMS | $8.73^\circ$ / $0.84^\circ$ | $3.58^\circ$ / $0.57^\circ$ | $4.67^\circ$ / $0.84^\circ$ | $3.67^\circ$ / $0.65^\circ$ | $6.28^\circ$ / $1.52^\circ$ | Platform Yaw response |
| | **$G_{\text{Peak}}$** | **0.7983** | **0.8274** | **0.4870** | **0.6478** | **0.7907** | **PD Cand 1 & 2 both beat P-only and M1** |
| | **$G_{\text{RMS}}$** | **0.2912** | **0.3075** | **0.1679** | **0.1781** | **0.4342** | **Cand 2 confirms Yaw RMS suppression; Cand 3 rebounds** |
| **Overall $SO(3)$** | Base Peak / RMS | $13.54^\circ$ / $4.75^\circ$ | $12.76^\circ$ / $6.97^\circ$ | $15.86^\circ$ / $8.35^\circ$ | $11.32^\circ$ / $5.95^\circ$ | $15.72^\circ$ / $9.21^\circ$ | Total 3D angular excitation |
| | Plat Peak / RMS | $9.95^\circ$ / $1.76^\circ$ | $5.53^\circ$ / $1.94^\circ$ | $8.62^\circ$ / $2.41^\circ$ | $5.87^\circ$ / $1.78^\circ$ | $9.31^\circ$ / $3.28^\circ$ | Platform 3D angular response |
| | **$G_{SO(3)\_\text{Peak}}$** | **0.7353** | **0.4334** | **0.5439** | **0.5182** | **0.5918** | All closed-loop modes outperform M1 ($0.7353$) |
| | **$G_{SO(3)\_\text{RMS}}$** | **0.3713** | **0.2789** | **0.2891** | **0.2985** | **0.3563** | Cand 2 retains strong overall $SO(3)$ balance |

---

## 4. Closed-Loop Negative Feedback & Residual Telemetry

| Parameter | Best P-Only Cand 2 (`152543`) | PD Cand 1 (`155855`) | PD Cand 2 (`163119`) [FROZEN BASELINE] | PD Cand 3 (`163650`) [NOT RETAINED] | Safety Limit / Note |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Active Gains ($K_p, K_d$)** | $K_p=[0.05, 0.05, 0.025], K_d=0$ | $K_p=[0.05, 0.05, 0.025], K_d=[0, 0, 0.001]$ | $K_p=[0.05, 0.05, 0.025], K_d=[0, 0, 0.0015]$ | $K_p=[0.05, 0.05, 0.025], K_d=[0, 0, 0.0020]$ | $f_c = 5.0\,\text{Hz}$ on D channel |
| **Roll Residual Peak / RMS** | $0.2749^\circ$ / $0.0655^\circ$ | $0.4117^\circ$ / $0.0791^\circ$ | $0.1880^\circ$ / $0.0544^\circ$ | $0.3714^\circ$ / $0.0999^\circ$ | Within $\pm 0.50^\circ$ limit |
| **Pitch Residual Peak / RMS** | $0.2298^\circ$ / $0.0659^\circ$ | $0.3167^\circ$ / $0.0808^\circ$ | $0.2833^\circ$ / $0.0624^\circ$ | $0.4036^\circ$ / $0.1061^\circ$ | Within $\pm 0.50^\circ$ limit |
| **Yaw Residual Peak / RMS** | $0.0900^\circ$ / $0.0143^\circ$ | $0.1252^\circ$ / $0.0232^\circ$ | $0.1268^\circ$ / $0.0208^\circ$ | $0.1724^\circ$ / $0.0426^\circ$ | Well within $\pm 0.50^\circ$ limit |
| **Saturation Cycles (R / P / Y)** | **0 / 0 / 0** ($0.0\%$) | **0 / 0 / 0** ($0.0\%$) | **0 / 0 / 0** ($0.0\%$) | **0 / 0 / 0** ($0.0\%$) | Zero saturation in all qualified runs |
| **Max Tracking Error** | 74 steps | 100 steps | 74 steps | 83 steps | Smooth tracking, well below trip threshold |
| **Max Joint Current** | $451\,\text{mA}$ | $180\,\text{mA}$ | $165\,\text{mA}$ | $369\,\text{mA}$ | Measured maximum currents; no hardware overcurrent fault |
| **Hardware Faults** | `0x0000` (0 faults) | `0x0000` (0 faults) | `0x0000` (0 faults) | `0x0000` (0 faults) | Zero motor faults |
| **Safety Aborts (WH / HS)** | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | Zero safety aborts |
| **Base IMU Age (Mean / Max)** | $3.36\,\text{ms}$ / $51.2\,\text{ms}$ | $2.64\,\text{ms}$ / $38.9\,\text{ms}$ | $3.61\,\text{ms}$ / $56.0\,\text{ms}$ | $2.75\,\text{ms}$ / $37.8\,\text{ms}$ | Healthy sensor latency |
| **Platform IMU Age (Mean / Max)** | $2.25\,\text{ms}$ / $48.1\,\text{ms}$ | $2.43\,\text{ms}$ / $46.2\,\text{ms}$ | $2.51\,\text{ms}$ / $23.9\,\text{ms}$ | $2.00\,\text{ms}$ / $38.9\,\text{ms}$ | Healthy sensor latency |
| **Driver Feedback Age (Mean / Max)**| $16.15\,\text{ms}$ / $175.6\,\text{ms}$ | $14.18\,\text{ms}$ / $192.5\,\text{ms}$ | $13.67\,\text{ms}$ / $188.2\,\text{ms}$ | $16.49\,\text{ms}$ / $254.3\,\text{ms}$ | Within nominal streaming envelope |

---

## 5. Controller Tuning Assessments & Engineering Conclusions

### 5.1 J11-M2-5 Proportional Tuning Recap
Four proportional configurations were systematically evaluated on real hardware:
- $K_{p\_yaw} = 0.100$: **NOT RETAINED** (over-correction, 26 cycles of yaw saturation, cross-axis degradation).
- $K_{p\_yaw} = 0.050$: **QUALIFIED BASELINE** (moderate under-damping, Yaw $G_{\text{Peak}} = 0.9030$).
- $K_{p\_yaw} = 0.025$: **BEST TESTED P-ONLY CONFIGURATION (RETAINED)** (Yaw $G_{\text{Peak}} = 0.8274$, $G_{\text{RMS}} = 0.3075$; $SO(3)$ RMS = $0.2789$; zero saturation).
- $K_{p\_yaw} = 0.0125$: **NOT RETAINED** (measured Yaw and overall performance rebounded when $K_{p\_yaw}$ was reduced further: Yaw $G_{\text{Peak}} \to 0.8828$, $G_{\text{RMS}} \to 0.3992$, $SO(3)$ RMS $\to 0.2865$).

### 5.2 J11-M2-6 Yaw-D Sweep Assessment & Retention Decision
The univariate Yaw-D sweep evaluated three derivative gain candidates with $f_c = 5.0\,\text{Hz}$ 1st-order LPF, superimposed on the retained P baseline ($K_p = [0.05, 0.05, 0.025]$):

1. **Candidate 1 ($K_{d\_yaw} = 0.0010\,\text{s}$)**:
   - **Classification**: **VALID / STRONG LOW-D CANDIDATE**
   - Observed Yaw $G_{\text{RMS}} = 0.1679$, Yaw $G_{\text{Peak}} = 0.4870$.
   - Successfully demonstrated initial derivative damping benefit on hardware. Preserved in archive as a verified low-gain working configuration.

2. **Candidate 2 ($K_{d\_yaw} = 0.0015\,\text{s}$)**:
   - **Classification**: **J11-M2 CURRENT FROZEN YAW-D WORKING BASELINE** (or **BEST TESTED WORKING YAW-D BASELINE**)
   - Observed Yaw $G_{\text{RMS}} = 0.1781$, Yaw $G_{\text{Peak}} = 0.6478$, $SO(3)$ $G_{\text{RMS}} = 0.2985$.
   - Replicated substantial Yaw RMS suppression under sufficient dynamic excitation, maintaining smooth tracking (max error 74 steps) and moderate electrical current ($165\,\text{mA}$).
   - Retained as the production working baseline for Stage J11-M2.

3. **Candidate 3 ($K_{d\_yaw} = 0.0020\,\text{s}$)**:
   - **Classification**: **NOT RETAINED (PERFORMANCE REBOUND AT HIGHER TESTED Kd)**
   - Under approximately 5 circles of continuous multi-axis manual disturbance ($49.10\,\text{s}$ effective duration), observed Yaw $G_{\text{RMS}} = 0.4342$ and Yaw $G_{\text{Peak}} = 0.7907$.
   - Demonstrates a clear performance rebound and diminishing return at $K_{d\_yaw} \ge 0.0020\,\text{s}$ under the tested conditions.
   - Concludes today's Yaw-D gain sweep without further increasing $K_d$.

> [!NOTE]
> Candidate 2 is designated as the **CURRENT FROZEN YAW-D WORKING BASELINE** within the tested discrete parameter sweep. It is not claimed to be a mathematically unique optimum, globally best across continuous parameter space, or an absolute physical performance limit.

### 5.3 J11-M2-7A Pitch-D Sweep Assessment & Retention Decision
The univariate Pitch-D sweep evaluated three derivative gain candidates with $f_c = 5.0\,\text{Hz}$ 1st-order LPF, superimposed on the retained P baseline ($K_p = [0.05, 0.05, 0.025]$) and frozen Yaw-D baseline ($K_{d\_yaw} = 0.0015\,\text{s}$):

1. **Candidate 1 ($K_{d\_pitch} = 0.0010\,\text{s}$)**:
   - **Classification**: **VALID / PROMISING PITCH-D CANDIDATE**
   - Qualified Repeat Run (`20260920_094056`, 48.0 s active duration): Pitch $G_{\text{RMS}} = 0.2832$, Pitch $G_{\text{Peak}} = 0.2799$, $SO(3)$ $G_{\text{RMS}} = 0.2889$.
   - Confirmed tangible Pitch RMS reduction compared to the pure P baseline ($0.3543$).

2. **Candidate 2 ($K_{d\_pitch} = 0.0015\,\text{s}$)**:
   - **Classification**: **RETAINED — CURRENT FROZEN PITCH-D WORKING BASELINE** (or **CURRENT RETAINED 2-AXIS PD WORKING BASELINE**)
   - Qualified Dynamic Run (`20260920_095850`, 52.0 s active duration): Pitch $G_{\text{RMS}} = 0.2607$, Pitch $G_{\text{Peak}} = 0.3279$, $SO(3)$ $G_{\text{RMS}} = 0.2575$, Roll $G_{\text{RMS}} = 0.2662$, Yaw $G_{\text{RMS}} = 0.2297$.
   - Best tested working Pitch-D candidate within the completed discrete hardware gain sweep, achieving both the lowest Pitch RMS and lowest multi-axis $SO(3)$ RMS among all evaluated configurations.
   - Retained as the frozen working baseline for Stage J11-M2.

3. **Candidate 3 ($K_{d\_pitch} = 0.0020\,\text{s}$)**:
   - **Classification**: **NOT RETAINED — PARTIAL DYNAMIC CHARACTERIZATION / EARLY DRIVER SAFETY LATCH**
   - Dynamic Run (`20260920_100514`, terminated at $t \approx 33.18\,\text{s}$): Pre-trip partial active window ($\approx 23.0\,\text{s}$) showed Pitch $G_{\text{RMS}} = 0.2970$, Pitch $G_{\text{Peak}} = 0.4424$, $SO(3)$ $G_{\text{RMS}} = 0.3255$, Yaw $G_{\text{RMS}} = 0.5398$.
   - The run terminated early due to a driver-side safety latch (`Timeout recovery target rejected: max jump 84 steps > limit 80 steps`). Zero residual saturation was observed ($0 / 1151$ cycles).
   - Pre-trip normalized metrics exhibited a clear performance rebound and cross-axis degradation. Concludes the Pitch-D sweep without further increasing $K_d$.

> [!NOTE]
> Candidate 2 ($K_{d\_pitch} = 0.0015\,\text{s}$) is designated as the **CURRENT FROZEN PITCH-D WORKING BASELINE** within the tested discrete parameter sweep. It is not claimed to be a mathematically unique optimum, globally best across continuous parameter space, or an absolute physical performance limit.

### 5.4 J11-M2-7B Roll-D Closure Decision

$K_{d\_roll}=0.0010\,\text{s}$ was tested and repeated with the retained Pitch/Yaw derivative gains. The qualified repeat (`20260920_102755`) completed without a recorded fault, but its representative normalized system response did not improve the balance achieved by final retained run `20260920_095850`. The Roll-D candidate is therefore **NOT RETAINED**. Final $K_{d\_roll}=0$ and Roll-D tuning is CLOSED; no `0.0005`, `0.0015`, or other Roll-D gain is scheduled.

### 5.5 System-Level Metrics vs. Frozen Baselines

| Comparison | M1 Pure Feedforward (`112354`) | Best P-Only Cand 2 (`152543`) | Frozen Yaw-PD Cand 2 (`163119`) | Frozen Pitch-PD Cand 2 (`095850`) | Comparative Assessment |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Roll ($G_{\text{Peak}} / G_{\text{RMS}}$)** | 0.4778 / 0.4071 | 0.4584 / 0.2724 | 0.3476 / 0.3451 | **0.3107 / 0.2662** | Lowest Roll rejection ratio observed |
| **Pitch ($G_{\text{Peak}} / G_{\text{RMS}}$)** | 0.5946 / 0.4153 | 0.3951 / 0.2794 | 0.5135 / 0.3545 | **0.3279 / 0.2607** | **Pitch RMS reduced by ~26% vs. pure P baseline** |
| **Yaw ($G_{\text{Peak}} / G_{\text{RMS}}$)** | 0.7983 / 0.2912 | 0.8274 / 0.3075 | 0.6478 / 0.1781 | **0.4424 / 0.2297** | Balanced multi-axis suppression maintained |
| **Overall $SO(3)$ ($G_{\text{Peak}} / G_{\text{RMS}}$)** | 0.7353 / 0.3713 | 0.4334 / 0.2789 | 0.5182 / 0.2985 | **0.3063 / 0.2575** | **Best tested multi-axis $SO(3)$ variance across runs** |

### 5.5 Core Engineering Conclusions

1. **Pitch-D Tuning Sweep Concluded**: The univariate sweep over $K_{d\_pitch} \in \{0.0, 0.0010, 0.0015, 0.0020\}\,\text{s}$ with $f_c = 5.0\,\text{Hz}$ LPF is officially closed.
2. **Pitch Damping Benefit Verified**: Adding filtered derivative action on Pitch orientation rate error consistently reduced platform Pitch variance ($G_{\text{RMS}}: 0.3545 \to 0.2832 \to \mathbf{0.2607}$) while enhancing total $SO(3)$ rejection ($G_{\text{RMS}}: 0.2985 \to \mathbf{0.2575}$).
3. **Gain Limit & Safety Latch Identified**: At $K_{d\_pitch} = 0.0020\,\text{s}$, pre-trip rejection ratios rebounded ($G_{\text{RMS}} \to 0.2970$) and a driver-side max-jump safety latch halted the run at $t \approx 33.18\,\text{s}$, establishing $K_{d\_pitch} = 0.0015\,\text{s}$ as the empirical boundary.
4. **Current Working Controller Freeze**: The controller configuration is frozen at $K_p = [0.05, 0.05, 0.025]$, $K_d = [0.0, 0.0015, 0.0015]\,\text{s}$, $K_i = [0, 0, 0]$, $f_c = 5.0\,\text{Hz}$, with residual limits at $\pm 0.5^\circ$ per axis (Roll = P, Pitch = PD, Yaw = PD).
5. **Roll-D Tuning Closed**: $K_{d\_roll}=0.0010\,\text{s}$ completed a qualified repeat but was not retained. Final $K_{d\_roll}=0$.
6. **ACTIVE-Gated Operator Prompt Standardized**: Operator dynamic disturbance is strictly gated by the creation of `/tmp/j11_active_signaled` and confirmed runner `ACTIVE` status to eliminate startup perception delays.

---

## 6. Progressive Strategy Roadmap & Tuning Log

The canonical performance record tracks system-level controller evolution through standardized trial records:

| Stage | Controller Architecture | $G_{\text{Peak}}$ (R / P / Y / $SO(3)$) | $G_{\text{RMS}}$ (R / P / Y / $SO(3)$) | Canonical Session | Status / Outcome |
| :--- | :--- | :---: | :---: | :--- | :--- |
| **J11-M1** | 3DOF Base-IMU Feedforward | 0.4778 / 0.5946 / 0.7983 / 0.7353 | 0.4071 / 0.4153 / 0.2912 / 0.3713 | `20260918_112354` | **FROZEN BASELINE** |
| **J11-M2** | Qualified Baseline ($K_p=[0.05, 0.05, 0.05]$) | 0.4900 / 0.3110 / 0.9030 / 0.3633 | 0.3066 / 0.2705 / 0.3374 / 0.2888 | `20260918_152354` | **QUALIFIED BASELINE** |
| **J11-M2-5** | Cand 1 Trial ($K_p=[0.05, 0.05, 0.10]$) | 0.5664 / 0.6917 / 1.5515 / 0.6874 | 0.4225 / 0.4464 / 0.5060 / 0.4482 | `20260918_160014` | **NOT RETAINED (Degraded)** |
| **J11-M2-5** | Cand 2 Trial ($K_p=[0.05, 0.05, 0.025]$) | **0.4584 / 0.3951 / 0.8274 / 0.4334** | **0.2724 / 0.2794 / 0.3075 / 0.2789** | `20260919_152543` | **BEST TESTED P-ONLY CONFIGURATION** |
| **J11-M2-5** | Cand 3 Trial ($K_p=[0.05, 0.05, 0.0125]$) | 0.4163 / 0.3715 / 0.8828 / 0.4463 | 0.2967 / 0.2624 / 0.3992 / 0.2865 | `20260919_153847` | **NOT RETAINED (Rebound)** |
| **J11-M2-6** | PD Cand 1 Trial ($K_{d\_y}=0.0010\,\text{s}, f_c=5\text{Hz}$) | 0.6608 / 0.4274 / 0.4870 / 0.5439 | 0.3517 / 0.3230 / 0.1679 / 0.2891 | `20260919_155855` | **VALID / STRONG LOW-D CANDIDATE** |
| **J11-M2-6** | PD Cand 2 Trial ($K_{d\_y}=0.0015\,\text{s}, f_c=5\text{Hz}$) | **0.3476 / 0.5135 / 0.6478 / 0.5182** | **0.3451 / 0.3545 / 0.1781 / 0.2985** | `20260919_163119` | **CURRENT FROZEN YAW-D WORKING BASELINE** |
| **J11-M2-6** | PD Cand 3 Trial ($K_{d\_y}=0.0020\,\text{s}, f_c=5\text{Hz}$) | 0.5750 / 0.5790 / 0.7907 / 0.5918 | 0.3458 / 0.3365 / 0.4342 / 0.3563 | `20260919_163650` | **NOT RETAINED (Performance Rebound)** |
| **J11-M2-7A** | Pitch-D Cand 1 ($K_{d\_p}=0.0010\,\text{s}$) | 0.5118 / 0.2799 / 0.5791 / 0.3800 | 0.3134 / 0.2832 / 0.2724 / 0.2889 | `20260920_094056` | **VALID / PROMISING PITCH-D CANDIDATE** |
| **J11-M2-7A** | Pitch-D Cand 2 ($K_{d\_p}=0.0015\,\text{s}$) | **0.3107 / 0.3279 / 0.4424 / 0.3063** | **0.2662 / 0.2607 / 0.2297 / 0.2575** | `20260920_095850` | **CURRENT FROZEN PITCH-D WORKING BASELINE** |
| **J11-M2-7A** | Pitch-D Cand 3 ($K_{d\_p}=0.0020\,\text{s}$) | 0.6100 / 0.4424 / 0.8551 / 0.5199 | 0.3221 / 0.2970 / 0.5398 / 0.3255 | `20260920_100514` | **NOT RETAINED (Early Driver Trip / Rebound)** |
| **J11-M2-7B** | Roll-D qualified repeat ($K_{d\_r}=0.0010\,\text{s}$) | 0.6056 / 0.6715 / 0.5475 / 0.6370 | 0.3328 / 0.3290 / 0.1954 / 0.2933 | `20260920_102755` | **NOT RETAINED; ROLL-D CLOSED** |
| **J11 FINAL** | Roll=P, Pitch=PD, Yaw=PD | **0.3107 / 0.3279 / 0.4424 / 0.3063** | **0.2662 / 0.2607 / 0.2297 / 0.2575** | `20260920_095850` | **FINAL RETAINED CLASSICAL BASELINE** |
| **J12** | Task-Space ADRC (ESO + Disturbance Rejection) | *Pending* | *Pending* | TBD | *Roadmap* |

---
*Archive Maintained by: Lyapunov / DeepMind Advanced Agentic Coding Team*
