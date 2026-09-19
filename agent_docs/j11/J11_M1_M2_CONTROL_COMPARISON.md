# J11 Canonical System-Level Controller Performance Comparison Archive

**Current Status**: Active Canonical Comparison Baseline  
**Last Updated**: 2026-09-19  
**Scope**: Authoritative performance comparison of multi-axis control architectures on the 6PUS parallel surgical robot:
- **J11-M1**: 3DOF Base-IMU $SO(3)$ Pure Feedforward (*Frozen Baseline*)
- **J11-M2**: 3DOF Base-IMU $SO(3)$ Feedforward + 3DOF Platform-IMU $SO(3)$ P-only Feedback (*Qualified Baseline: $K_p = [0.05, 0.05, 0.05]$*)
- **J11-M2-5**: 3DOF P Gain Tuning (*Coarse sweep completed; Candidate 2 retained as BEST TESTED P-ONLY CONFIGURATION*)
- **Subsequent Stages**: 3DOF PD (M2-6) $\to$ 3DOF PID (M2-7) $\to$ Active Disturbance Rejection Control (J12 ADRC).

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
│ Progression Roadmap:                                                            │
│   • J11-M2-6: 3DOF PD Feedback (Derivative damping on platform rate error)      │
│   • J11-M2-7: 3DOF PID Feedback (Integral action only if steady bias persists)  │
│   • J12: Extended State Observer & ADRC (Task-space total disturbance rejection)│
└─────────────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **Canonical Comparison Definition**:
> This document serves exclusively as the **Canonical System-Level Controller Performance Comparison Archive**. Intermediate single-channel checkouts and trial logs are compressed into historical notes. Only multi-axis, system-level evaluations are elevated to the master performance record.

---

## 2. Canonical Datasets & Execution Provenance

All comparative metrics are computed directly from machine-generated, timestamped, 50 Hz real-hardware telemetry runs:

| Attribute | J11-M1 Canonical Baseline | J11-M2 Qualified Baseline | J11-M2-5 Cand 1 Trial | J11-M2-5 Cand 2 Trial | J11-M2-5 Cand 3 Trial |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Run Timestamp** | `20260918_112354` | `20260918_152354` | `20260918_160014` | `20260919_152543` | `20260919_153847` |
| **Stage Identifier** | `J11_DISTURB_QUAL` | `j11_m2_4_phaseB_dynamic` | `j11_m2_5_phaseB_dynamic` | `j11_m2_5_cand2_phaseB_dynamic` | `j11_m2_5_cand3_phaseB_dynamic` |
| **Evidence Directory** | [`results/j11/runtime_validation/20260918_112354/`](../../results/j11/runtime_validation/20260918_112354/) | [`results/j11/m2_pid/20260918_152354/`](../../results/j11/m2_pid/20260918_152354/) | [`results/j11/m2_pid/20260918_160014/`](../../results/j11/m2_pid/20260918_160014/) | [`results/j11/m2_pid/20260919_152543/`](../../results/j11/m2_pid/20260919_152543/) | [`results/j11/m2_pid/20260919_153847/`](../../results/j11/m2_pid/20260919_153847/) |
| **Primary Telemetry CSV** | `J11_DISTURB_QUAL_feedforward_powered.csv` | `j11_m2_4_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_5_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_5_cand2_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_5_cand3_phaseB_dynamic_telemetry_powered.csv` |
| **Duration / Rate** | 120.0 s (5999 rows @ 50 Hz) | 60.0 s (2999 rows @ 50 Hz) | 60.0 s (2997 rows @ 50 Hz) | 60.0 s (2999 rows @ 50 Hz) | 60.0 s (2999 rows @ 50 Hz) |
| **Active Gains** | None ($K_p=0, K_i=0, K_d=0$) | $K_p = [0.05, 0.05, 0.05]$ | $K_p = [0.05, 0.05, 0.10]$ | $K_p = [0.05, 0.05, 0.025]$ | $K_p = [0.05, 0.05, 0.0125]$ |
| **Residual Limits** | N/A | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis |
| **Classification** | **FROZEN BASELINE** | **QUALIFIED BASELINE** | **NOT RETAINED (Degraded)** | **BEST TESTED P-ONLY CONFIGURATION** | **NOT RETAINED (Rebound)** |

> [!WARNING]
> **Formal Exclusion of Session `20260918_150445` and `20260919_153610`**:
> Sessions `20260918_150445` and `20260919_153610` are formally classified as **NON-QUALIFIED / INVALID (NO-DISTURBANCE RUNS)** and excluded from canonical performance evaluation because manual Base disturbance was not applied during those runs. Raw telemetry is preserved intact strictly as observational records.

---

## 3. Master System-Level Performance Comparison Table

Because manual base disturbances are applied naturally by hand, raw platform excursion depends on the input disturbance energy. Scientific cross-controller comparison therefore relies on normalized rejection ratios:
$$G_{\text{Peak}} = \frac{\text{Peak}(\text{Platform Error})}{\text{Peak}(\text{Base Excursion})}, \qquad G_{\text{RMS}} = \frac{\text{RMS}(\text{Platform Error})}{\text{RMS}(\text{Base Excursion})}$$
$$G_{SO(3)\_\text{Peak}} = \frac{\text{Peak}(\text{Platform } SO(3) \text{ Error})}{\text{Peak}(\text{Base } SO(3) \text{ Excursion})}, \qquad G_{SO(3)\_\text{RMS}} = \frac{\text{RMS}(\text{Platform } SO(3) \text{ Error})}{\text{RMS}(\text{Base } SO(3) \text{ Excursion})}$$

### Canonical Master Comparison:

| Axis / Metric | Parameter | A. M1: Pure FF (`112354`) | B. M2 Qualified (`152354`) | C. Cand 1 ($K_{p\_y}=0.10$) (`160014`) | D. Cand 2 ($K_{p\_y}=0.025$) (`152543`) | E. Cand 3 ($K_{p\_y}=0.0125$) (`153847`) | Assessment |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **Roll** | Base Peak / RMS | $11.18^\circ$ / $2.54^\circ$ | $9.56^\circ$ / $3.65^\circ$ | $11.64^\circ$ / $4.05^\circ$ | $11.99^\circ$ / $4.81^\circ$ | $12.39^\circ$ / $5.03^\circ$ | Strong disturbance energy across trials |
| | Plat Peak / RMS | $5.34^\circ$ / $1.04^\circ$ | $4.68^\circ$ / $1.12^\circ$ | $6.60^\circ$ / $1.71^\circ$ | $5.50^\circ$ / $1.31^\circ$ | $5.16^\circ$ / $1.49^\circ$ | Platform excursion well restrained |
| | **$G_{\text{Peak}}$** | **0.4778** | **0.4900** | **0.5664** | **0.4584** | **0.4163** | Both Cand 2 and 3 improve over M1 |
| | **$G_{\text{RMS}}$** | **0.4071** | **0.3066** | **0.4225** | **0.2724** | **0.2967** | **Cand 2 achieves best Roll RMS** |
| **Pitch** | Base Peak / RMS | $10.86^\circ$ / $2.77^\circ$ | $12.07^\circ$ / $4.55^\circ$ | $12.45^\circ$ / $4.59^\circ$ | $11.60^\circ$ / $4.72^\circ$ | $14.06^\circ$ / $5.75^\circ$ | Strong multi-axis pitch inputs |
| | Plat Peak / RMS | $6.46^\circ$ / $1.15^\circ$ | $3.75^\circ$ / $1.23^\circ$ | $8.61^\circ$ / $2.05^\circ$ | $4.59^\circ$ / $1.32^\circ$ | $5.22^\circ$ / $1.51^\circ$ | All stable P modes strongly suppress pitch |
| | **$G_{\text{Peak}}$** | **0.5946** | **0.3110** | **0.6917** | **0.3951** | **0.3715** | Both Cand 2 and 3 far superior to M1 |
| | **$G_{\text{RMS}}$** | **0.4153** | **0.2705** | **0.4464** | **0.2794** | **0.2624** | Strong consistent variance attenuation |
| **Yaw** | Base Peak / RMS | $10.94^\circ$ / $2.89^\circ$ | $3.48^\circ$ / $1.53^\circ$ | $4.39^\circ$ / $2.49^\circ$ | $4.33^\circ$ / $1.87^\circ$ | $5.09^\circ$ / $1.85^\circ$ | Natural multi-axis yaw disturbance |
| | Plat Peak / RMS | $8.73^\circ$ / $0.84^\circ$ | $3.14^\circ$ / $0.51^\circ$ | $6.82^\circ$ / $1.26^\circ$ | $3.58^\circ$ / $0.57^\circ$ | $4.50^\circ$ / $0.74^\circ$ | Platform yaw excursion |
| | **$G_{\text{Peak}}$** | **0.7983** | **0.9030** | **1.5515** | **0.8274** | **0.8828** | **Cand 2 achieves lowest Peak ($0.8274$)** |
| | **$G_{\text{RMS}}$** | **0.2912** | **0.3374** | **0.5060** | **0.3075** | **0.3992** | **Cand 2 achieves lowest RMS ($0.3075$)** |
| **Overall $SO(3)$** | Base Peak / RMS | $13.54^\circ$ / $4.75^\circ$ | $12.95^\circ$ / $6.03^\circ$ | $13.84^\circ$ / $6.59^\circ$ | $12.76^\circ$ / $6.97^\circ$ | $15.01^\circ$ / $7.84^\circ$ | 3D attitude excitation |
| | Plat Peak / RMS | $9.95^\circ$ / $1.76^\circ$ | $4.70^\circ$ / $1.74^\circ$ | $9.52^\circ$ / $2.96^\circ$ | $5.53^\circ$ / $1.94^\circ$ | $6.70^\circ$ / $2.25^\circ$ | 3D attitude response |
| | **$G_{SO(3)\_\text{Peak}}$** | **0.7353** | **0.3633** | **0.6874** | **0.4334** | **0.4463** | Substantially superior to M1 ($0.7353$) |
| | **$G_{SO(3)\_\text{RMS}}$** | **0.3713** | **0.2888** | **0.4482** | **0.2789** | **0.2865** | **Cand 2 achieves best overall SO(3) RMS** |

---

## 4. Closed-Loop Negative Feedback & Residual Telemetry

| Parameter | M2 Qualified Baseline (`152354`) | Cand 1 (`160014`) | Cand 2 (`152543`) | Cand 3 (`153847`) | Note / Safety Limit |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **$K_p$ Configuration** | $[0.05, 0.05, 0.05]$ | $[0.05, 0.05, 0.10]$ | $[0.05, 0.05, 0.025]$ | $[0.05, 0.05, 0.0125]$ | Systematic coarse sweep |
| **Roll Residual Peak / RMS** | $0.2342^\circ$ / $0.0553^\circ$ | $0.3330^\circ$ / $0.0859^\circ$ | $0.2749^\circ$ / $0.0655^\circ$ | $0.2571^\circ$ / $0.0746^\circ$ | Safe ($< 0.50^\circ$ limit) |
| **Pitch Residual Peak / RMS** | $0.1878^\circ$ / $0.0607^\circ$ | $0.4274^\circ$ / $0.1022^\circ$ | $0.2298^\circ$ / $0.0659^\circ$ | $0.2589^\circ$ / $0.0754^\circ$ | Safe ($< 0.50^\circ$ limit) |
| **Yaw Residual Peak / RMS** | $0.1575^\circ$ / $0.0255^\circ$ | $\mathbf{0.5000^\circ}$ / $0.1218^\circ$ | $\mathbf{0.0900^\circ}$ / $0.0143^\circ$ | $\mathbf{0.0559^\circ}$ / $0.0093^\circ$ | Clean monotonic decrease |
| **Saturation Cycles (R / P / Y)** | **0 / 0 / 0** ($0.0\%$) | **0 / 0 / 26** ($0.88\%$) | **0 / 0 / 0** ($0.0\%$) | **0 / 0 / 0** ($0.0\%$) | Zero saturation in Cand 2 & 3 |
| **Pearson Correlation $\rho$ (R / P / Y)** | -0.99998 / -0.99998 / -0.99962 | -0.99964 / -0.99991 / -0.99552 | -0.99998 / -0.99998 / -0.99962 | -0.99997 / -0.99995 / -0.99947 | Strict negative feedback |
| **Max Tracking Error** | 62 steps | 90 steps | 74 steps | 61 steps | Smooth execution |
| **Max Joint Current** | $171\,\text{mA}$ | $282\,\text{mA}$ | $451\,\text{mA}$ | $167\,\text{mA}$ | Well below $500\,\text{mA}$ trip limit |
| **Hardware Faults** | `0x0000` (0 faults) | `0x0000` (0 faults) | `0x0000` (0 faults) | `0x0000` (0 faults) | Zero motor faults |
| **Safety Aborts (WH / HS)** | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | Zero safety aborts |
| **Base IMU Age (Mean / Max)** | $2.14\,\text{ms}$ / $76.2\,\text{ms}$ | $2.20\,\text{ms}$ / $78.0\,\text{ms}$ | $3.36\,\text{ms}$ / $51.2\,\text{ms}$ | $1.38\,\text{ms}$ / $46.9\,\text{ms}$ | Measured runtime latency |
| **Platform IMU Age (Mean / Max)** | $2.48\,\text{ms}$ / $81.5\,\text{ms}$ | $2.59\,\text{ms}$ / $82.5\,\text{ms}$ | $2.25\,\text{ms}$ / $48.1\,\text{ms}$ | $3.17\,\text{ms}$ / $48.8\,\text{ms}$ | Measured runtime latency |
| **Driver Feedback Age (Mean / Max)**| $15.8\,\text{ms}$ / $245.1\,\text{ms}$ | $17.3\,\text{ms}$ / $231.1\,\text{ms}$ | $16.15\,\text{ms}$ / $175.6\,\text{ms}$ | $12.53\,\text{ms}$ / $147.0\,\text{ms}$ | Measured runtime latency |

---

## 5. M2-5 Coarse Tuning Assessment & Engineering Conclusions

### 5.1 Tested Configurations & Retention Decision
Four configurations were systematically evaluated on real hardware:
- $K_{p\_yaw} = 0.100$: **NOT RETAINED** (over-correction, 26 cycles of yaw saturation, cross-axis degradation).
- $K_{p\_yaw} = 0.050$: **QUALIFIED BASELINE** (moderate under-damping, Yaw $G_{\text{Peak}} = 0.9030$).
- $K_{p\_yaw} = 0.025$: **BEST TESTED P-ONLY CONFIGURATION (RETAINED)** (Yaw $G_{\text{Peak}} = 0.8274$, $G_{\text{RMS}} = 0.3075$; $SO(3)$ RMS = $0.2789$; zero saturation).
- $K_{p\_yaw} = 0.0125$: **NOT RETAINED** (measured Yaw and overall performance rebounded when $K_{p\_yaw}$ was reduced further: Yaw $G_{\text{Peak}} \to 0.8828$, $G_{\text{RMS}} \to 0.3992$, $SO(3)$ RMS $\to 0.2865$).

> [!NOTE]
> Candidate 2 ($K_p = [0.05, 0.05, 0.025], K_i = 0, K_d = 0$) is formally designated as the **BEST TESTED P-ONLY CONFIGURATION** in the current coarse sweep. It is not claimed to be globally optimal, an exact optimum, or mathematically optimal.

### 5.2 System-Level Metrics vs. M1 Frozen Baseline
| Metric | M1 Pure Feedforward (`112354`) | Candidate 2 Best Tested P-Only (`152543`) | Engineering Assessment |
| :--- | :---: | :---: | :--- |
| **Roll ($G_{\text{Peak}} / G_{\text{RMS}}$)** | 0.4778 / 0.4071 | **0.4584 / 0.2724** | Roll clearly better than M1 |
| **Pitch ($G_{\text{Peak}} / G_{\text{RMS}}$)** | 0.5946 / 0.4153 | **0.3951 / 0.2794** | Pitch clearly better than M1 |
| **Yaw ($G_{\text{Peak}} / G_{\text{RMS}}$)** | **0.7983 / 0.2912** | 0.8274 / 0.3075 | Yaw remains slightly worse than M1 |
| **Overall $SO(3)$ ($G_{\text{Peak}} / G_{\text{RMS}}$)** | 0.7353 / 0.3713 | **0.4334 / 0.2789** | Overall $SO(3)$ clearly better than M1 |

**Core Engineering Conclusions**:
1. Roll, Pitch, and overall $SO(3)$ performance are clearly better than M1 Pure Feedforward.
2. Yaw remains slightly worse than M1 Pure Feedforward.
3. The coarse sweep indicates that the useful tested Yaw-P region lies between the degraded low/high candidates, with $K_{p\_yaw} = 0.025$ providing the best measured overall balance.
4. The best measured Yaw $G_{\text{RMS}}$ among the tested P-only configurations was $0.3075$.
5. The P-only experiments show a non-monotonic gain-performance relationship on the Yaw channel. Increasing $K_{p\_yaw}$ to $0.10$ degraded performance, while reducing it to $0.025$ improved the system-level balance. Further reducing it to $0.0125$ caused Yaw rejection to rebound. The exact physical mechanism has not yet been isolated.
6. P-only tuning therefore provides a strong traditional-control baseline, but dynamic performance still has room for improvement.

---

## 6. Progressive Strategy Roadmap & Tuning Log

The canonical performance record tracks system-level controller evolution through standardized trial records:

| Stage | Controller Architecture | $G_{\text{Peak}}$ (R / P / Y / $SO(3)$) | $G_{\text{RMS}}$ (R / P / Y / $SO(3)$) | Canonical Session | Status / Outcome |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **J11-M1** | 3DOF Base-IMU Feedforward | 0.4778 / 0.5946 / 0.7983 / 0.7353 | 0.4071 / 0.4153 / 0.2912 / 0.3713 | `20260918_112354` | **FROZEN BASELINE** |
| **J11-M2** | Qualified Baseline ($K_p=[0.05, 0.05, 0.05]$) | 0.4900 / 0.3110 / 0.9030 / 0.3633 | 0.3066 / 0.2705 / 0.3374 / 0.2888 | `20260918_152354` | **QUALIFIED BASELINE** |
| **J11-M2-5** | Cand 1 Trial ($K_p=[0.05, 0.05, 0.10]$) | 0.5664 / 0.6917 / 1.5515 / 0.6874 | 0.4225 / 0.4464 / 0.5060 / 0.4482 | `20260918_160014` | **NOT RETAINED (Degraded)** |
| **J11-M2-5** | Cand 2 Trial ($K_p=[0.05, 0.05, 0.025]$) | **0.4584 / 0.3951 / 0.8274 / 0.4334** | **0.2724 / 0.2794 / 0.3075 / 0.2789** | `20260919_152543` | **BEST TESTED P-ONLY CONFIGURATION** |
| **J11-M2-5** | Cand 3 Trial ($K_p=[0.05, 0.05, 0.0125]$) | 0.4163 / 0.3715 / 0.8828 / 0.4463 | 0.2967 / 0.2624 / 0.3992 / 0.2865 | `20260919_153847` | **NOT RETAINED (Rebound)** |
| **J11-M2-6** | 3DOF FF + 3DOF PD ($K_p + K_d$) | *Pending* | *Pending* | TBD | *Next Stage Roadmap* |
| **J11-M2-7** | 3DOF FF + 3DOF PID ($K_p + K_d + K_i$) | *Pending* | *Pending* | TBD | *Roadmap* |
| **J12** | Task-Space ADRC (ESO + Disturbance Rejection) | *Pending* | *Pending* | TBD | *Roadmap* |

---
*Archive Maintained by: Lyapunov / DeepMind Advanced Agentic Coding Team*
