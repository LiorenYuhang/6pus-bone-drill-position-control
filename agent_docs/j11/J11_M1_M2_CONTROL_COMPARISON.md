# J11 Canonical System-Level Controller Performance Comparison Archive

**Current Status**: Active Canonical Comparison Baseline  
**Last Updated**: 2026-09-18  
**Scope**: Authoritative performance comparison of multi-axis control architectures on the 6PUS parallel surgical robot:
- **J11-M1**: 3DOF Base-IMU $SO(3)$ Pure Feedforward (*Frozen Baseline*)
- **J11-M2**: 3DOF Base-IMU $SO(3)$ Feedforward + 3DOF Platform-IMU $SO(3)$ P-only Feedback (*Qualified Baseline: $K_p = [0.05, 0.05, 0.05]$*)
- **J11-M2-5**: 3DOF P Gain Tuning (*Active Stage; Candidate 1 tested and not retained*)
- **Subsequent Stages**: 3DOF Tuned P $\to$ 3DOF PD (M2-6) $\to$ 3DOF PID (M2-7) $\to$ Active Disturbance Rejection Control (J12 ADRC).

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
│ Progression Roadmap:                                                            │
│   • J11-M2-5: 3DOF P Gain Tuning (Active stage, single-variable tuning trials)  │
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

| Attribute | J11-M1 Canonical Baseline | J11-M2 Qualified Baseline | J11-M2-5 Candidate 1 Trial |
| :--- | :--- | :--- | :--- |
| **Run Timestamp** | `20260918_112354` | `20260918_152354` | `20260918_160014` |
| **Stage Identifier** | `J11_DISTURB_QUAL` | `j11_m2_4_phaseB_dynamic` | `j11_m2_5_phaseB_dynamic` |
| **Evidence Directory** | [`results/j11/runtime_validation/20260918_112354/`](../../results/j11/runtime_validation/20260918_112354/) | [`results/j11/m2_pid/20260918_152354/`](../../results/j11/m2_pid/20260918_152354/) | [`results/j11/m2_pid/20260918_160014/`](../../results/j11/m2_pid/20260918_160014/) |
| **Primary Telemetry CSV** | `J11_DISTURB_QUAL_feedforward_powered.csv` | `j11_m2_4_phaseB_dynamic_telemetry_powered.csv` | `j11_m2_5_phaseB_dynamic_telemetry_powered.csv` |
| **Duration / Rate** | 120.0 s (5999 rows @ 50 Hz) | 60.0 s (2999 rows @ 50 Hz, 2929 writes) | 60.0 s (2997 rows @ 50 Hz, 2943 writes) |
| **Control Structure** | 3DOF Base Feedforward Only | 3DOF Feedforward + 3DOF P Feedback | 3DOF Feedforward + 3DOF P Feedback |
| **Active Gains** | None ($K_p=0, K_i=0, K_d=0$) | $K_p = [0.05, 0.05, 0.05]$ | $K_p = [0.05, 0.05, 0.10]$ |
| **Residual Limits** | N/A | $\pm 0.5^\circ$ per axis | $\pm 0.5^\circ$ per axis |
| **Classification** | **FROZEN BASELINE** | **QUALIFIED BASELINE (CURRENT BEST)** | **INTERMEDIATE TRIAL (NOT RETAINED)** |

> [!WARNING]
> **Formal Exclusion of Session `20260918_150445`**:
> Session `results/j11/m2_pid/20260918_150445/` is formally classified as **NON-QUALIFIED / INVALID** and excluded from canonical performance evaluation because the operator did not intentionally apply the prescribed multi-axis manual Base disturbance during that run. Its raw telemetry is preserved intact strictly as an observational record.

---

## 3. Master System-Level Performance Comparison Table

Because manual base disturbances are applied naturally by hand, raw platform excursion depends on the input disturbance energy. Scientific cross-controller comparison therefore relies on normalized rejection ratios:
$$G_{\text{Peak}} = \frac{\text{Peak}(\text{Platform Error})}{\text{Peak}(\text{Base Excursion})}, \qquad G_{\text{RMS}} = \frac{\text{RMS}(\text{Platform Error})}{\text{RMS}(\text{Base Excursion})}$$
$$G_{SO(3)\_\text{Peak}} = \frac{\text{Peak}(\text{Platform } SO(3) \text{ Error})}{\text{Peak}(\text{Base } SO(3) \text{ Excursion})}, \qquad G_{SO(3)\_\text{RMS}} = \frac{\text{RMS}(\text{Platform } SO(3) \text{ Error})}{\text{RMS}(\text{Base } SO(3) \text{ Excursion})}$$

### Canonical Master Comparison:

| Axis / Metric | Parameter | M1: Pure Feedforward (`20260918_112354`) | M2: Qualified 3DOF P Baseline (`20260918_152354`) | M2 Candidate 1 Trial ($K_{p\_yaw}=0.10$) (`20260918_160014`) | Evaluation vs. Qualified Baseline |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **Roll** | Base Peak / RMS | $11.1825^\circ$ / $2.5426^\circ$ | $9.5619^\circ$ / $3.6490^\circ$ | $11.6446^\circ$ / $4.0537^\circ$ | — |
| | Plat Peak / RMS | $5.3425^\circ$ / $1.0351^\circ$ | $4.6850^\circ$ / $1.1188^\circ$ | $6.5956^\circ$ / $1.7125^\circ$ | Excursion increased |
| | **$G_{\text{Peak}}$** | **0.4778** | **0.4900** | **0.5664** | **-15.59%** (Degraded) |
| | **$G_{\text{RMS}}$** | **0.4071** | **0.3066** | **0.4225** | **-37.80%** (Degraded) |
| **Pitch** | Base Peak / RMS | $10.8622^\circ$ / $2.7747^\circ$ | $12.0675^\circ$ / $4.5458^\circ$ | $12.4453^\circ$ / $4.5927^\circ$ | — |
| | Plat Peak / RMS | $6.4585^\circ$ / $1.1522^\circ$ | $3.7535^\circ$ / $1.2295^\circ$ | $8.6080^\circ$ / $2.0500^\circ$ | Platform excursion surge |
| | **$G_{\text{Peak}}$** | **0.5946** | **0.3110** | **0.6917** | **-122.41%** (Severe degradation) |
| | **$G_{\text{RMS}}$** | **0.4153** | **0.2705** | **0.4464** | **-65.03%** (Severe degradation) |
| **Yaw** | Base Peak / RMS | $10.9373^\circ$ / $2.8939^\circ$ | $3.4753^\circ$ / $1.5259^\circ$ | $4.3949^\circ$ / $2.4879^\circ$ | Natural multi-axis yaw disturbance |
| | Plat Peak / RMS | $8.7312^\circ$ / $0.8428^\circ$ | $3.1383^\circ$ / $0.5148^\circ$ | $6.8188^\circ$ / $1.2589^\circ$ | Platform yaw amplification |
| | **$G_{\text{Peak}}$** | **0.7983** | **0.9030** | **1.5515** | **-71.82%** (Severe degradation, $>1.0$) |
| | **$G_{\text{RMS}}$** | **0.2912** | **0.3374** | **0.5060** | **-49.97%** (Severe degradation) |
| **Overall $SO(3)$** | Base Peak / RMS | $13.5370^\circ$ / $4.7470^\circ$ | $12.9456^\circ$ / $6.0256^\circ$ | $13.8446^\circ$ / $6.5948^\circ$ | Multi-axis 3D disturbance input |
| | Plat Peak / RMS | $9.9533^\circ$ / $1.7626^\circ$ | $4.7035^\circ$ / $1.7405^\circ$ | $9.5163^\circ$ / $2.9561^\circ$ | 3D attitude error doubled |
| | **$G_{SO(3)\_\text{Peak}}$** | **0.7353** | **0.3633** | **0.6874** | **-89.21%** (Severe degradation) |
| | **$G_{SO(3)\_\text{RMS}}$** | **0.3713** | **0.2888** | **0.4482** | **-55.20%** (Severe degradation) |

### Engineering Assessment on Candidate 1 Trial:
1. **Decision**: **$K_{p\_yaw} = 0.10$ IS NOT RETAINED**.
2. **Attribution & Cross-Axis Observation**: Increasing $K_{p\_yaw}$ from $0.05$ to $0.10$ produced substantial system-level cross-axis performance degradation in this experiment. The exact physical mechanism has not yet been isolated.
3. **Observed Empirical Phenomena**:
   - Roll rejection degraded ($-15.59\%$ Peak, $-37.80\%$ RMS).
   - Pitch rejection degraded severely ($-122.41\%$ Peak, $-65.03\%$ RMS).
   - Yaw rejection degraded severely ($-71.82\%$ Peak, $-49.97\%$ RMS), with $G_{\text{Peak}} = 1.5515$ indicating platform excursion exceeding base excitation.
   - Overall $SO(3)$ attitude error doubled ($-89.21\%$ Peak, $-55.20\%$ RMS).
   - Yaw attitude residual correction reached saturation for $26$ cycles ($0.88\%$).
4. **Current Best Baseline Retained**: The qualified baseline **$K_p = [0.05, 0.05, 0.05]$** remains the active authoritative reference. Stage M2-5 gain tuning remains active.

---

## 4. Closed-Loop Negative Feedback & Residual Telemetry

| Parameter | M2 Qualified Baseline (`20260918_152354`) | M2 Candidate 1 Trial (`20260918_160014`) | Note / Limit |
| :--- | :---: | :---: | :---: |
| **$K_p$ Configuration** | $[0.05, 0.05, 0.05]$ | $[0.05, 0.05, 0.10]$ | Single-variable yaw test |
| **Roll Residual Peak / RMS** | $0.2342^\circ$ / $0.0553^\circ$ | $0.3330^\circ$ / $0.0859^\circ$ | Within $\pm 0.50^\circ$ limit |
| **Pitch Residual Peak / RMS** | $0.1878^\circ$ / $0.0607^\circ$ | $0.4274^\circ$ / $0.1022^\circ$ | Within $\pm 0.50^\circ$ limit |
| **Yaw Residual Peak / RMS** | $0.1575^\circ$ / $0.0255^\circ$ | $\mathbf{0.5000^\circ}$ / $0.1218^\circ$ | Reached $\pm 0.50^\circ$ limit |
| **Saturation Cycles (R / P / Y)** | **0 / 0 / 0** ($0.0\%$) | **0 / 0 / 26** ($0.88\%$ on Yaw) | 26 cycles clamped on Yaw |
| **Pearson Correlation $\rho$ (R / P / Y)** | -0.999977 / -0.999978 / -0.999618 | -0.999635 / -0.999905 / -0.995515 | Negative feedback preserved |
| **Max Tracking Error** | 62 steps | 90 steps | Continuous servo follow-up |
| **Max Joint Current** | $171\,\text{mA}$ | $282\,\text{mA}$ | Below $500\,\text{mA}$ safety trip |
| **Hardware Faults** | `0x0000` (0 faults) | `0x0000` (0 faults) | Zero motor faults |
| **Safety Aborts (WH / HS)** | 0 / 0 | 0 / 0 | Zero safety aborts |
| **Base IMU Age** | Mean $2.14\,\text{ms}$ / Max $76.2\,\text{ms}$ | Mean $2.20\,\text{ms}$ / Max $78.0\,\text{ms}$ | Measured runtime latency |
| **Platform IMU Age** | Mean $2.48\,\text{ms}$ / Max $81.5\,\text{ms}$ | Mean $2.59\,\text{ms}$ / Max $82.5\,\text{ms}$ | Measured runtime latency |
| **Driver Feedback Age** | Mean $15.8\,\text{ms}$ / Max $245.1\,\text{ms}$ | Mean $17.3\,\text{ms}$ / Max $231.1\,\text{ms}$ | Measured runtime latency |

---

## 5. Historical Single-Axis Channel Qualification

Before 3DOF integration, Roll/Pitch/Yaw P-only tests were used strictly for feedback-channel qualification.

All three channels passed hardware verification for:
- feedback sign
- $SO(3)$ error/residual path
- execution chain
- safety behavior

Detailed single-axis evidence remains archived under [`results/j11/m2_pid/`](../../results/j11/m2_pid/) and is not used as canonical system-level performance evidence.

---

## 6. Progressive Strategy Roadmap & Tuning Log

The canonical performance record tracks system-level controller evolution through standardized trial records:

| Stage | Controller Architecture | $G_{\text{Peak}}$ (R / P / Y / $SO(3)$) | $G_{\text{RMS}}$ (R / P / Y / $SO(3)$) | Canonical Session | Status / Outcome |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **J11-M1** | 3DOF Base-IMU Feedforward | 0.4778 / 0.5946 / 0.7983 / 0.7353 | 0.4071 / 0.4153 / 0.2912 / 0.3713 | `20260918_112354` | **FROZEN BASELINE** |
| **J11-M2** | Qualified 3DOF P Baseline ($K_p=[0.05, 0.05, 0.05]$) | **0.4900 / 0.3110 / 0.9030 / 0.3633** | **0.3066 / 0.2705 / 0.3374 / 0.2888** | `20260918_152354` | **QUALIFIED BASELINE (CURRENT BEST)** |
| **J11-M2-5** | Candidate 1 Trial ($K_p=[0.05, 0.05, 0.10]$) | 0.5664 / 0.6917 / 1.5515 / 0.6874 | 0.4225 / 0.4464 / 0.5060 / 0.4482 | `20260918_160014` | **NOT RETAINED (Degraded)** |
| **J11-M2-5** | Subsequent Tuning Candidates | *Pending Selection* | *Pending Selection* | TBD | *Active Tuning Stage* |
| **J11-M2-6** | 3DOF FF + 3DOF PD ($K_p + K_d$) | *Pending* | *Pending* | TBD | *Roadmap* |
| **J11-M2-7** | 3DOF FF + 3DOF PID ($K_p + K_d + K_i$) | *Pending* | *Pending* | TBD | *Roadmap* |
| **J12** | Task-Space ADRC (ESO + Disturbance Rejection) | *Pending* | *Pending* | TBD | *Roadmap* |

---
*Archive Maintained by: Lyapunov / DeepMind Advanced Agentic Coding Team*
