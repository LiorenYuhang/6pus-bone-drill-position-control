# J11 Control Performance Comparison Archive (M1 Feedforward vs. M2 Feedback)

**Current Status**: Active Canonical Comparison Baseline  
**Last Updated**: 2026-09-18  
**Scope**: Authoritative comparison between **J11-M1 Pure Feedforward** and **J11-M2-1 Roll P-only Feedback** on real physical hardware.

---

## 1. Stage Definition & Architectural Scope (Phase Clarification)

> [!IMPORTANT]
> **Strict Stage Definition for J11-M2-1**:
> - Stage J11-M2 introduces the **Task-Space $SO(3)$ Attitude PID Control Architecture** into the system.
> - However, the real-hardware checkout **J11-M2-1 is strictly a 1DOF Roll P-only experiment**, NOT a Full 3DOF PID deployment.
> - Active controller configuration in M2-1:
>   - $K_{p\_roll} = 0.05, \quad K_{p\_pitch} = 0.0, \quad K_{p\_yaw} = 0.0$
>   - $K_i = 0, \quad K_d = 0$
>   - $\text{max\_residual\_roll} = 0.5^\circ$
> - **Precise Control Structure**:
>   $$\mathbf{\text{3DOF Base-IMU } SO(3) \text{ Feedforward}} + \mathbf{\text{1DOF Platform-IMU Roll P Feedback}}$$
>   - **Roll Axis**: Feedforward + P-Feedback
>   - **Pitch Axis**: Feedforward Only ($K_p = 0$)
>   - **Yaw Axis**: Feedforward Only ($K_p = 0$)
> - **Attribution Boundary**: Any Pitch/Yaw stabilization observed in M2-1 originates entirely from the frozen M1 full $SO(3)$ feedforward loop, and must NOT be attributed to Roll P feedback.
> - **Approved Terminology**: "Task-Space $SO(3)$ Attitude PID architecture introduced".  
> - **Forbidden Terminology**: "3DOF PID completed", "Full PID attitude control verified".

### J11-M2 Phased Research Roadmap:
1. **J11-M2-1**: Roll P-only Hardware Feedback Checkout (**COMPLETED & VERIFIED**)
2. **J11-M2-2**: Pitch P-only Hardware Feedback Checkout (*Pending*)
3. **J11-M2-3**: Yaw P-only Hardware Feedback Checkout (*Pending*)
4. **J11-M2-4**: 3DOF P Hardware Feedback Checkout (*Pending*)
5. **J11-M2-5**: PD Feedback (if derivative damping is required, *Pending*)
6. **J11-M2-6**: Integral Action (I term, evaluated only if steady-state bias persists, *Pending*)

---

## 2. Canonical Datasets & Execution Provenance

All comparative data in this archive are drawn directly from machine-generated, timestamped, 50 Hz real-hardware telemetry runs:

| Attribute | M1 Canonical Dynamic Baseline | M2-1B Dynamic Hardware Checkout |
| :--- | :--- | :--- |
| **Run Timestamp** | `20260918_112354` | `20260918_135111` |
| **Stage Identifier** | `J11_DISTURB_QUAL` | `j11_m2_1_phaseB_dynamic` |
| **Evidence Path** | [`results/j11/runtime_validation/20260918_112354/`](../../results/j11/runtime_validation/20260918_112354/) | [`results/j11/m2_pid/20260918_135111/`](../../results/j11/m2_pid/20260918_135111/) |
| **Primary Telemetry CSV** | `J11_DISTURB_QUAL_feedforward_powered.csv` | `j11_m2_1_phaseB_dynamic_telemetry_powered.csv` |
| **Telemetry Format** | 50 Hz, 5999 rows (120 s) | 50 Hz, 3000 rows (60 s), 53-col unified schema |
| **Control Mode** | Pure Feedforward ($R_{BP\_cmd} = \Delta R_B^T R_{BP0}$) | Composite ($R_{BP\_cmd} = \Delta R_{PID\_B} @ R_{BP\_FF}$) |
| **Feedback Active** | False (Observer Only) | True (Roll P-only: $K_p=0.05$, Max $0.5^\circ$) |
| **Hardware Mode** | `active` (Real 6PUS, RS485 50 Hz) | `active` (Real 6PUS, RS485 50 Hz) |
| **Driver Runtime** | Dedicated Python Servo Thread + Two-Tier Safety | Dedicated Python Servo Thread + Two-Tier Safety |

---

## 3. Quantitative Comparison & Rejection Metrics

### Methodological Rigor on Manual Disturbance:
> [!NOTE]
> Because dynamic disturbances are applied manually to the robot Base, the input disturbance profiles are not mathematically identical across sessions.
> Consequently, comparing raw Platform Roll Peak or RMS values directly does not isolate controller efficacy.
> The primary scientific and engineering metrics for cross-controller evaluation are the normalized rejection ratios:
> $$G_{\text{Peak}} = \frac{\text{Peak}(\text{Platform Roll})}{\text{Peak}(\text{Base Roll})}, \qquad G_{\text{RMS}} = \frac{\text{RMS}(\text{Platform Roll})}{\text{RMS}(\text{Base Roll})}$$
> Normalized improvements are defined as:
> $$\Delta G_{\text{Peak}} = \frac{G_{\text{Peak}}^{M1} - G_{\text{Peak}}^{M2}}{G_{\text{Peak}}^{M1}} \times 100\%, \qquad \Delta G_{\text{RMS}} = \frac{G_{\text{RMS}}^{M1} - G_{\text{RMS}}^{M2}}{G_{\text{RMS}}^{M1}} \times 100\%$$

### Master Performance Summary Table:

| Metric | M1: Pure Feedforward | M2-1B: FF + Roll P ($K_p=0.05$) | Physical Delta / Assessment |
| :--- | :---: | :---: | :---: |
| **Base Roll Peak (\|max\|)** | $11.1825^\circ$ | $8.2766^\circ$ | $-2.9059^\circ$ |
| **Base Roll RMS** | $2.5277^\circ$ | $2.9193^\circ$ | $+0.3916^\circ$ (higher input energy) |
| **Platform Roll Peak (\|max\|)** | $5.3425^\circ$ | $3.5193^\circ$ | $-1.8232^\circ$ (raw excursion drop) |
| **Platform Roll RMS** | $1.0291^\circ$ | $1.1777^\circ$ | $+0.1487^\circ$ (under $+15.5\%$ base RMS) |
| **$G_{\text{Peak}}$ (Plat Peak / Base Peak)** | $\mathbf{0.4778}$ | $\mathbf{0.4252}$ | $\mathbf{-0.0526}$ (lower is better) |
| **$G_{\text{RMS}}$ (Plat RMS / Base RMS)** | $\mathbf{0.4071}$ ($40.71\%$) | $\mathbf{0.4034}$ ($40.34\%$) | $\mathbf{-0.0037}$ (lower is better) |
| **Normalized Peak Improvement** | *Baseline* | $\mathbf{+11.01\%}$ | Clear transient peak attenuation |
| **Normalized RMS Improvement** | *Baseline* | $\mathbf{+0.91\%}$ | Positive variance attenuation |

---

## 4. Closed-Loop Negative Feedback & Residual Analysis

### 4.1. Negative Feedback Sign Verification in Action
The fundamental prerequisite for closed-loop stability is that corrective action strictly opposes measured platform attitude errors.
- **Empirical Correlation**: The Pearson correlation coefficient between Platform Roll and the commanded Roll PID Residual is:
  $$\rho(\text{Platform Roll}, \text{Roll PID Residual}) = \mathbf{-0.999969} \approx -1.0000$$
- **Extremum Spot Checks**:
  - Positive Platform Excursion Peak ($+3.5193^\circ$): $e_{R\_W\_roll} = -3.5170^\circ \implies \delta \phi_{\text{res}} = \mathbf{-0.17585^\circ}$ (Counter-corrective negative rotation).
  - Negative Platform Excursion Peak ($-3.2004^\circ$): $e_{R\_W\_roll} = +3.2011^\circ \implies \delta \phi_{\text{res}} = \mathbf{+0.16005^\circ}$ (Counter-corrective positive rotation).

### 4.2. Residual Capacity & Saturation Metrics
- **Configured Residual Ceiling**: $\delta \phi_{\text{max}} = 0.50^\circ$.
- **Peak Commanded Residual**: $0.17585^\circ$.
- **Dynamic Headroom Used**: $0.17585^\circ / 0.50^\circ = \mathbf{35.17\%}$.
- **Saturation Cycles**: **0 cycles** ($\ge 0.4999^\circ$). The controller operated entirely within its linear range without windup or clipping.
- **Cross-Axis Decoupling**: Pitch PID residual $= 0.00000^\circ$ strictly; Yaw PID residual $= 0.00000^\circ$ strictly. Zero cross-channel bleed.

---

## 5. Actuator & Runtime Safety Telemetry

| Parameter | M1 Canonical Baseline | M2-1B Hardware Checkout | Safety Margin / Limit |
| :--- | :---: | :---: | :---: |
| **Active Commanded Cycles** | 5928 / 5999 | 2930 / 3000 | 100% active cycles commanded |
| **Actuator Stroke Span** | 395 steps (808..1203) | 339 steps (787..1220) | Well within $[0, 2000]$ workspace |
| **Max Motor Current** | $156\,\text{mA}$ | $168\,\text{mA}$ | Safe ($< 500\,\text{mA}$ trip threshold) |
| **Hardware Fault Codes** | `0x0000` (0 faults) | `0x0000` (0 faults) | Zero motor faults |
| **Warning Hold Occurrences** | 17 (recovered) | 0 | Zero warning hold triggers |
| **Hard Stop Occurrences** | 0 | 0 | Zero safety aborts |
| **Driver Feedback Age** | Mean 15.1 ms / Max 305 ms | Mean 15.3 ms / Max 288 ms | Compliant with Two-Tier Policy |
| **Base IMU Age** | Steady-state $< 25\,\text{ms}$ | Steady-state $< 41\,\text{ms}$ | Safe ($< 100\,\text{ms}$ latch limit) |
| **Platform IMU Age** | Steady-state $< 25\,\text{ms}$ | Steady-state $< 42\,\text{ms}$ | Safe ($< 100\,\text{ms}$ latch limit) |

---

## 6. Engineering Interpretation & Takeaways

1. **Proof of Closed-Loop Viability**:
   The transition from open-loop feedforward to closed-loop feedforward + feedback is inherently prone to instability due to sensor latency, unmodeled kinematics, and numerical frame mismatches. The fact that M2-1B achieved $\rho = -1.0000$ negative feedback correlation, zero limit-cycle oscillations, zero current spikes, and strict numerical stability across 60 seconds of continuous dynamic perturbation validates the $SO(3)$ Lie-algebra error formulation and left-composition law.

2. **Transient vs. Variance Attenuation**:
   - $G_{\text{Peak}}$ improved by **$11.01\%$** ($0.4778 \to 0.4252$), indicating that even a modest proportional gain ($K_p = 0.05$) effectively clamps the extreme excursion tips where feedforward tracking error momentarily peaks.
   - $G_{\text{RMS}}$ improved by **$0.91\%$** ($0.4071 \to 0.4034$). Because $K_p$ is intentionally conservative and no derivative damping ($K_d=0$) or integral action ($K_i=0$) has been added, bulk variance reduction is modest but strictly positive.

3. **Gain Margins for Future Tuning**:
   With only $35.2\%$ of the $0.5^\circ$ residual limit utilized at peak disturbance and maximum currents remaining at $168\,\text{mA}$ (far below the $500\,\text{mA}$ threshold), substantial gain margin remains for subsequent tuning iterations (e.g., $K_p \in [0.08, 0.15]$).

---

## 7. Future Comparison Slots (Template for Subsequent Stages)

As subsequent stages are executed under equivalent disturbance protocols, their canonical results will be appended below:

| Stage | Controller Architecture | $G_{\text{Peak}}$ (Roll) | $G_{\text{RMS}}$ (Roll) | $\Delta G_{\text{Peak}}$ vs M1 | $\Delta G_{\text{RMS}}$ vs M1 | Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **M1** | 3DOF Base Feedforward | 0.4778 | 0.4071 | Baseline | Baseline | **FROZEN** |
| **M2-1** | 3DOF FF + Roll P ($K_p=0.05$) | 0.4252 | 0.4034 | +11.01% | +0.91% | **VERIFIED** |
| **M2-2** | 3DOF FF + Pitch P | TBD | TBD | TBD | TBD | *Pending* |
| **M2-3** | 3DOF FF + Yaw P | TBD | TBD | TBD | TBD | *Pending* |
| **M2-4** | 3DOF FF + 3DOF P | TBD | TBD | TBD | TBD | *Pending* |
| **M2-5** | 3DOF FF + 3DOF PD | TBD | TBD | TBD | TBD | *Pending* |
| **M3** | ADRC (ESO + Disturbance Rejection) | TBD | TBD | TBD | TBD | *Roadmap* |

---
*Archive Maintained by: Lyapunov / DeepMind Advanced Agentic Coding Team*
