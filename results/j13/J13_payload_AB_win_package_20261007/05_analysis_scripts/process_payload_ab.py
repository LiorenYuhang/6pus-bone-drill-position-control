import csv
import json
import math
import os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# File paths
DIR_A = Path("/home/lyapunov/bone_drill/results/j13/j13_2a2_shadow/20261007_143007/j13_3axis_payload_classical_10s")
DIR_B = Path("/home/lyapunov/bone_drill/results/j13/j13_2c1_3axis_active/20261007_143036/j13_3axis_payload_ladrc_10s")

OUT_DIR = Path("/home/lyapunov/bone_drill/results/j13/j13_payload_ab_analysis_20261007")
OUT_DIR.mkdir(parents=True, exist_ok=True)

def load_data(dir_path):
    telem_file = list(dir_path.glob("*telemetry_powered.csv"))[0]
    cycle_file = dir_path / "cycle_timing.csv"
    trans_file = dir_path / "transport_phase_timing.csv"
    summary_file = list(dir_path.glob("*summary_powered.json"))[0]
    
    with open(telem_file) as f:
        rows = list(csv.DictReader(f))
        
    with open(summary_file) as f:
        summary = json.load(f)
        
    cycle_rows = []
    if cycle_file.exists():
        with open(cycle_file) as f:
            cycle_rows = list(csv.DictReader(f))
            
    trans_rows = []
    if trans_file.exists():
        with open(trans_file) as f:
            trans_rows = list(csv.DictReader(f))
            
    return rows, summary, cycle_rows, trans_rows

rows_a, sum_a, cyc_a, trans_a = load_data(DIR_A)
rows_b, sum_b, cyc_b, trans_b = load_data(DIR_B)

def parse_field(r, key, idx=None):
    val = r[key]
    if val.startswith('['):
        arr = json.loads(val)
        return arr[idx] if idx is not None else arr
    return float(val)

def get_stats(arr):
    arr = np.array(arr, dtype=np.float64)
    rms = np.sqrt(np.mean(arr**2))
    mae = np.mean(np.abs(arr))
    p95 = np.percentile(np.abs(arr), 95)
    peak = np.max(np.abs(arr))
    mean = np.mean(arr)
    std = np.std(arr)
    return {"rms": rms, "mae": mae, "p95": p95, "peak": peak, "mean": mean, "std": std}

def prepare_active_data(rows):
    act_rows = [r for r in rows if r['state'] == 'ACTIVE']
    t0_act = int(act_rows[0]['timestamp_ns'])
    
    t_sec = np.array([(int(r['timestamp_ns']) - t0_act) * 1e-9 for r in act_rows])
    
    b_roll = np.array([float(r['base_rel_roll_deg']) for r in act_rows])
    b_pitch = np.array([float(r['base_rel_pitch_deg']) for r in act_rows])
    b_yaw = np.array([float(r['base_rel_yaw_deg']) for r in act_rows])
    
    e_r = np.array([parse_field(r, 'e_R_W_deg', 0) for r in act_rows])
    e_p = np.array([parse_field(r, 'e_R_W_deg', 1) for r in act_rows])
    e_y = np.array([parse_field(r, 'e_R_W_deg', 2) for r in act_rows])
    
    p_roll = np.array([float(r['platform_rel_roll_deg']) for r in act_rows])
    p_pitch = np.array([float(r['platform_rel_pitch_deg']) for r in act_rows])
    p_yaw = np.array([float(r['platform_rel_yaw_deg']) for r in act_rows])
    
    u_comb = np.array([parse_field(r, 'u_combined_actual_deg') for r in act_rows])
    u_class = np.array([parse_field(r, 'classical_u_actual_deg') for r in act_rows])
    u_ladrc_raw = np.array([parse_field(r, 'ladrc_u_raw_deg') for r in act_rows])
    u_ladrc_shad = np.array([parse_field(r, 'ladrc_u_shadow_deg') for r in act_rows])
    
    z1 = np.array([parse_field(r, 'ladrc_z1_deg') for r in act_rows])
    z2 = np.array([parse_field(r, 'ladrc_z2_deg_s') for r in act_rows])
    
    cmd_steps = np.array([parse_field(r, 'commanded_steps') for r in act_rows])
    fb_steps = np.array([parse_field(r, 'feedback_steps') for r in act_rows])
    step_err = np.array([parse_field(r, 'command_feedback_errors') for r in act_rows])
    currents = np.array([parse_field(r, 'joint_currents_raw_mA') for r in act_rows])
    fb_age = np.array([float(r['driver_feedback_age_ms']) for r in act_rows])
    
    return {
        "rows": act_rows,
        "t_sec": t_sec,
        "b_roll": b_roll, "b_pitch": b_pitch, "b_yaw": b_yaw,
        "e_r": e_r, "e_p": e_p, "e_y": e_y,
        "p_roll": p_roll, "p_pitch": p_pitch, "p_yaw": p_yaw,
        "u_comb": u_comb, "u_class": u_class,
        "u_ladrc_raw": u_ladrc_raw, "u_ladrc_shad": u_ladrc_shad,
        "z1": z1, "z2": z2,
        "cmd_steps": cmd_steps, "fb_steps": fb_steps,
        "step_err": step_err, "currents": currents,
        "fb_age": fb_age
    }

dA = prepare_active_data(rows_a)
dB = prepare_active_data(rows_b)

# Configure plot styles
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Liberation Sans', 'Arial']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.dpi'] = 150

# 1. Fig 01: Base Roll Comparison
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(dA['t_sec'], dA['b_roll'], label='Classical Run Base Roll', color='#1f77b4', lw=1.5)
ax.plot(dB['t_sec'], dB['b_roll'], label='3-Axis LADRC Run Base Roll', color='#ff7f0e', lw=1.5, ls='--')
ax.set_title("Fig 01: Base Roll Excitation Comparison (Eccentric Payload)", fontsize=12, fontweight='bold')
ax.set_xlabel("Time from ACTIVE Start (s)", fontsize=10)
ax.set_ylabel("Base Roll (deg)", fontsize=10)
ax.grid(True, ls=':', alpha=0.6)
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(OUT_DIR / "fig01_base_roll_comparison.png")
plt.close(fig)

# 2. Fig 02: Base Pitch Comparison
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(dA['t_sec'], dA['b_pitch'], label='Classical Run Base Pitch', color='#1f77b4', lw=1.5)
ax.plot(dB['t_sec'], dB['b_pitch'], label='3-Axis LADRC Run Base Pitch', color='#ff7f0e', lw=1.5, ls='--')
ax.set_title("Fig 02: Base Pitch Excitation Comparison (Eccentric Payload)", fontsize=12, fontweight='bold')
ax.set_xlabel("Time from ACTIVE Start (s)", fontsize=10)
ax.set_ylabel("Base Pitch (deg)", fontsize=10)
ax.grid(True, ls=':', alpha=0.6)
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(OUT_DIR / "fig02_base_pitch_comparison.png")
plt.close(fig)

# 3. Fig 03: Base Yaw Comparison
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(dA['t_sec'], dA['b_yaw'], label='Classical Run Base Yaw', color='#1f77b4', lw=1.5)
ax.plot(dB['t_sec'], dB['b_yaw'], label='3-Axis LADRC Run Base Yaw', color='#ff7f0e', lw=1.5, ls='--')
ax.set_title("Fig 03: Base Yaw Excitation Comparison (Eccentric Payload)", fontsize=12, fontweight='bold')
ax.set_xlabel("Time from ACTIVE Start (s)", fontsize=10)
ax.set_ylabel("Base Yaw (deg)", fontsize=10)
ax.grid(True, ls=':', alpha=0.6)
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(OUT_DIR / "fig03_base_yaw_comparison.png")
plt.close(fig)

# 4. Fig 04: Roll Error Comparison
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(dA['t_sec'], dA['e_r'], label='Classical (SO(3) FF + Roll P)', color='#1f77b4', lw=1.5)
ax.plot(dB['t_sec'], dB['e_r'], label='3-Axis LADRC (SO(3) FF + LADRC)', color='#d62728', lw=1.5)
ax.axhline(0, color='gray', lw=0.8, ls=':')
ax.set_title("Fig 04: Platform Roll Attitude Error ($e_R$) Comparison", fontsize=12, fontweight='bold')
ax.set_xlabel("Time from ACTIVE Start (s)", fontsize=10)
ax.set_ylabel("Roll Error (deg)", fontsize=10)
ax.grid(True, ls=':', alpha=0.6)
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(OUT_DIR / "fig04_roll_error_comparison.png")
plt.close(fig)

# 5. Fig 05: Pitch Error Comparison
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(dA['t_sec'], dA['e_p'], label='Classical (SO(3) FF + Pitch PD)', color='#1f77b4', lw=1.5)
ax.plot(dB['t_sec'], dB['e_p'], label='3-Axis LADRC (SO(3) FF + LADRC)', color='#d62728', lw=1.5)
ax.axhline(0, color='gray', lw=0.8, ls=':')
ax.set_title("Fig 05: Platform Pitch Attitude Error ($e_P$) Comparison", fontsize=12, fontweight='bold')
ax.set_xlabel("Time from ACTIVE Start (s)", fontsize=10)
ax.set_ylabel("Pitch Error (deg)", fontsize=10)
ax.grid(True, ls=':', alpha=0.6)
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(OUT_DIR / "fig05_pitch_error_comparison.png")
plt.close(fig)

# 6. Fig 06: Yaw Error Comparison
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(dA['t_sec'], dA['e_y'], label='Classical (SO(3) FF + Yaw PD)', color='#1f77b4', lw=1.5)
ax.plot(dB['t_sec'], dB['e_y'], label='3-Axis LADRC (SO(3) FF + LADRC)', color='#d62728', lw=1.5)
ax.axhline(0, color='gray', lw=0.8, ls=':')
ax.set_title("Fig 06: Platform Yaw Attitude Error ($e_Y$) Comparison", fontsize=12, fontweight='bold')
ax.set_xlabel("Time from ACTIVE Start (s)", fontsize=10)
ax.set_ylabel("Yaw Error (deg)", fontsize=10)
ax.grid(True, ls=':', alpha=0.6)
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(OUT_DIR / "fig06_yaw_error_comparison.png")
plt.close(fig)

# 7. Fig 07: Classical Control Output (Residual Command) vs Time
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(dA['t_sec'], dA['u_comb'][:, 0], label='Classical Roll Residual (P)', color='#1f77b4', lw=1.4)
ax.plot(dA['t_sec'], dA['u_comb'][:, 1], label='Classical Pitch Residual (PD)', color='#2ca02c', lw=1.4)
ax.plot(dA['t_sec'], dA['u_comb'][:, 2], label='Classical Yaw Residual (PD)', color='#9467bd', lw=1.4)
ax.axhline(+0.5, color='red', ls='--', lw=0.9, label='Clamp limit (±0.5°)')
ax.axhline(-0.5, color='red', ls='--', lw=0.9)
ax.set_title("Fig 07: Classical Controller Authoritative Output vs Time", fontsize=12, fontweight='bold')
ax.set_xlabel("Time from ACTIVE Start (s)", fontsize=10)
ax.set_ylabel("Residual Command (deg)", fontsize=10)
ax.set_ylim(-0.6, 0.6)
ax.grid(True, ls=':', alpha=0.6)
ax.legend(loc='upper right', ncol=2)
fig.tight_layout()
fig.savefig(OUT_DIR / "fig07_classical_control_output.png")
plt.close(fig)

# 8. Fig 08: LADRC Control Output (u_actual) vs Time
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(dB['t_sec'], dB['u_comb'][:, 0], label='LADRC Roll $u_{act}$', color='#1f77b4', lw=1.4)
ax.plot(dB['t_sec'], dB['u_comb'][:, 1], label='LADRC Pitch $u_{act}$', color='#2ca02c', lw=1.4)
ax.plot(dB['t_sec'], dB['u_comb'][:, 2], label='LADRC Yaw $u_{act}$', color='#9467bd', lw=1.4)
ax.axhline(+0.5, color='red', ls='--', lw=0.9, label='Clamp limit (±0.5°)')
ax.axhline(-0.5, color='red', ls='--', lw=0.9)
ax.set_title("Fig 08: 3-Axis LADRC Authoritative Output vs Time", fontsize=12, fontweight='bold')
ax.set_xlabel("Time from ACTIVE Start (s)", fontsize=10)
ax.set_ylabel("LADRC $u_{act}$ Command (deg)", fontsize=10)
ax.set_ylim(-0.6, 0.6)
ax.grid(True, ls=':', alpha=0.6)
ax.legend(loc='upper right', ncol=2)
fig.tight_layout()
fig.savefig(OUT_DIR / "fig08_ladrc_control_output.png")
plt.close(fig)

# 9. Fig 09: LADRC z2 States vs Time
fig, ax = plt.subplots(figsize=(8, 4.5))
ax.plot(dB['t_sec'], dB['z2'][:, 0], label='Roll $z_2$ (Total Disturbance)', color='#1f77b4', lw=1.5)
ax.plot(dB['t_sec'], dB['z2'][:, 1], label='Pitch $z_2$ (Total Disturbance)', color='#2ca02c', lw=1.5)
ax.plot(dB['t_sec'], dB['z2'][:, 2], label='Yaw $z_2$ (Total Disturbance)', color='#9467bd', lw=1.5)
ax.axhline(0, color='gray', lw=0.8, ls=':')
ax.set_title("Fig 09: 3-Axis LADRC ESO $z_2$ Disturbance Estimates vs Time", fontsize=12, fontweight='bold')
ax.set_xlabel("Time from ACTIVE Start (s)", fontsize=10)
ax.set_ylabel("ESO $z_2$ (deg/s)", fontsize=10)
ax.grid(True, ls=':', alpha=0.6)
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(OUT_DIR / "fig09_ladrc_z2_states.png")
plt.close(fig)

# 10. Fig 10: Initial Static Bias Zoom (First 1.5s)
fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
mask_a = dA['t_sec'] <= 1.5
mask_b = dB['t_sec'] <= 1.5

axes[0].plot(dA['t_sec'][mask_a], dA['e_r'][mask_a], label='Classical Roll Error', color='#1f77b4', lw=1.5)
axes[0].plot(dB['t_sec'][mask_b], dB['e_r'][mask_b], label='LADRC Roll Error', color='#d62728', lw=1.5)
axes[0].axhline(0, color='gray', ls=':', lw=0.8)
axes[0].set_ylabel("Roll Err (°)")
axes[0].legend(loc='upper right')
axes[0].grid(True, ls=':', alpha=0.6)
axes[0].set_title("Fig 10: Initial Static Bias Zoom (0 - 1.5s Static Sag under Payload)", fontsize=11, fontweight='bold')

axes[1].plot(dA['t_sec'][mask_a], dA['e_p'][mask_a], label='Classical Pitch Error', color='#1f77b4', lw=1.5)
axes[1].plot(dB['t_sec'][mask_b], dB['e_p'][mask_b], label='LADRC Pitch Error', color='#d62728', lw=1.5)
axes[1].axhline(0, color='gray', ls=':', lw=0.8)
axes[1].set_ylabel("Pitch Err (°)")
axes[1].legend(loc='upper right')
axes[1].grid(True, ls=':', alpha=0.6)

axes[2].plot(dA['t_sec'][mask_a], dA['e_y'][mask_a], label='Classical Yaw Error', color='#1f77b4', lw=1.5)
axes[2].plot(dB['t_sec'][mask_b], dB['e_y'][mask_b], label='LADRC Yaw Error', color='#d62728', lw=1.5)
axes[2].axhline(0, color='gray', ls=':', lw=0.8)
axes[2].set_ylabel("Yaw Err (°)")
axes[2].set_xlabel("Time from ACTIVE Start (s)")
axes[2].legend(loc='upper right')
axes[2].grid(True, ls=':', alpha=0.6)

fig.tight_layout()
fig.savefig(OUT_DIR / "fig10_initial_static_bias_zoom.png")
plt.close(fig)

print("Generated all 10 figures successfully in:", OUT_DIR)

# Compute Table Statistics
print("\n=== Table 3: Base Excitation Comparability ===")
for ax_name, k in [("Base Roll", "b_roll"), ("Base Pitch", "b_pitch"), ("Base Yaw", "b_yaw")]:
    sA = get_stats(dA[k])
    sB = get_stats(dB[k])
    print(f"{ax_name:10s} | A: RMS={sA['rms']:5.2f}°, P95={sA['p95']:5.2f}°, Peak={sA['peak']:5.2f}° | B: RMS={sB['rms']:5.2f}°, P95={sB['p95']:5.2f}°, Peak={sB['peak']:5.2f}°")

print("\n=== Table 4: Platform Attitude Error Comparison ===")
for ax_name, k in [("Roll Error", "e_r"), ("Pitch Error", "e_p"), ("Yaw Error", "e_y")]:
    sA = get_stats(dA[k])
    sB = get_stats(dB[k])
    diff_rms = (sB['rms'] - sA['rms']) / sA['rms'] * 100.0
    print(f"{ax_name:10s} | Classical: RMS={sA['rms']:5.2f}°, MAE={sA['mae']:5.2f}°, P95={sA['p95']:5.2f}°, Peak={sA['peak']:5.2f}° | LADRC: RMS={sB['rms']:5.2f}°, MAE={sB['mae']:5.2f}°, P95={sB['p95']:5.2f}°, Peak={sB['peak']:5.2f}° | ΔRMS: {diff_rms:+5.1f}%")

print("\n=== Table 5: Control Effort Comparison ===")
for ax_idx, ax_name in [(0, "Roll"), (1, "Pitch"), (2, "Yaw")]:
    uA = dA['u_comb'][:, ax_idx]
    uB = dB['u_comb'][:, ax_idx]
    sA = get_stats(uA)
    sB = get_stats(uB)
    satA = 100.0 * np.sum(np.abs(uA) >= 0.4999) / len(uA)
    satB = 100.0 * np.sum(np.abs(uB) >= 0.4999) / len(uB)
    print(f"{ax_name:6s} | Classical: RMS={sA['rms']:5.3f}°, Peak={sA['peak']:5.3f}°, Sat={satA:4.1f}% | LADRC: RMS={sB['rms']:5.3f}°, Peak={sB['peak']:5.3f}°, Sat={satB:4.1f}%")

print("\n=== Table 5b: LADRC Internal States (z1, z2) ===")
for ax_idx, ax_name in [(0, "Roll"), (1, "Pitch"), (2, "Yaw")]:
    z1_s = get_stats(dB['z1'][:, ax_idx])
    z2_s = get_stats(dB['z2'][:, ax_idx])
    print(f"{ax_name:6s} | z1: RMS={z1_s['rms']:5.3f}°, Peak={z1_s['peak']:5.3f}° | z2: RMS={z2_s['rms']:5.3f}°/s, Peak={z2_s['peak']:5.3f}°/s")

print("\n=== Initial Static Bias (First 1.5s) ===")
for ax_name, k in [("Roll Error", "e_r"), ("Pitch Error", "e_p"), ("Yaw Error", "e_y")]:
    mA = np.mean(dA[k][dA['t_sec'] <= 1.5])
    pkA = np.max(np.abs(dA[k][dA['t_sec'] <= 1.5]))
    mB = np.mean(dB[k][dB['t_sec'] <= 1.5])
    pkB = np.max(np.abs(dB[k][dB['t_sec'] <= 1.5]))
    print(f"{ax_name:10s} (0-1.5s) | Classical: Mean={mA:+5.3f}°, Peak={pkA:5.3f}° | LADRC: Mean={mB:+5.3f}°, Peak={pkB:5.3f}°")

print("\n=== Table 6: Execution Health & Runtime Metrics ===")
def get_runtime_metrics(cyc_rows, trans_rows, telem_dA):
    def get_max_c(col):
        v = [float(r[col]) for r in cyc_rows if col in r and r[col] != '']
        return max(v) if v else 0.0
    def get_max_t(col):
        v = [float(r[col]) for r in trans_rows if col in r and r[col] != '']
        return max(v) if v else 0.0
    
    max_step_err = np.max(np.abs(telem_dA['step_err']))
    mean_curr = np.mean(telem_dA['currents'])
    max_curr = np.max(telem_dA['currents'])
    max_age = np.max(telem_dA['fb_age'])
    max_sched = get_max_c('scheduler_gap_ms')
    max_tx = get_max_c('uart_tx_ms')
    max_drain = get_max_c('uart_drain_ms')
    return {
        "max_step_err": max_step_err, "mean_curr": mean_curr, "max_curr": max_curr,
        "max_age": max_age, "max_sched": max_sched, "max_tx": max_tx, "max_drain": max_drain
    }

rA = get_runtime_metrics(cyc_a, trans_a, dA)
rB = get_runtime_metrics(cyc_b, trans_b, dB)

print(f"Classical | Max Step Err={rA['max_step_err']} steps, Mean Curr={rA['mean_curr']:.1f}mA, Max Curr={rA['max_curr']}mA, Max Age={rA['max_age']:.1f}ms, Max Sched={rA['max_sched']:.1f}ms, Max Tx={rA['max_tx']:.1f}ms, Max Drain={rA['max_drain']:.1f}ms")
print(f"LADRC     | Max Step Err={rB['max_step_err']} steps, Mean Curr={rB['mean_curr']:.1f}mA, Max Curr={rB['max_curr']}mA, Max Age={rB['max_age']:.1f}ms, Max Sched={rB['max_sched']:.1f}ms, Max Tx={rB['max_tx']:.1f}ms, Max Drain={rB['max_drain']:.1f}ms")

