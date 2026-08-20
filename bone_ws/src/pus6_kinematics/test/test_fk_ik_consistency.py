import math
import sys
from pathlib import Path
import numpy as np
import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.kinematics_interface import (
    ForwardKinematicsResult,
    KinematicsError,
    WorkspaceLimitError,
)

def so3_error_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    """Compute SO(3) geodesic distance between two rotation matrices in degrees."""
    R_diff = R1.T @ R2
    tr = (np.trace(R_diff) - 1.0) / 2.0
    tr = np.clip(tr, -1.0, 1.0)
    return float(np.degrees(np.arccos(tr)))

def test_fk_nominal_home_exact():
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height
    home_pose = [0.0, 0.0, z_nom, 0.0, 0.0, 0.0]

    # Home sliders: exactly 50.0mm (0.050m)
    sliders = kin.inverse_kinematics(home_pose)
    assert all(abs(s - 0.050) < 1e-6 for s in sliders)

    # FK solve
    fk_res = kin.forward_kinematics_with_metrics(sliders)
    assert fk_res.converged is True
    assert fk_res.residual_norm < 1e-10

    trans_err_mm = np.linalg.norm(np.array(fk_res.pose[:3]) - np.array(home_pose[:3])) * 1000.0
    R_true = np.array(kin._rotation_matrix_zyx(0.0, 0.0, 0.0))
    R_hat = np.array(fk_res.rotation_matrix)
    rot_err_deg = so3_error_deg(R_true, R_hat)

    assert trans_err_mm < 1e-6 # < 1 nanometer
    assert rot_err_deg < 1e-6

def test_fk_single_axis_cardinal_directions():
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height

    test_matrix = [
        ("Home", [0.0, 0.0, z_nom, 0.0, 0.0, 0.0]),
        ("+X (5mm)", [0.005, 0.0, z_nom, 0.0, 0.0, 0.0]),
        ("-X (5mm)", [-0.005, 0.0, z_nom, 0.0, 0.0, 0.0]),
        ("+Y (5mm)", [0.0, 0.005, z_nom, 0.0, 0.0, 0.0]),
        ("-Y (5mm)", [0.0, -0.005, z_nom, 0.0, 0.0, 0.0]),
        ("+Z (10mm)", [0.0, 0.0, z_nom + 0.010, 0.0, 0.0, 0.0]),
        ("-Z (10mm)", [0.0, 0.0, z_nom - 0.010, 0.0, 0.0, 0.0]),
        ("+Roll (0.05rad)", [0.0, 0.0, z_nom, 0.05, 0.0, 0.0]),
        ("-Roll (0.05rad)", [0.0, 0.0, z_nom, -0.05, 0.0, 0.0]),
        ("+Pitch (0.05rad)", [0.0, 0.0, z_nom, 0.0, 0.05, 0.0]),
        ("-Pitch (0.05rad)", [0.0, 0.0, z_nom, 0.0, -0.05, 0.0]),
        ("+Yaw (0.05rad)", [0.0, 0.0, z_nom, 0.0, 0.0, 0.05]),
        ("-Yaw (0.05rad)", [0.0, 0.0, z_nom, 0.0, 0.0, -0.05]),
    ]

    for name, pose_in in test_matrix:
        sliders = kin.inverse_kinematics(pose_in)
        fk_res = kin.forward_kinematics_with_metrics(sliders)

        assert fk_res.converged is True, f"Failed on {name}"
        assert fk_res.residual_norm < 1e-9, f"Residual too large on {name}: {fk_res.residual_norm}"

        trans_err_mm = np.linalg.norm(np.array(fk_res.pose[:3]) - np.array(pose_in[:3])) * 1000.0
        R_true = np.array(kin._rotation_matrix_zyx(pose_in[3], pose_in[4], pose_in[5]))
        R_hat = np.array(fk_res.rotation_matrix)
        rot_err_deg = so3_error_deg(R_true, R_hat)

        assert trans_err_mm < 1e-5, f"{name} translation error: {trans_err_mm:.6e} mm"
        assert rot_err_deg < 1e-5, f"{name} SO(3) rotation error: {rot_err_deg:.6e} deg"

def test_fk_combined_6dof_poses():
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height

    combined_cases = [
        ("M13_Composite", [0.005, -0.003, z_nom + 0.008, 0.03, -0.02, 0.025]),
        ("Extreme_Corner_1", [0.004, 0.004, z_nom - 0.006, 0.035, 0.035, -0.03]),
        ("Extreme_Corner_2", [-0.004, -0.004, z_nom + 0.006, -0.035, -0.035, 0.03]),
    ]

    for name, pose_in in combined_cases:
        sliders = kin.inverse_kinematics(pose_in)
        # Cold start (from Home)
        fk_res_cold = kin.forward_kinematics_with_metrics(sliders)
        # Warm start (from near pose)
        near_guess = [p + 0.001 for p in pose_in]
        fk_res_warm = kin.forward_kinematics_with_metrics(sliders, initial_guess=near_guess)

        assert fk_res_cold.converged is True
        assert fk_res_warm.converged is True

        # Warm start should take <= iterations than cold start
        assert fk_res_warm.iterations <= fk_res_cold.iterations

        trans_err_mm = np.linalg.norm(np.array(fk_res_cold.pose[:3]) - np.array(pose_in[:3])) * 1000.0
        R_true = np.array(kin._rotation_matrix_zyx(pose_in[3], pose_in[4], pose_in[5]))
        R_hat = np.array(fk_res_cold.rotation_matrix)
        rot_err_deg = so3_error_deg(R_true, R_hat)

        assert trans_err_mm < 1e-5
        assert rot_err_deg < 1e-5

def test_fk_random_safe_workspace_sampling_100():
    np.random.seed(42)
    kin = Analytical6PusKinematics()
    z_nom = kin.nominal_height

    N_samples = 100
    trans_errors_mm = []
    so3_errors_deg = []
    residual_norms = []
    iterations_list = []

    for _ in range(N_samples):
        # Sample pose within safe physical envelope
        x = np.random.uniform(-0.004, 0.004) # +/- 4mm
        y = np.random.uniform(-0.004, 0.004) # +/- 4mm
        z = z_nom + np.random.uniform(-0.008, 0.008) # +/- 8mm
        roll = np.random.uniform(-0.035, 0.035) # +/- ~2 deg
        pitch = np.random.uniform(-0.035, 0.035)
        yaw = np.random.uniform(-0.035, 0.035)

        pose_in = [x, y, z, roll, pitch, yaw]
        sliders = kin.inverse_kinematics(pose_in)
        fk_res = kin.forward_kinematics_with_metrics(sliders)

        assert fk_res.converged is True
        trans_err = np.linalg.norm(np.array(fk_res.pose[:3]) - np.array(pose_in[:3])) * 1000.0
        R_true = np.array(kin._rotation_matrix_zyx(roll, pitch, yaw))
        R_hat = np.array(fk_res.rotation_matrix)
        so3_err = so3_error_deg(R_true, R_hat)

        trans_errors_mm.append(trans_err)
        so3_errors_deg.append(so3_err)
        residual_norms.append(fk_res.residual_norm)
        iterations_list.append(fk_res.iterations)

    print(f"\n--- Random 100 Sample Results ---")
    print(f"Convergence Rate: 100% ({N_samples}/{N_samples})")
    print(f"Translation Error (mm): Max={np.max(trans_errors_mm):.4e}, Mean={np.mean(trans_errors_mm):.4e}")
    print(f"SO(3) Rotation Error (deg): Max={np.max(so3_errors_deg):.4e}, Mean={np.mean(so3_errors_deg):.4e}")
    print(f"Residual Norm (m): Max={np.max(residual_norms):.4e}, Mean={np.mean(residual_norms):.4e}")
    print(f"Iterations: P50={np.percentile(iterations_list, 50)}, Max={np.max(iterations_list)}, Mean={np.mean(iterations_list):.2f}")

    assert np.max(trans_errors_mm) < 1e-4 # < 0.1 micrometer
    assert np.max(so3_errors_deg) < 1e-4
    assert np.max(residual_norms) < 1e-9

def test_fk_failure_handling_out_of_reach():
    kin = Analytical6PusKinematics()
    # Pathological slider lengths that violate triangle inequality (strut length = 84mm)
    impossible_sliders = [0.050, 0.050, 0.050, 0.500, 0.050, 0.050] # 500mm slider
    with pytest.raises(WorkspaceLimitError, match="solver failed to converge"):
        kin.forward_kinematics(impossible_sliders, max_iter=10)

def test_fk_input_validation():
    kin = Analytical6PusKinematics()
    with pytest.raises(KinematicsError, match="must have length 6"):
        kin.forward_kinematics([0.05, 0.05, 0.05])
    with pytest.raises(KinematicsError, match="must have length 6"):
        kin.forward_kinematics_with_metrics([0.05]*6, initial_guess=[0.0]*3)
