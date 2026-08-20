import math

from pus6_kinematics.analytical_6pus import Analytical6PusKinematics
from pus6_kinematics.geometry_config import load_geometry_config


def test_geometry_yaml_is_explicitly_preassembly_pending():
    config = load_geometry_config()
    assert config.schema_version == 1
    assert config.status == "pending_final_assembly_confirmation"
    assert len(config.base_anchors_m) == 6
    assert len(config.platform_anchors_body_m) == 6
    assert config.imu["status"] == "reserved_pending_final_installation"
    assert math.isclose(config.steps_per_meter * config.meters_per_step, 1.0, rel_tol=1e-12, abs_tol=1e-12)
    assert config.confirmation["base_anchors"] == "pending_final_assembly_measurement"
    assert config.storage_park_raw_steps == (0, 0, 0, 0, 0, 0)
    assert config.operating_home_raw_steps == (1000, 1000, 1000, 1000, 1000, 1000)
    assert (config.absolute_raw_min_steps, config.absolute_raw_max_steps) == (0, 2000)
    assert (config.normal_motion_min_steps, config.normal_motion_max_steps) == (200, 1800)


def test_default_kinematics_reads_yaml_without_behavior_change():
    config = load_geometry_config()
    kin = Analytical6PusKinematics()
    assert kin.geometry_config.path == config.path
    assert math.isclose(kin.nominal_height, 0.13261226796206452, rel_tol=0.0, abs_tol=1e-12)
    assert kin.base_anchors == [list(anchor) for anchor in config.base_anchors_m]
    assert kin.platform_anchors_body == [list(anchor) for anchor in config.platform_anchors_body_m]
    sliders = kin.inverse_kinematics([0.0, 0.0, kin.nominal_height, 0.0, 0.0, 0.0])
    assert all(math.isclose(value, 0.05, rel_tol=0.0, abs_tol=1e-9) for value in sliders)
