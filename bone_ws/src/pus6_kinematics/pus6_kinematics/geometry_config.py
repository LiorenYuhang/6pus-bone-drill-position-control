"""Validated YAML configuration for the 6PUS geometry contract.

The YAML contains a software reference model plus explicit pending markers for
values that require final mechanical assembly measurements.  This module is
deliberately independent of rclpy so J3 helpers and offline C2-HW tooling can
load the same configuration from a fresh terminal.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover - environment/package error
    raise RuntimeError("PyYAML is required to load the 6PUS geometry configuration") from exc


_CONFIG_FILENAME = "6pus_geometry.yaml"


@dataclass(frozen=True)
class GeometryConfig:
    """Typed values used by the numerical model and preassembly tooling."""

    path: Path
    schema_version: int
    model_id: str
    status: str
    base_radius_m: float
    platform_radius_m: float
    strut_length_m: float
    lambda_rad: float
    mu_rad: float
    nominal_height_m: float
    base_anchors_m: tuple[tuple[float, float, float], ...]
    platform_anchors_body_m: tuple[tuple[float, float, float], ...]
    steps_per_meter: float
    meters_per_step: float
    storage_park_raw_steps: tuple[int, ...]
    operating_home_raw_steps: tuple[int, ...]
    operating_home_stroke_m: tuple[float, ...]
    stroke_min_m: float
    stroke_max_m: float
    absolute_raw_min_steps: int
    absolute_raw_max_steps: int
    software_command_min_steps: int
    software_command_max_steps: int
    normal_motion_min_steps: int
    normal_motion_max_steps: int
    workspace_height_min_m: float
    workspace_height_max_m: float
    imu: dict[str, Any]
    confirmation: dict[str, Any]


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"geometry config field {name!r} must be a mapping")
    return value


def _float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"geometry config field {name!r} must be numeric")
    return float(value)


def _int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"geometry config field {name!r} must be an integer")
    return int(value)


def _vec(value: Any, name: str, length: int, cast=float) -> tuple[Any, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"geometry config field {name!r} must have length {length}")
    return tuple(cast(item) for item in value)


def _anchors(value: Any, name: str) -> tuple[tuple[float, float, float], ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"geometry config field {name!r} must contain six anchors")
    return tuple(_vec(item, f"{name}[{idx}]", 3, float) for idx, item in enumerate(value))


def _candidate_paths(explicit_path: str | os.PathLike[str] | None) -> list[Path]:
    if explicit_path:
        return [Path(explicit_path).expanduser()]

    paths: list[Path] = []
    env_path = os.environ.get("PUS6_GEOMETRY_CONFIG")
    if env_path:
        paths.append(Path(env_path).expanduser())

    # Source-tree path: .../pus6_kinematics/pus6_kinematics/geometry_config.py
    module_path = Path(__file__).resolve()
    paths.append(module_path.parents[1] / "config" / _CONFIG_FILENAME)

    # Installed ROS package path, only when the ROS environment is available.
    try:  # pragma: no cover - source-tree tests use the path above
        from ament_index_python.packages import get_package_share_directory

        paths.append(Path(get_package_share_directory("pus6_kinematics")) / "config" / _CONFIG_FILENAME)
    except Exception:
        pass
    return paths


def default_geometry_config_path() -> Path:
    """Return the resolved default config path without parsing it."""
    for path in _candidate_paths(None):
        if path.is_file():
            return path
    candidates = ", ".join(str(path) for path in _candidate_paths(None))
    raise FileNotFoundError(f"6PUS geometry YAML not found; checked: {candidates}")


def load_geometry_config(path: str | os.PathLike[str] | None = None) -> GeometryConfig:
    """Load and validate the 6PUS YAML geometry/configuration contract."""
    resolved = next((candidate for candidate in _candidate_paths(path) if candidate.is_file()), None)
    if resolved is None:
        requested = str(path) if path else "default locations"
        raise FileNotFoundError(f"6PUS geometry YAML not found at {requested}")

    with resolved.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    root = _mapping(raw, "root")
    geometry = _mapping(root.get("geometry"), "geometry")
    actuator = _mapping(root.get("actuator"), "actuator")
    limits = _mapping(actuator.get("stroke_limits_m"), "actuator.stroke_limits_m")
    absolute_limits = _mapping(actuator.get("absolute_raw_limits_steps"), "actuator.absolute_raw_limits_steps")
    command_limits = _mapping(actuator.get("software_command_limits_steps"), "actuator.software_command_limits_steps")
    normal_limits = _mapping(actuator.get("normal_motion_limits_steps"), "actuator.normal_motion_limits_steps")
    height_limits = _mapping(
        actuator.get("workspace_height_limits_m"), "actuator.workspace_height_limits_m"
    )

    schema_version = int(root.get("schema_version", 0))
    if schema_version != 1:
        raise ValueError(f"unsupported 6PUS geometry schema_version={schema_version}")

    base_anchors = _anchors(geometry.get("base_anchors_m"), "geometry.base_anchors_m")
    platform_anchors = _anchors(
        geometry.get("platform_anchors_body_m"), "geometry.platform_anchors_body_m"
    )
    steps_per_meter = _float(actuator.get("steps_per_meter"), "actuator.steps_per_meter")
    meters_per_step = _float(actuator.get("meters_per_step"), "actuator.meters_per_step")
    if steps_per_meter <= 0.0 or meters_per_step <= 0.0:
        raise ValueError("actuator conversion values must be positive")
    if not math.isclose(steps_per_meter * meters_per_step, 1.0, rel_tol=1e-9, abs_tol=1e-12):
        raise ValueError("actuator conversion values must be reciprocal")

    absolute_min = _int(absolute_limits.get("min"), "actuator.absolute_raw_limits_steps.min")
    absolute_max = _int(absolute_limits.get("max"), "actuator.absolute_raw_limits_steps.max")
    command_min = _int(command_limits.get("min"), "actuator.software_command_limits_steps.min")
    command_max = _int(command_limits.get("max"), "actuator.software_command_limits_steps.max")
    normal_min = _int(normal_limits.get("min"), "actuator.normal_motion_limits_steps.min")
    normal_max = _int(normal_limits.get("max"), "actuator.normal_motion_limits_steps.max")
    storage_park = _vec(actuator.get("storage_park_raw_steps"), "storage_park_raw_steps", 6, int)
    operating_home = _vec(actuator.get("operating_home_raw_steps"), "operating_home_raw_steps", 6, int)
    if storage_park != (0, 0, 0, 0, 0, 0):
        raise ValueError("STORAGE_PARK must be [0]*6 in the current safety contract")
    if operating_home != (1000, 1000, 1000, 1000, 1000, 1000):
        raise ValueError("OPERATING_HOME must be [1000]*6 in the current safety contract")
    if not (0 <= absolute_min <= absolute_max):
        raise ValueError("absolute raw limits must be ordered and non-negative")
    if not (absolute_min <= command_min <= command_max <= absolute_max):
        raise ValueError("software command limits must lie inside absolute raw limits")
    if not (command_min <= normal_min <= normal_max <= command_max):
        raise ValueError("normal motion limits must lie inside software command limits")

    return GeometryConfig(
        path=resolved,
        schema_version=schema_version,
        model_id=str(root.get("model_id", "")),
        status=str(root.get("status", "")),
        base_radius_m=_float(geometry.get("base_radius_m"), "geometry.base_radius_m"),
        platform_radius_m=_float(geometry.get("platform_radius_m"), "geometry.platform_radius_m"),
        strut_length_m=_float(geometry.get("strut_length_m"), "geometry.strut_length_m"),
        lambda_rad=_float(geometry.get("lambda_rad"), "geometry.lambda_rad"),
        mu_rad=_float(geometry.get("mu_rad"), "geometry.mu_rad"),
        nominal_height_m=_float(geometry.get("nominal_height_m"), "geometry.nominal_height_m"),
        base_anchors_m=base_anchors,
        platform_anchors_body_m=platform_anchors,
        steps_per_meter=steps_per_meter,
        meters_per_step=meters_per_step,
        storage_park_raw_steps=_vec(actuator.get("storage_park_raw_steps"), "storage_park_raw_steps", 6, int),
        operating_home_raw_steps=_vec(actuator.get("operating_home_raw_steps"), "operating_home_raw_steps", 6, int),
        operating_home_stroke_m=_vec(actuator.get("operating_home_stroke_m"), "operating_home_stroke_m", 6, float),
        stroke_min_m=_float(limits.get("min"), "actuator.stroke_limits_m.min"),
        stroke_max_m=_float(limits.get("max"), "actuator.stroke_limits_m.max"),
        absolute_raw_min_steps=absolute_min,
        absolute_raw_max_steps=absolute_max,
        software_command_min_steps=command_min,
        software_command_max_steps=command_max,
        normal_motion_min_steps=normal_min,
        normal_motion_max_steps=normal_max,
        workspace_height_min_m=_float(height_limits.get("min"), "actuator.workspace_height_limits_m.min"),
        workspace_height_max_m=_float(height_limits.get("max"), "actuator.workspace_height_limits_m.max"),
        imu=_mapping(root.get("imu", {}), "imu"),
        confirmation=_mapping(geometry.get("confirmation", {}), "geometry.confirmation"),
    )
