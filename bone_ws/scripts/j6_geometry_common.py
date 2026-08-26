"""Offline helpers shared by the J6 Phase 6 geometry tools."""

from __future__ import annotations

import hashlib
import math
import statistics
from pathlib import Path
from typing import Any, Mapping

import yaml

from pus6_kinematics.geometry_config import default_geometry_config_path

REQUIRED_MEASUREMENTS = (
    "base_radius_m",
    "platform_radius_m",
    "strut_lengths_m",
    "lambda_rad",
    "mu_rad",
    "nominal_height_m",
)


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must contain numeric values")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must contain finite values")
    return number


def _samples(value: Any, name: str) -> list[float]:
    if isinstance(value, Mapping):
        value = value.get("samples", value.get("values"))
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        value = [value]
    if not isinstance(value, (list, tuple)) or not value:
        raise ValueError(f"{name} must contain at least one numeric sample")
    return [_finite_number(item, name) for item in value]


def _summary(values: list[float], name: str) -> dict[str, Any]:
    if any(value <= 0.0 for value in values) and name not in {"lambda_rad", "mu_rad"}:
        raise ValueError(f"{name} samples must be positive")
    mean = statistics.fmean(values)
    spread = max(values) - min(values)
    return {
        "samples": list(values),
        "count": len(values),
        "mean": mean,
        "stddev_population": statistics.pstdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
        "spread": spread,
    }


def normalize_measurements(raw: Mapping[str, Any]) -> dict[str, Any]:
    """Normalize a user measurement mapping into an immutable record payload."""
    source = raw.get("measurements", raw)
    if not isinstance(source, Mapping):
        raise ValueError("measurements input must be a JSON object")

    aliases = {
        "strut_length_m": "strut_lengths_m",
        "rod_lengths_m": "strut_lengths_m",
    }
    values: dict[str, list[float]] = {}
    for key in REQUIRED_MEASUREMENTS:
        raw_key = key
        if raw_key not in source:
            for alias, canonical in aliases.items():
                if canonical == key and alias in source:
                    raw_key = alias
                    break
        if raw_key not in source:
            raise ValueError(f"missing required measured geometry field: {key}")
        values[key] = _samples(source[raw_key], key)

    if len(values["strut_lengths_m"]) < 6:
        raise ValueError("strut_lengths_m must contain at least six axis measurements")
    if any(value <= 0.0 for value in values["base_radius_m"] + values["platform_radius_m"]):
        raise ValueError("base and platform radii must be positive")
    if any(value <= 0.0 for value in values["lambda_rad"] + values["mu_rad"]):
        raise ValueError("lambda_rad and mu_rad must be positive")

    return {key: _summary(values[key], key) for key in REQUIRED_MEASUREMENTS}


def measurement_means(record: Mapping[str, Any]) -> dict[str, float]:
    measurements = record.get("measurements", record)
    return {
        key: float(measurements[key]["mean"])
        for key in REQUIRED_MEASUREMENTS
    }


def generate_anchor_coordinates(
    base_radius_m: float,
    platform_radius_m: float,
    lambda_rad: float,
    mu_rad: float,
) -> tuple[list[list[float]], list[list[float]]]:
    """Generate the six nominal anchor coordinates from measured scalar geometry."""
    c, d = base_radius_m, platform_radius_m
    lam, mu = lambda_rad, mu_rad
    base = [
        [-c * math.sin(lam / 2.0), c * math.cos(lam / 2.0), 0.0],
        [-c * math.cos(math.pi / 6.0 - lam / 2.0), -c * math.sin(math.pi / 6.0 - lam / 2.0), 0.0],
        [-c * math.cos(math.pi / 6.0 + lam / 2.0), -c * math.sin(math.pi / 6.0 + lam / 2.0), 0.0],
        [c * math.cos(math.pi / 6.0 + lam / 2.0), -c * math.sin(math.pi / 6.0 + lam / 2.0), 0.0],
        [c * math.cos(math.pi / 6.0 - lam / 2.0), -c * math.sin(math.pi / 6.0 - lam / 2.0), 0.0],
        [c * math.sin(lam / 2.0), c * math.cos(lam / 2.0), 0.0],
    ]
    platform = [
        [-d * math.cos(math.pi / 6.0 + mu / 2.0), d * math.sin(math.pi / 6.0 + mu / 2.0), 0.0],
        [-d * math.cos(math.pi / 6.0 - mu / 2.0), d * math.sin(math.pi / 6.0 - mu / 2.0), 0.0],
        [-d * math.sin(mu / 2.0), -d * math.cos(mu / 2.0), 0.0],
        [d * math.sin(mu / 2.0), -d * math.cos(mu / 2.0), 0.0],
        [d * math.cos(math.pi / 6.0 - mu / 2.0), d * math.sin(math.pi / 6.0 - mu / 2.0), 0.0],
        [d * math.cos(math.pi / 6.0 + mu / 2.0), d * math.sin(math.pi / 6.0 + mu / 2.0), 0.0],
    ]
    return base, platform


def load_raw_yaml(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser()
    with resolved.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("geometry YAML root must be an object")
    return value


def build_candidate_yaml(
    base_config_path: str | Path,
    measurement_record: Mapping[str, Any],
    *,
    measurement_source: str | None = None,
) -> dict[str, Any]:
    raw = load_raw_yaml(base_config_path)
    geometry = raw.get("geometry")
    if not isinstance(geometry, dict):
        raise ValueError("geometry YAML is missing a geometry mapping")
    means = measurement_means(measurement_record)
    base_anchors, platform_anchors = generate_anchor_coordinates(
        means["base_radius_m"],
        means["platform_radius_m"],
        means["lambda_rad"],
        means["mu_rad"],
    )
    raw["model_id"] = f"{raw.get('model_id', '6pus')}_phase6_candidate"
    raw["status"] = "candidate_phase6_validation"
    geometry.update({
        "base_radius_m": means["base_radius_m"],
        "platform_radius_m": means["platform_radius_m"],
        "strut_length_m": means["strut_lengths_m"],
        "lambda_rad": means["lambda_rad"],
        "mu_rad": means["mu_rad"],
        "nominal_height_m": means["nominal_height_m"],
        "base_anchors_m": base_anchors,
        "platform_anchors_body_m": platform_anchors,
    })
    confirmation = geometry.setdefault("confirmation", {})
    if not isinstance(confirmation, dict):
        confirmation = {}
        geometry["confirmation"] = confirmation
    confirmation.update({
        "phase6_candidate_only": True,
        "measurement_source": measurement_source,
        "anchor_source": "derived_from_measured_radii_and_angles",
    })
    source = raw.setdefault("source", {})
    if isinstance(source, dict):
        source["final_assembly_measurement"] = measurement_source or "provided_measurement_record"
    return raw


def write_yaml_once(value: Mapping[str, Any], output_path: str | Path) -> Path:
    output = Path(output_path).expanduser()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing candidate: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(dict(value), stream, sort_keys=False, allow_unicode=True)
    return output


def reject_production_overwrite(base_path: str | Path, output_path: str | Path) -> None:
    base = Path(base_path).expanduser().resolve()
    output = Path(output_path).expanduser().resolve()
    if output == base:
        raise ValueError("candidate output must not overwrite the production geometry YAML")


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def default_config_path() -> Path:
    return default_geometry_config_path()
