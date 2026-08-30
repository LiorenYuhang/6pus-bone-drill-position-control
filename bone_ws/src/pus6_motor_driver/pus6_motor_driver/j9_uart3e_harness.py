"""UART3E one-shot harness gate helpers.

This module keeps the startup gate separate from the motion policy so the
read-only audit can be classified from structured fields only.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any, Callable, Mapping

from .safety import (
    MAX_CURRENT_MA,
    MOTOR_IDS,
    StartupClassification,
    classify_startup_state,
)


READY_FOR_HOME = "READY_FOR_HOME"
NOT_READY = "NOT_READY"

_REQUIRED_STARTUP_FIELDS = (
    "status",
    "initial_positions_steps",
    "initial_currents_ma",
    "initial_faults",
)


@dataclass(frozen=True)
class UART3EStartupGateResult:
    ready: bool
    ready_state: str
    classification: str | None
    startup_status: str | None
    positions_steps: tuple[int, ...]
    currents_ma: tuple[int, ...]
    faults: tuple[int, ...]
    missing_fields: tuple[str, ...]
    reason: str | None


@dataclass(frozen=True)
class UART3EHarnessResult:
    startup_gate: UART3EStartupGateResult
    motion_started: bool
    final_state: str
    final_readonly_audit: dict[str, Any] | None = None


def _lookup_axis_value(axis_map: Mapping[Any, Any], axis_id: int) -> Any | None:
    if axis_id in axis_map:
        return axis_map[axis_id]
    key = str(axis_id)
    if key in axis_map:
        return axis_map[key]
    return None


def _extract_axis_vector(
    axis_map: Mapping[Any, Any],
    *,
    label: str,
) -> tuple[tuple[int, ...], tuple[str, ...]]:
    values: list[int] = []
    missing: list[str] = []
    for axis_id in MOTOR_IDS:
        raw = _lookup_axis_value(axis_map, axis_id)
        if raw is None:
            missing.append(f"{label}[ID{axis_id}]")
            continue
        if not isinstance(raw, int) or isinstance(raw, bool):
            raise ValueError(f"{label}[ID{axis_id}] must be an integer, got {type(raw).__name__}")
        values.append(int(raw))
    return tuple(values), tuple(missing)


def classify_uart3e_startup_audit(audit: Mapping[str, Any]) -> UART3EStartupGateResult:
    """Classify the structured J8-C audit without parsing console text.

    The only accepted startup condition for UART3E is VALID_PARK.
    """

    missing_fields = tuple(field for field in _REQUIRED_STARTUP_FIELDS if field not in audit)
    if missing_fields:
        return UART3EStartupGateResult(
            ready=False,
            ready_state=NOT_READY,
            classification=None,
            startup_status=str(audit.get("status")) if "status" in audit else None,
            positions_steps=(),
            currents_ma=(),
            faults=(),
            missing_fields=missing_fields,
            reason=f"missing required fields: {', '.join(missing_fields)}",
        )

    startup_status = str(audit["status"])
    if startup_status != "PASS":
        return UART3EStartupGateResult(
            ready=False,
            ready_state=NOT_READY,
            classification=None,
            startup_status=startup_status,
            positions_steps=(),
            currents_ma=(),
            faults=(),
            missing_fields=(),
            reason=f"startup audit status is not PASS: {startup_status}",
        )

    positions, missing_positions = _extract_axis_vector(
        audit["initial_positions_steps"],
        label="initial_positions_steps",
    )
    currents, missing_currents = _extract_axis_vector(
        audit["initial_currents_ma"],
        label="initial_currents_ma",
    )
    faults, missing_faults = _extract_axis_vector(
        audit["initial_faults"],
        label="initial_faults",
    )

    missing = missing_positions + missing_currents + missing_faults
    if missing:
        return UART3EStartupGateResult(
            ready=False,
            ready_state=NOT_READY,
            classification=None,
            startup_status=startup_status,
            positions_steps=positions,
            currents_ma=currents,
            faults=faults,
            missing_fields=missing,
            reason=f"missing required axis fields: {', '.join(missing)}",
        )

    if any(fault != 0 for fault in faults):
        return UART3EStartupGateResult(
            ready=False,
            ready_state=NOT_READY,
            classification=None,
            startup_status=startup_status,
            positions_steps=positions,
            currents_ma=currents,
            faults=faults,
            missing_fields=(),
            reason="startup audit reported a non-zero motor fault",
        )

    if any(current < 0 or current > MAX_CURRENT_MA for current in currents):
        return UART3EStartupGateResult(
            ready=False,
            ready_state=NOT_READY,
            classification=None,
            startup_status=startup_status,
            positions_steps=positions,
            currents_ma=currents,
            faults=faults,
            missing_fields=(),
            reason=f"startup audit reported an unsafe current above {MAX_CURRENT_MA} mA",
        )

    assessment = classify_startup_state(positions)
    classification = assessment.classification.value
    if assessment.classification == StartupClassification.VALID_PARK:
        return UART3EStartupGateResult(
            ready=True,
            ready_state=READY_FOR_HOME,
            classification=classification,
            startup_status=startup_status,
            positions_steps=positions,
            currents_ma=currents,
            faults=faults,
            missing_fields=(),
            reason=None,
        )

    return UART3EStartupGateResult(
        ready=False,
        ready_state=NOT_READY,
        classification=classification,
        startup_status=startup_status,
        positions_steps=positions,
        currents_ma=currents,
        faults=faults,
        missing_fields=(),
        reason=f"UART3E startup requires VALID_PARK, got {classification}",
    )


def run_uart3e_startup_dry_preflight(audit: Mapping[str, Any]) -> UART3EStartupGateResult:
    """Software-only preflight that never opens the motion path."""

    return classify_uart3e_startup_audit(audit)


def run_uart3e_harness(
    startup_audit: Mapping[str, Any],
    *,
    dry_preflight_only: bool = False,
    home_runner: Callable[[], Any] | None = None,
    park_runner: Callable[[], Any] | None = None,
    final_readonly_audit: Mapping[str, Any] | None = None,
) -> UART3EHarnessResult:
    """One-shot UART3E harness gate and motion wrapper.

    The dry-preflight mode returns READY_FOR_HOME without touching the motion
    path. The motion runners are intentionally injectable for testing.
    """

    gate = run_uart3e_startup_dry_preflight(startup_audit)
    if not gate.ready or dry_preflight_only:
        return UART3EHarnessResult(
            startup_gate=gate,
            motion_started=False,
            final_state=gate.ready_state,
            final_readonly_audit=None,
        )

    if home_runner is None or park_runner is None:
        raise ValueError("home_runner and park_runner are required when motion is enabled")

    home_runner()
    park_runner()
    return UART3EHarnessResult(
        startup_gate=gate,
        motion_started=True,
        final_state="COMPLETED",
        final_readonly_audit=dict(final_readonly_audit) if final_readonly_audit is not None else None,
    )


__all__ = [
    "NOT_READY",
    "READY_FOR_HOME",
    "UART3EHarnessResult",
    "UART3EStartupGateResult",
    "classify_uart3e_startup_audit",
    "run_uart3e_harness",
    "run_uart3e_startup_dry_preflight",
]
