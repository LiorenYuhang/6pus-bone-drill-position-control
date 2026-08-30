"""Regression tests for the UART3E read-only startup gate."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from pus6_motor_driver.j9_uart3e_harness import (
    NOT_READY,
    READY_FOR_HOME,
    classify_uart3e_startup_audit,
    run_uart3e_harness,
    run_uart3e_startup_dry_preflight,
)


def _audit(
    *,
    status: str = "PASS",
    positions: tuple[int, ...] = (0, 0, 0, 0, 0, 0),
    currents: tuple[int, ...] = (0, 0, 0, 0, 0, 0),
    faults: tuple[int, ...] = (0, 0, 0, 0, 0, 0),
    include_positions: bool = True,
    include_currents: bool = True,
    include_faults: bool = True,
) -> dict[str, object]:
    audit: dict[str, object] = {"status": status}
    if include_positions:
        audit["initial_positions_steps"] = {idx + 1: pos for idx, pos in enumerate(positions)}
    if include_currents:
        audit["initial_currents_ma"] = {idx + 1: curr for idx, curr in enumerate(currents)}
    if include_faults:
        audit["initial_faults"] = {idx + 1: fault for idx, fault in enumerate(faults)}
    return audit


def test_uart3e_gate_accepts_valid_park() -> None:
    gate = classify_uart3e_startup_audit(_audit(positions=(0, 1, 0, 0, 1, 1)))
    assert gate.ready is True
    assert gate.ready_state == READY_FOR_HOME
    assert gate.classification == "valid_park"
    assert gate.reason is None


def test_uart3e_gate_rejects_valid_home() -> None:
    gate = classify_uart3e_startup_audit(_audit(positions=(1000, 1000, 1000, 1000, 1000, 1000)))
    assert gate.ready is False
    assert gate.ready_state == NOT_READY
    assert gate.classification == "valid_home"
    assert "VALID_PARK" in gate.reason


def test_uart3e_gate_rejects_valid_in_envelope() -> None:
    gate = classify_uart3e_startup_audit(_audit(positions=(743, 744, 742, 742, 743, 745)))
    assert gate.ready is False
    assert gate.ready_state == NOT_READY
    assert gate.classification == "valid_in_envelope"
    assert "VALID_PARK" in gate.reason


@pytest.mark.parametrize(
    ("label", "audit"),
    [
        ("fault", _audit(faults=(0, 1, 0, 0, 0, 0))),
        ("current", _audit(currents=(0, 0, 501, 0, 0, 0))),
        ("timeout", _audit(status="FAIL")),
    ],
)
def test_uart3e_gate_rejects_unsafe_states(label: str, audit: dict[str, object]) -> None:
    gate = classify_uart3e_startup_audit(audit)
    assert gate.ready is False
    assert gate.ready_state == NOT_READY
    assert gate.reason is not None
    assert label in {"fault", "current", "timeout"}


def test_uart3e_gate_fails_closed_on_missing_required_fields() -> None:
    gate = classify_uart3e_startup_audit(_audit(include_faults=False))
    assert gate.ready is False
    assert gate.ready_state == NOT_READY
    assert "missing required fields" in gate.reason


def test_uart3e_dry_preflight_reaches_ready_for_home_without_motion() -> None:
    calls: list[str] = []

    def forbidden_motion() -> None:
        calls.append("motion")
        raise AssertionError("motion path must not be opened in dry preflight")

    result = run_uart3e_harness(
        _audit(positions=(0, 1, 0, 0, 1, 1)),
        dry_preflight_only=True,
        home_runner=forbidden_motion,
        park_runner=forbidden_motion,
    )

    assert result.startup_gate.ready is True
    assert result.startup_gate.ready_state == READY_FOR_HOME
    assert result.motion_started is False
    assert result.final_state == READY_FOR_HOME
    assert calls == []


def test_uart3e_dry_preflight_blocks_motion_for_non_park_startup() -> None:
    calls: list[str] = []

    def forbidden_motion() -> None:
        calls.append("motion")
        raise AssertionError("motion path must not be opened")

    result = run_uart3e_startup_dry_preflight(_audit(positions=(1000, 1000, 1000, 1000, 1000, 1000)))

    assert result.ready is False
    assert result.ready_state == NOT_READY
    assert result.classification == "valid_home"
    assert calls == []

