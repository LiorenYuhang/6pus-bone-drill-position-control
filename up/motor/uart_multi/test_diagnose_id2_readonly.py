"""No-hardware tests for ID2 pure read-only diagnosis."""

from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from protocol import (  # noqa: E402
    CMD_WR_REGISTER,
    ReadRegisterResponse,
    StatusFeedback,
    pack_read_register_cmd,
    pack_read_status_cmd,
    pack_write_register_cmd,
    parse_frame,
)
from tools import diagnose_id2_readonly as tool  # noqa: E402


def _status(position, current=0, force=280, temperature=32, fault=0):
    return StatusFeedback(
        target_position=67,
        current_position=position,
        current_mA=current,
        force_g=force,
        force_adc=100,
        temperature=temperature,
        error_code=fault,
    )


class FakeBus:
    instances = []

    def __init__(self, port, *, timeout, retries):
        self.port = port
        self.timeout = timeout
        self.retries = retries
        self.events = []
        self.closed = False
        type(self).instances.append(self)

    def request(self, frame, **kwargs):
        parsed = parse_frame(frame)
        self.events.append(
            ("request", parsed.motor_id, parsed.command, parsed.register_addr)
        )
        return b""

    def close(self):
        if not self.closed:
            self.events.append(("close",))
            self.closed = True


class FakeMotor:
    def __init__(self, bus, motor_id, statuses):
        self.bus = bus
        self.motor_id = motor_id
        self.statuses = statuses
        self._status_index = 0

    def read_register(self, address):
        self.bus.request(
            pack_read_register_cmd(self.motor_id, address),
            expected_id=self.motor_id,
        )
        return ReadRegisterResponse(register_addr=address, values=(67,))

    def read_status(self):
        self.bus.request(
            pack_read_status_cmd(self.motor_id),
            expected_id=self.motor_id,
        )
        idx = min(self._status_index, len(self.statuses) - 1)
        self._status_index += 1
        return self.statuses[idx]


def _run(monkeypatch, tmp_path, statuses, *, samples=5):
    FakeBus.instances.clear()
    monkeypatch.setattr(tool, "UartBus", FakeBus)
    monkeypatch.setattr(tool.time, "sleep", lambda s: None)
    monkeypatch.setattr(tool, "_configure_logging", lambda p: None)

    monkeypatch.setattr(
        tool, "Motor", lambda bus, mid: FakeMotor(bus, mid, statuses)
    )

    log_file = tmp_path / "id2_diag.log"
    args = tool.parse_args(
        [
            "--port",
            "/dev/serial/by-id/test",
            "--samples",
            str(samples),
            "--log",
            str(log_file),
        ]
    )
    result = tool.run(args)
    bus = FakeBus.instances[0]
    return result, bus


def test_diagnose_id2_success_zero_writes(monkeypatch, tmp_path):
    statuses = [_status(67), _status(66), _status(67), _status(65), _status(67)]
    result, bus = _run(monkeypatch, tmp_path, statuses, samples=5)
    assert result == 0
    write_events = [
        e for e in bus.events if e[0] == "request" and e[2] == CMD_WR_REGISTER
    ]
    assert len(write_events) == 0


@pytest.mark.parametrize(
    ("frame", "match"),
    [
        (
            pack_write_register_cmd(2, 0x1A, [1]),
            "strictly read-only",
        ),
        (
            pack_read_register_cmd(1, 0x29),
            "outside fixed ID2 scope",
        ),
        (
            pack_read_register_cmd(2, 0x25),
            "is not allowed",
        ),
    ],
)
def test_audited_bus_blocks_invalid_requests(frame, match):
    delegate = FakeBus("test", timeout=0.05, retries=0)
    audited = tool.Id2ReadOnlyAuditedBus(delegate)
    with pytest.raises(tool.ReadOnlyViolation, match=match):
        audited.request(frame)


def test_cli_fixes_scope_parameters_and_avoids_id_flash_motion_helpers():
    assert tool.MOTOR_ID == 2
    assert tool.DEFAULT_SAMPLE_COUNT == 500

    source = (ROOT / "tools" / "diagnose_id2_readonly.py").read_text(
        encoding="utf-8"
    )
    tree = ast.parse(source)
    called_attributes = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert called_attributes.isdisjoint(
        {"set_motor_id", "save_parameters", "set_position", "set_speed", "move"}
    )
