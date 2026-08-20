"""No-hardware tests for the single-motor UART protocol and transport."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motor import Motor  # noqa: E402
from protocol import (  # noqa: E402
    CMD_RD_REGISTER,
    CMD_RD_STATUS,
    CMD_SAVE_SUCCESS,
    CMD_WR_REGISTER,
    FrameChecksumError,
    FrameFieldError,
    FrameHeaderError,
    FrameLengthError,
    FrameStreamParser,
    Register,
    RSP_FRAME_HEADER,
    calculate_checksum,
    pack_read_register_cmd,
    pack_read_status_cmd,
    pack_write_register_cmd,
    parse_frame,
    parse_save_responses,
    is_save_success_response,
    unpack_read_register_response,
    unpack_status_response,
    validate_response,
)
from uart_bus import NoResponseError, RetryExhaustedError, UartBus  # noqa: E402
from tools import assign_id  # noqa: E402


class FakeSerial:
    """Small deterministic pyserial substitute used by all transport tests."""

    def __init__(self, *, max_chunk: int = 4096) -> None:
        self.rx = bytearray()
        self.tx = []
        self.pending_response = b""
        self.max_chunk = max_chunk
        self.timeout = 0.05
        self.closed = False

    def reset_input_buffer(self) -> None:
        self.rx.clear()

    def reset_output_buffer(self) -> None:
        self.tx.clear()

    def write(self, data: bytes) -> int:
        self.tx.append(bytes(data))
        if self.pending_response:
            self.rx.extend(self.pending_response)
            self.pending_response = b""
        return len(data)

    def flush(self) -> None:
        return None

    def read(self, size: int = 1) -> bytes:
        if not self.rx:
            return b""
        count = min(size, self.max_chunk, len(self.rx))
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result

    def close(self) -> None:
        self.closed = True


def response_frame(
    motor_id: int,
    command: int,
    register_addr: int,
    payload: bytes,
) -> bytes:
    length = 3 + len(payload)
    body = bytes((length, motor_id, command))
    body += register_addr.to_bytes(2, "little") + payload
    return RSP_FRAME_HEADER + body + bytes((calculate_checksum(body, start=0),))


def status_response(motor_id: int = 1, command: int = CMD_RD_STATUS) -> bytes:
    # target, actual, current, force, force ADC, temperature, fault
    payload = bytes.fromhex("E803 D007 6400 2C01 0A00 20 00")
    return response_frame(motor_id, command, 0, payload)


def save_response_pair(motor_id: int = 1):
    first = response_frame(
        motor_id,
        CMD_WR_REGISTER,
        Register.SAVE_PARAMS,
        bytes(12),
    )
    second = response_frame(
        motor_id,
        CMD_SAVE_SUCCESS,
        Register.SAVE_PARAMS,
        bytes(12),
    )
    return first, second


REAL_M2_SAVE_FIRST = bytes.fromhex(
    "AA 55 0F 02 32 1C 00 00 00 00 00 00 00 F9 00 7A 08 22 00 FC"
)
REAL_M2_SAVE_SECOND = bytes.fromhex(
    "AA 55 0F 02 40 1C 00 00 00 00 00 00 00 F9 00 7A 08 22 00 0A"
)


def bus_with_response(response: bytes, *, max_chunk: int = 4096, retries: int = 0):
    fake = FakeSerial(max_chunk=max_chunk)
    bus = UartBus(serial_instance=fake, timeout=0.001, retries=retries)
    fake.pending_response = response
    return bus, fake


def test_command_packing_matches_length_and_checksum_rules():
    assert pack_read_status_cmd(1) == bytes.fromhex("55 AA 03 01 30 00 00 34")
    read = pack_read_register_cmd(1, Register.ID_ADDR, 1)
    assert read == bytes.fromhex("55 AA 04 01 31 16 00 01 4D")
    write = pack_write_register_cmd(1, Register.ID_ADDR, [2])
    assert write == bytes.fromhex("55 AA 05 01 32 16 00 02 00 50")
    assert len(write) == 5 + write[2]


def test_status_response_parses_signed_and_unsigned_fields():
    result = unpack_status_response(status_response())
    assert result.target_position == 1000
    assert result.current_position == 2000
    assert result.current_mA == 100
    assert result.force_g == 300
    assert result.force_adc == 10
    assert result.temperature == 32
    assert result.error_code == 0


def test_read_register_response_parses_split_frame():
    raw = response_frame(1, CMD_RD_REGISTER, Register.ID_ADDR, bytes.fromhex("02 00"))
    bus, fake = bus_with_response(raw, max_chunk=1)
    motor = Motor(bus, 1)
    result = motor.read_register(Register.ID_ADDR)
    assert result.values == [2]
    assert fake.tx == [pack_read_register_cmd(1, Register.ID_ADDR, 1)]


def test_length_error_is_rejected():
    raw = bytearray(status_response())
    raw[2] = 0x0E
    with pytest.raises(FrameLengthError):
        parse_frame(bytes(raw))


def test_checksum_error_is_rejected():
    raw = bytearray(status_response())
    raw[-1] ^= 0x01
    with pytest.raises(FrameChecksumError):
        parse_frame(bytes(raw))


def test_wrong_response_id_is_rejected():
    with pytest.raises(FrameFieldError):
        validate_response(status_response(motor_id=2), expected_id=1)


def test_stream_parser_handles_partial_and_concatenated_frames():
    first = status_response(1)
    second = status_response(1)
    parser = FrameStreamParser()
    assert parser.feed(first[:4]) == []
    frames = parser.feed(first[4:] + second)
    assert len(frames) == 2
    assert frames[0].raw == first
    assert frames[1].raw == second


def test_save_parameters_requires_cmd_0x40_and_save_register():
    first, second = save_response_pair()
    result = parse_save_responses([first, second], expected_id=1)
    assert result.success is True
    assert result.second.command == CMD_SAVE_SUCCESS
    assert result.second.register_addr == Register.SAVE_PARAMS


def test_real_m2_save_response_pair_is_successful():
    result = parse_save_responses(
        [REAL_M2_SAVE_FIRST, REAL_M2_SAVE_SECOND],
        expected_id=2,
    )
    assert result.success is True
    assert result.first.command == CMD_WR_REGISTER
    assert result.first.register_addr == Register.SAVE_PARAMS
    assert result.second.command == CMD_SAVE_SUCCESS
    assert result.second.register_addr == Register.SAVE_PARAMS
    assert is_save_success_response(REAL_M2_SAVE_SECOND, expected_id=2)


def test_save_success_rejects_any_wrong_required_field():
    wrong_header = bytearray(REAL_M2_SAVE_SECOND)
    wrong_header[0] = 0x55
    wrong_length = response_frame(2, CMD_SAVE_SUCCESS, Register.SAVE_PARAMS, bytes(10))
    wrong_id = response_frame(3, CMD_SAVE_SUCCESS, Register.SAVE_PARAMS, bytes(12))
    wrong_command = response_frame(2, 0x41, Register.SAVE_PARAMS, bytes(12))
    wrong_register = response_frame(2, CMD_SAVE_SUCCESS, 0x001D, bytes(12))
    wrong_checksum = bytearray(REAL_M2_SAVE_SECOND)
    wrong_checksum[-1] ^= 0x01

    with pytest.raises(FrameHeaderError):
        parse_save_responses(
            [REAL_M2_SAVE_FIRST, bytes(wrong_header)],
            expected_id=2,
        )
    with pytest.raises(FrameLengthError):
        parse_save_responses([REAL_M2_SAVE_FIRST, wrong_length], expected_id=2)
    with pytest.raises(FrameFieldError):
        parse_save_responses([REAL_M2_SAVE_FIRST, wrong_id], expected_id=2)
    with pytest.raises(FrameFieldError):
        parse_save_responses([REAL_M2_SAVE_FIRST, wrong_command], expected_id=2)
    with pytest.raises(FrameFieldError):
        parse_save_responses([REAL_M2_SAVE_FIRST, wrong_register], expected_id=2)
    with pytest.raises(FrameChecksumError):
        parse_save_responses(
            [REAL_M2_SAVE_FIRST, bytes(wrong_checksum)],
            expected_id=2,
        )
    assert not is_save_success_response(bytes(wrong_header), expected_id=2)
    assert not is_save_success_response(wrong_length, expected_id=2)
    assert not is_save_success_response(wrong_id, expected_id=2)
    assert not is_save_success_response(wrong_command, expected_id=2)
    assert not is_save_success_response(wrong_register, expected_id=2)
    assert not is_save_success_response(bytes(wrong_checksum), expected_id=2)


def test_save_parameters_is_consumed_as_two_length_delimited_frames():
    first, second = save_response_pair()
    bus, fake = bus_with_response(first + second, max_chunk=2)
    result = Motor(bus, 1).save_parameters()
    assert result.success is True
    assert len(fake.tx) == 1
    assert fake.tx[0] == pack_write_register_cmd(1, Register.SAVE_PARAMS, [1])
    assert fake.rx == bytearray()


def test_timeout_is_reported_after_configured_attempts():
    fake = FakeSerial()
    bus = UartBus(serial_instance=fake, timeout=0.001, retries=0)
    with pytest.raises(RetryExhaustedError) as caught:
        bus.request(pack_read_status_cmd(1), expected_id=1, expected_command=CMD_RD_STATUS)
    assert isinstance(caught.value.last_error, NoResponseError)


def test_motor_id_change_matches_write_response_to_old_id():
    bus, fake = bus_with_response(status_response(1, command=CMD_WR_REGISTER))
    motor = Motor(bus, 1)
    motor.set_motor_id(2)
    assert motor.motor_id == 2
    assert fake.tx == [pack_write_register_cmd(1, Register.ID_ADDR, [2])]


def test_same_id_change_is_refused_without_transmitting():
    fake = FakeSerial()
    bus = UartBus(serial_instance=fake, timeout=0.001, retries=0)
    with pytest.raises(ValueError):
        Motor(bus, 1).set_motor_id(1)
    assert fake.tx == []


def test_id_write_is_not_retried_when_ack_is_missing():
    fake = FakeSerial()
    bus = UartBus(serial_instance=fake, timeout=0.001, retries=3)
    with pytest.raises(RetryExhaustedError):
        Motor(bus, 1).set_motor_id(2)
    assert fake.tx == [pack_write_register_cmd(1, Register.ID_ADDR, [2])]


def test_assign_id_main_passes_all_parsed_arguments_to_run(monkeypatch, tmp_path):
    captured = {}

    def fake_run(args):
        captured["args"] = args
        return 17

    log_path = tmp_path / "m1.log"
    monkeypatch.setattr(assign_id, "run", fake_run)
    result = assign_id.main([
        "--port", "/dev/ttyTEST0",
        "--old-id", "3",
        "--new-id", "4",
        "--timeout", "0.125",
        "--retries", "5",
        "--reconnect-timeout", "12.5",
        "--log", str(log_path),
    ])

    args = captured["args"]
    assert result == 17
    assert args.port == "/dev/ttyTEST0"
    assert args.old_id == 3
    assert args.new_id == 4
    assert args.timeout == 0.125
    assert args.retries == 5
    assert args.reconnect_timeout == 12.5
    assert args.log == str(log_path)


def test_m1_read_only_flow_skips_id_write_and_flash_save(monkeypatch, tmp_path):
    calls = {"read_bus_ids": [], "set_id": 0, "save": 0, "events": []}

    class FakeBus:
        def __init__(self, port, *, timeout, retries):
            self.instance_id = 1 + sum(
                event[0] == "open" for event in calls["events"]
            )
            calls["events"].append(
                ("open", self.instance_id, port, timeout, retries)
            )

        def __enter__(self):
            calls["events"].append(("enter", self.instance_id))
            return self

        def __exit__(self, exc_type, exc_value, traceback):
            calls["events"].append(("close", self.instance_id))
            return False

    class FakeMotor:
        def __init__(self, bus, motor_id):
            assert motor_id == 1
            self.bus = bus

        def read_register(self, register_addr):
            assert register_addr == Register.ID_ADDR
            calls["read_bus_ids"].append(self.bus.instance_id)
            return type("Response", (), {"values": [1]})()

        def set_motor_id(self, new_id):
            calls["set_id"] += 1

        def save_parameters(self):
            calls["save"] += 1

    answers = iter(("YES", "YES"))
    port_states = iter((False, True))
    monkeypatch.setattr("builtins.input", lambda prompt: next(answers))
    monkeypatch.setattr(assign_id, "UartBus", FakeBus)
    monkeypatch.setattr(assign_id, "Motor", FakeMotor)
    monkeypatch.setattr(assign_id, "_configure_logging", lambda path: None)
    monkeypatch.setattr(
        assign_id,
        "_port_exists",
        lambda port: calls["events"].append(("exists", port)) or next(port_states),
    )
    monkeypatch.setattr(assign_id.time, "sleep", lambda seconds: None)

    stable_port = "/dev/serial/by-id/usb-test-if00-port0"
    args = assign_id.parse_args([
        "--port", stable_port,
        "--old-id", "1",
        "--new-id", "1",
        "--timeout", "0.2",
        "--retries", "4",
        "--reconnect-timeout", "1.0",
        "--log", str(tmp_path / "m1.log"),
    ])
    assert assign_id.run(args) == 0
    opens = [event for event in calls["events"] if event[0] == "open"]
    assert opens == [
        ("open", 1, stable_port, 0.2, 4),
        ("open", 2, stable_port, 0.2, 4),
    ]
    assert calls["events"].index(("close", 1)) < calls["events"].index(opens[1])
    assert calls["read_bus_ids"] == [1, 2]
    assert calls["set_id"] == 0
    assert calls["save"] == 0


def test_reconnect_wait_has_explicit_timeout(monkeypatch):
    times = iter((10.0, 10.6))
    monkeypatch.setattr(assign_id, "_port_exists", lambda port: False)
    monkeypatch.setattr(assign_id.time, "monotonic", lambda: next(times))

    with pytest.raises(TimeoutError, match="重新出现并可用超时"):
        assign_id._open_reconnected_bus(
            "/dev/serial/by-id/usb-missing",
            timeout=0.05,
            retries=2,
            reconnect_timeout=0.5,
        )
