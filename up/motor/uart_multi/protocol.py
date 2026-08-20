"""Protocol primitives for the INSIM LAF D-type UART interface.

The layout and constants in this module are taken from PRJ-01-TS-U-007
V2.0.4 and the existing ``inspire_motor_uart`` reference implementation.
This module contains no serial-port or motion-control code.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional

CMD_FRAME_HEADER = bytes((0x55, 0xAA))
RSP_FRAME_HEADER = bytes((0xAA, 0x55))

CMD_RD_STATUS = 0x30
CMD_RD_REGISTER = 0x31
CMD_WR_REGISTER = 0x32
CMD_SAVE_SUCCESS = 0x40
BROADCAST_ID = 0xFF


class Register:
    """Register addresses used by the single-motor communication phase."""

    ID_ADDR = 0x16
    SAVE_PARAMS = 0x1C
    ACTUAL_POSITION = 0x2A
    CURRENT_VALUE = 0x2B
    ACTUAL_FORCE = 0x2C
    TEMPERATURE = 0x2E
    ERROR_CODE = 0x2F

MIN_DATA_LENGTH = 3  # CMD (1) + register address (2)
MAX_FRAME_LENGTH = 5 + 0xFF
SAVE_RESPONSE_LENGTH = 0x0F


class ProtocolError(ValueError):
    """Base class for malformed or semantically invalid protocol frames."""


class FrameHeaderError(ProtocolError):
    """The frame header is not the expected command or response header."""


class FrameLengthError(ProtocolError):
    """The declared length does not match the received frame."""


class FrameChecksumError(ProtocolError):
    """The frame checksum is invalid."""


class FrameFieldError(ProtocolError):
    """A frame field such as ID, command, or register is unexpected."""


def _validate_id(motor_id: int, *, allow_broadcast: bool = False) -> None:
    if not isinstance(motor_id, int):
        raise FrameFieldError("motor_id must be an integer")
    if 1 <= motor_id <= 0xFE:
        return
    if allow_broadcast and motor_id == BROADCAST_ID:
        return
    raise FrameFieldError("motor ID must be in 1..254")


def _validate_register(register_addr: int) -> None:
    if not isinstance(register_addr, int) or not 0 <= register_addr <= 0xFFFF:
        raise FrameFieldError("register address must be in 0..65535")


def calculate_checksum(data: bytes, start: int = 2, end: Optional[int] = None) -> int:
    """Return the low byte of the sum, excluding the two-byte frame header.

    For a complete frame, pass ``data[:-1]`` or ``end=-1`` so the checksum
    byte itself is not included.
    """

    return sum(data[start:end]) & 0xFF


def _build_frame(
    motor_id: int,
    command: int,
    register_addr: int,
    payload: bytes,
    *,
    header: bytes = CMD_FRAME_HEADER,
    allow_broadcast: bool = False,
) -> bytes:
    _validate_id(motor_id, allow_broadcast=allow_broadcast)
    if not isinstance(command, int) or not 0 <= command <= 0xFF:
        raise FrameFieldError("command must be a byte")
    _validate_register(register_addr)
    data_length = 3 + len(payload)
    if data_length > 0xFF:
        raise FrameLengthError("payload is too large for the one-byte length field")
    body = bytes((data_length, motor_id, command))
    body += register_addr.to_bytes(2, "little") + payload
    return header + body + bytes((calculate_checksum(body, start=0),))


def pack_read_status_cmd(motor_id: int = 1) -> bytes:
    """Build CMD_RD_STATUS (0x30), whose register address is 0x0000."""

    return _build_frame(motor_id, CMD_RD_STATUS, 0x0000, b"")


def pack_read_register_cmd(motor_id: int, register_addr: int, count: int = 1) -> bytes:
    """Build CMD_RD_REGISTER (0x31)."""

    if not isinstance(count, int) or not 1 <= count <= 0xFF:
        raise FrameFieldError("register count must be in 1..255")
    return _build_frame(motor_id, CMD_RD_REGISTER, register_addr, bytes((count,)))


def pack_write_register_cmd(
    motor_id: int,
    register_addr: int,
    data: Iterable[int],
) -> bytes:
    """Build CMD_WR_REGISTER (0x32), encoding each register as unsigned LE16."""

    values = list(data)
    if not values:
        raise FrameFieldError("at least one register value is required")
    payload = bytearray()
    for value in values:
        if not isinstance(value, int) or not 0 <= value <= 0xFFFF:
            raise FrameFieldError("register values must be in 0..65535")
        payload.extend(value.to_bytes(2, "little"))
    return _build_frame(motor_id, CMD_WR_REGISTER, register_addr, bytes(payload))


@dataclass(frozen=True)
class Frame:
    """A validated complete frame."""

    raw: bytes
    header: bytes
    length: int
    motor_id: int
    command: int
    register_addr: int
    data: bytes


def parse_frame(raw: bytes, *, expected_header: Optional[bytes] = None) -> Frame:
    """Parse one complete frame and validate header, length, and checksum."""

    if not isinstance(raw, (bytes, bytearray)):
        raise ProtocolError("frame must be bytes-like")
    raw = bytes(raw)
    if len(raw) < 5 + MIN_DATA_LENGTH:
        raise FrameLengthError("frame is shorter than the minimum protocol frame")
    header = raw[:2]
    if expected_header is not None:
        if header != expected_header:
            raise FrameHeaderError(
                f"unexpected header {header.hex(' ')}, expected {expected_header.hex(' ')}"
            )
    elif header not in (CMD_FRAME_HEADER, RSP_FRAME_HEADER):
        raise FrameHeaderError(f"unknown frame header {header.hex(' ')}")

    data_length = raw[2]
    if data_length < MIN_DATA_LENGTH:
        raise FrameLengthError(f"invalid declared data length {data_length}")
    expected_total = 5 + data_length
    if len(raw) != expected_total:
        raise FrameLengthError(
            f"declared data length {data_length} requires {expected_total} bytes, got {len(raw)}"
        )
    expected_checksum = calculate_checksum(raw, start=2, end=-1)
    if raw[-1] != expected_checksum:
        raise FrameChecksumError(
            f"checksum 0x{raw[-1]:02X} != 0x{expected_checksum:02X}"
        )
    motor_id = raw[3]
    if motor_id != BROADCAST_ID:
        _validate_id(motor_id)
    register_addr = int.from_bytes(raw[5:7], "little")
    return Frame(
        raw=raw,
        header=header,
        length=data_length,
        motor_id=motor_id,
        command=raw[4],
        register_addr=register_addr,
        data=raw[7:-1],
    )


def validate_response(
    raw: bytes,
    *,
    expected_id: Optional[int] = None,
    expected_command: Optional[int] = None,
) -> Frame:
    """Parse a response and optionally validate its ID and command."""

    frame = parse_frame(raw, expected_header=RSP_FRAME_HEADER)
    if expected_id is not None and frame.motor_id != expected_id:
        raise FrameFieldError(
            f"response ID {frame.motor_id} != expected ID {expected_id}"
        )
    if expected_command is not None and frame.command != expected_command:
        raise FrameFieldError(
            f"response command 0x{frame.command:02X} != expected 0x{expected_command:02X}"
        )
    return frame


class FrameStreamParser:
    """Incrementally recover complete response frames from arbitrary chunks."""

    def __init__(self, *, expected_header: bytes = RSP_FRAME_HEADER) -> None:
        self.expected_header = expected_header
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> List[Frame]:
        if not isinstance(chunk, (bytes, bytearray)):
            raise ProtocolError("stream chunk must be bytes-like")
        self._buffer.extend(chunk)
        frames: List[Frame] = []
        while True:
            start = self._buffer.find(self.expected_header)
            if start < 0:
                # Retain a possible first header byte for the next chunk.
                keep = 1 if self._buffer[-1:] == self.expected_header[:1] else 0
                if keep:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 3:
                break
            data_length = self._buffer[2]
            if data_length < MIN_DATA_LENGTH:
                del self._buffer[:2]
                raise FrameLengthError(f"invalid declared data length {data_length}")
            total = 5 + data_length
            if total > MAX_FRAME_LENGTH:
                del self._buffer[:2]
                raise FrameLengthError(f"frame exceeds maximum length: {total}")
            if len(self._buffer) < total:
                break
            raw = bytes(self._buffer[:total])
            del self._buffer[:total]
            frames.append(parse_frame(raw, expected_header=self.expected_header))
        return frames

    def clear(self) -> None:
        self._buffer.clear()


@dataclass(frozen=True)
class StatusFeedback:
    target_position: int
    current_position: int
    current_mA: int
    force_g: int
    force_adc: int
    temperature: int
    error_code: int


@dataclass(frozen=True)
class ReadRegisterResponse:
    register_addr: int
    values: List[int]


@dataclass(frozen=True)
class SaveParametersResponse:
    first: Frame
    second: Frame
    success: bool


def unpack_status_response(raw: bytes) -> StatusFeedback:
    """Parse a status response or the normal write-register status response."""

    frame = validate_response(raw)
    if frame.command not in (CMD_RD_STATUS, CMD_WR_REGISTER):
        raise FrameFieldError(f"not a status response: command 0x{frame.command:02X}")
    if frame.length != 0x0F or len(frame.data) != 12:
        raise FrameLengthError("status response must have a 15-byte data segment")
    return StatusFeedback(
        target_position=int.from_bytes(frame.data[0:2], "little", signed=True),
        current_position=int.from_bytes(frame.data[2:4], "little", signed=True),
        current_mA=int.from_bytes(frame.data[4:6], "little", signed=False),
        force_g=int.from_bytes(frame.data[6:8], "little", signed=True),
        force_adc=int.from_bytes(frame.data[8:10], "little", signed=False),
        temperature=int.from_bytes(frame.data[10:11], "little", signed=True),
        error_code=frame.data[11],
    )


def unpack_read_register_response(
    raw: bytes,
    expected_addr: int,
    expected_count: int = 1,
) -> ReadRegisterResponse:
    """Parse a read-register response and validate address and value count."""

    if expected_count < 1:
        raise FrameFieldError("expected_count must be positive")
    frame = validate_response(raw, expected_command=CMD_RD_REGISTER)
    if frame.register_addr != expected_addr:
        raise FrameFieldError(
            f"response register 0x{frame.register_addr:04X} != expected 0x{expected_addr:04X}"
        )
    if len(frame.data) != 2 * expected_count:
        raise FrameLengthError(
            f"expected {expected_count} register values, got {len(frame.data) // 2}"
        )
    values = [
        int.from_bytes(frame.data[offset:offset + 2], "little")
        for offset in range(0, len(frame.data), 2)
    ]
    return ReadRegisterResponse(frame.register_addr, values)


def unpack_write_response(raw: bytes) -> StatusFeedback:
    """Parse the normal first response to a write-register command."""

    frame = validate_response(raw, expected_command=CMD_WR_REGISTER)
    return unpack_status_response(frame.raw)


def parse_save_responses(
    raw_frames: Iterable[bytes],
    *,
    expected_id: Optional[int] = None,
) -> SaveParametersResponse:
    """Validate the two responses returned by SAVE_PARAMS.

    The first response is the normal CMD_WR_REGISTER status response. Real
    hardware confirms that the second response is a standard frame whose CMD
    is 0x40 and whose register address is SAVE_PARAMS (0x001C).
    """

    frames = list(raw_frames)
    if len(frames) != 2:
        raise FrameLengthError(f"save parameters requires 2 response frames, got {len(frames)}")
    first = validate_response(
        frames[0], expected_id=expected_id, expected_command=CMD_WR_REGISTER
    )
    if first.register_addr != Register.SAVE_PARAMS:
        raise FrameFieldError(
            f"save response 1 register 0x{first.register_addr:04X} != 0x{Register.SAVE_PARAMS:04X}"
        )
    unpack_status_response(first.raw)
    second = validate_response(
        frames[1], expected_id=expected_id, expected_command=CMD_SAVE_SUCCESS
    )
    if second.length != SAVE_RESPONSE_LENGTH:
        raise FrameLengthError(
            f"save response 2 has data length {second.length}, expected {SAVE_RESPONSE_LENGTH}"
        )
    if second.register_addr != Register.SAVE_PARAMS:
        raise FrameFieldError(
            f"save response 2 register 0x{second.register_addr:04X} != 0x{Register.SAVE_PARAMS:04X}"
        )
    return SaveParametersResponse(first=first, second=second, success=True)


def is_save_success_response(raw: bytes, *, expected_id: Optional[int] = None) -> bool:
    """Validate the second save response, including CMD and register fields."""

    try:
        frame = validate_response(
            raw,
            expected_id=expected_id,
            expected_command=CMD_SAVE_SUCCESS,
        )
    except ProtocolError:
        return False
    return (
        frame.length == SAVE_RESPONSE_LENGTH
        and frame.register_addr == Register.SAVE_PARAMS
    )


def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{value:02X}" for value in data)
