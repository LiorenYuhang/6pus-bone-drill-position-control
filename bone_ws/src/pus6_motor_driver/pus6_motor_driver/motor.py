"""Single logical motor interface built on top of one shared UartBus."""

from __future__ import annotations

from typing import Iterable

try:
    from .protocol import (
        CMD_RD_REGISTER,
        CMD_RD_STATUS,
        CMD_WR_REGISTER,
        Register,
        ReadRegisterResponse,
        SaveParametersResponse,
        StatusFeedback,
        pack_read_register_cmd,
        pack_read_status_cmd,
        pack_write_register_cmd,
        parse_save_responses,
        unpack_read_register_response,
        unpack_status_response,
        unpack_write_response,
    )
    from .uart_bus import UartBus
except ImportError:
    from protocol import (
        CMD_RD_REGISTER,
        CMD_RD_STATUS,
        CMD_WR_REGISTER,
        Register,
        ReadRegisterResponse,
        SaveParametersResponse,
        StatusFeedback,
        pack_read_register_cmd,
        pack_read_status_cmd,
        pack_write_register_cmd,
        parse_save_responses,
        unpack_read_register_response,
        unpack_status_response,
        unpack_write_response,
    )
    from uart_bus import UartBus


class Motor:
    """A logical motor ID; the physical serial port belongs to ``UartBus``."""

    def __init__(self, bus: UartBus, motor_id: int = 1) -> None:
        if not 1 <= motor_id <= 0xFE:
            raise ValueError("motor_id must be in 1..254")
        self.bus = bus
        self.motor_id = motor_id

    def read_status(self) -> StatusFeedback:
        raw = self.bus.request(
            pack_read_status_cmd(self.motor_id),
            expected_id=self.motor_id,
            expected_command=CMD_RD_STATUS,
        )
        return unpack_status_response(raw)

    def read_register(self, register_addr: int, count: int = 1) -> ReadRegisterResponse:
        raw = self.bus.request(
            pack_read_register_cmd(self.motor_id, register_addr, count),
            expected_id=self.motor_id,
            expected_command=CMD_RD_REGISTER,
        )
        return unpack_read_register_response(raw, register_addr, count)

    def write_register(self, register_addr: int, data: Iterable[int]) -> StatusFeedback:
        """Write normal registers and parse the normal status response.

        SAVE_PARAMS is intentionally excluded because it returns two frames;
        callers must use ``save_parameters`` so the second frame is consumed.
        """

        if register_addr == Register.SAVE_PARAMS:
            raise ValueError("use save_parameters() for register 0x1C")
        raw = self.bus.request(
            pack_write_register_cmd(self.motor_id, register_addr, data),
            expected_id=self.motor_id,
            expected_command=CMD_WR_REGISTER,
            retries=0,
        )
        return unpack_write_response(raw)

    def set_motor_id(self, new_id: int) -> StatusFeedback:
        """Write a different ID and update the logical ID after its response.

        The response to the ID write is matched against the old ID because the
        official protocol states that the new ID takes effect immediately and
        the reference implementation receives the final response under the
        old address.
        """

        if not 1 <= new_id <= 0xFE:
            raise ValueError("new_id must be in 1..254")
        if new_id == self.motor_id:
            raise ValueError("new_id equals the current ID; no write is performed")
        old_id = self.motor_id
        raw = self.bus.request(
            pack_write_register_cmd(old_id, Register.ID_ADDR, [new_id]),
            expected_id=old_id,
            expected_command=CMD_WR_REGISTER,
            retries=0,
        )
        result = unpack_write_response(raw)
        self.motor_id = new_id
        return result

    def save_parameters(self) -> SaveParametersResponse:
        """Save parameters and consume both official response frames."""

        frame = pack_write_register_cmd(self.motor_id, Register.SAVE_PARAMS, [1])
        raw_frames = self.bus.request_frames(
            frame,
            response_count=2,
            expected_id=self.motor_id,
            retries=0,
        )
        return parse_save_responses(raw_frames, expected_id=self.motor_id)
