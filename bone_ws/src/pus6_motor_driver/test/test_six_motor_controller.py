import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

import pytest
from unittest.mock import MagicMock
from pus6_motor_driver.protocol import (
    CMD_RD_REGISTER,
    CMD_RD_STATUS,
    CMD_WR_REGISTER,
    StatusFeedback,
    pack_read_register_cmd,
    pack_read_status_cmd,
    pack_write_register_cmd,
    parse_frame,
)
from pus6_motor_driver.safety import (
    ACTUAL_POSITION_ADDR,
    CONTROL_MODE_ADDR,
    LOWER_LIMIT_ADDR,
    MOTOR_IDS,
    PAUSE_ADDR,
    SafetyViolation,
    TargetValidationError,
    TARGET_POSITION_ADDR,
    TARGET_SPEED_ADDR,
    UPPER_LIMIT_ADDR,
)
from pus6_motor_driver.six_motor_controller import SixMotorController


class FakeBus:
    def __init__(self):
        self.events = []
        self.closed = False
        self.fail_on_id = None
        self.positions = {mid: 50 for mid in MOTOR_IDS}
        self.currents = {mid: 10 for mid in MOTOR_IDS}
        self.faults = {mid: 0 for mid in MOTOR_IDS}
        self.limits = {mid: (0, 1800) for mid in MOTOR_IDS}
        self.modes = {mid: 2 for mid in MOTOR_IDS}
        self.speeds = {mid: 50 for mid in MOTOR_IDS}

    def request(self, frame, **kwargs):
        parsed = parse_frame(frame)
        mid = parsed.motor_id
        cmd = parsed.command
        reg = parsed.register_addr
        payload = [int.from_bytes(parsed.data[i:i+2], "little") for i in range(0, len(parsed.data), 2)]
        self.events.append((mid, cmd, reg, payload))

        if self.fail_on_id == mid and cmd == CMD_WR_REGISTER:
            raise OSError(f"Simulated write error on ID{mid}")

        if cmd == CMD_RD_REGISTER:
            if reg == LOWER_LIMIT_ADDR:
                val = self.limits[mid][0]
            elif reg == UPPER_LIMIT_ADDR:
                val = self.limits[mid][1]
            elif reg == CONTROL_MODE_ADDR:
                val = self.modes[mid]
            elif reg == TARGET_SPEED_ADDR:
                val = self.speeds[mid]
            elif reg == ACTUAL_POSITION_ADDR:
                val = self.positions[mid]
            else:
                val = 0
            # return 10-byte response frame: AA 55 05 mid 31 reg val(2B) checksum
            data = bytes([0xAA, 0x55, 0x05, mid, 0x31, reg & 0xFF, (reg >> 8) & 0xFF, val & 0xFF, (val >> 8) & 0xFF])
            ck = sum(data[2:]) & 0xFF
            return data + bytes([ck])

        elif cmd == CMD_RD_STATUS:
            tgt = self.positions[mid]
            cur_pos = self.positions[mid]
            cur_mA = self.currents[mid]
            err = self.faults[mid]
            data = bytearray([0xAA, 0x55, 0x0F, mid, 0x30, 0x00, 0x00,
                              tgt & 0xFF, (tgt >> 8) & 0xFF,
                              cur_pos & 0xFF, (cur_pos >> 8) & 0xFF,
                              cur_mA & 0xFF, (cur_mA >> 8) & 0xFF,
                              0x00, 0x00,
                              0x00, 0x00,
                              0x20, err])
            ck = sum(data[2:]) & 0xFF
            data.append(ck)
            return bytes(data)

        elif cmd == CMD_WR_REGISTER:
            if reg == TARGET_POSITION_ADDR and payload:
                self.positions[mid] = payload[0]
            elif reg == CONTROL_MODE_ADDR and len(payload) >= 4:
                self.modes[mid] = payload[0]
                self.speeds[mid] = payload[3]

            tgt = self.positions[mid]
            cur_pos = self.positions[mid]
            cur_mA = self.currents[mid]
            err = self.faults[mid]
            data = bytearray([0xAA, 0x55, 0x0F, mid, 0x32, reg & 0xFF, (reg >> 8) & 0xFF,
                              tgt & 0xFF, (tgt >> 8) & 0xFF,
                              cur_pos & 0xFF, (cur_pos >> 8) & 0xFF,
                              cur_mA & 0xFF, (cur_mA >> 8) & 0xFF,
                              0x00, 0x00,
                              0x00, 0x00,
                              0x20, err])
            ck = sum(data[2:]) & 0xFF
            data.append(ck)
            return bytes(data)

        return b""

    def close(self):
        self.closed = True


def test_six_motor_controller_connect_and_read():
    bus = FakeBus()
    ctrl = SixMotorController(bus=bus)
    ctrl.connect()
    assert ctrl.is_connected

    limits = ctrl.read_limits()
    assert len(limits) == 6
    assert limits[1] == (0, 1800)

    positions = ctrl.read_positions()
    assert positions == [50, 50, 50, 50, 50, 50]

    states = ctrl.read_states()
    assert len(states) == 6
    assert all(s.current_position == 50 for s in states)
    assert all(s.error_code == 0 for s in states)

    ctrl.close()
    assert not ctrl.is_connected
    assert bus.closed


def test_six_motor_controller_configure_all():
    bus = FakeBus()
    ctrl = SixMotorController(bus=bus)
    ctrl.connect()
    ctrl.configure_all(mode=2, speed=50)

    # Check setup writes on 0x25
    writes = [e for e in bus.events if e[1] == CMD_WR_REGISTER and e[2] == CONTROL_MODE_ADDR]
    assert len(writes) == 6
    for e in writes:
        # payload should be [mode, 0, 0, speed, safety_max]
        assert e[3] == [2, 0, 0, 50, 1800]


def test_six_motor_controller_command_positions_success():
    bus = FakeBus()
    ctrl = SixMotorController(bus=bus)
    ctrl.connect()

    targets = [60, 70, 80, 90, 100, 110]
    responses = ctrl.command_positions(targets)
    assert len(responses) == 6

    # Verify positions in fake bus
    assert ctrl.read_positions() == targets

    # Verify write events
    target_writes = [e for e in bus.events if e[1] == CMD_WR_REGISTER and e[2] == TARGET_POSITION_ADDR]
    assert len(target_writes) == 6
    for idx, e in enumerate(target_writes):
        assert e[0] == MOTOR_IDS[idx]
        assert e[3] == [targets[idx]]


def test_six_motor_controller_command_positions_invalid_zero_writes():
    bus = FakeBus()
    ctrl = SixMotorController(bus=bus)
    ctrl.connect()
    initial_events_count = len(bus.events)

    # 1. Invalid length (5 targets)
    with pytest.raises(TargetValidationError):
        ctrl.command_positions([10, 20, 30, 40, 50])
    assert len(bus.events) == initial_events_count

    # 2. Out of range target (-1)
    with pytest.raises(TargetValidationError):
        ctrl.command_positions([10, 20, -1, 40, 50, 60])
    assert len(bus.events) == initial_events_count


def test_six_motor_controller_write_failure_triggers_pause_all():
    bus = FakeBus()
    bus.fail_on_id = 3
    ctrl = SixMotorController(bus=bus)
    ctrl.connect()

    with pytest.raises(OSError, match="Simulated write error on ID3"):
        ctrl.command_positions([60, 70, 80, 90, 100, 110])

    # Check that pause writes (0x1A) were issued to motors
    pause_writes = [e for e in bus.events if e[1] == CMD_WR_REGISTER and e[2] == PAUSE_ADDR]
    assert len(pause_writes) == 6


def test_six_motor_controller_pause_all():
    bus = FakeBus()
    ctrl = SixMotorController(bus=bus)
    ctrl.connect()
    res = ctrl.pause_all()
    assert all(res.values())
    pause_writes = [e for e in bus.events if e[1] == CMD_WR_REGISTER and e[2] == PAUSE_ADDR]
    assert len(pause_writes) == 6


def test_six_motor_controller_snapshot_and_metrics():
    bus = FakeBus()
    ctrl = SixMotorController(bus=bus)
    ctrl.connect()

    # Test read_state_snapshot
    snapshot = ctrl.read_state_snapshot()
    assert len(snapshot.states) == 6
    assert len(snapshot.positions_steps) == 6
    assert len(snapshot.positions_meters) == 6
    assert len(snapshot.per_axis_timestamps) == 6
    assert snapshot.timestamp_end >= snapshot.timestamp_start
    assert ctrl.last_snapshot is snapshot

    # Test command_positions_with_metrics
    res = ctrl.command_positions_with_metrics([10, 20, 30, 40, 50, 60])
    assert res.commanded_targets == [10, 20, 30, 40, 50, 60]
    assert len(res.responses) == 6
    assert len(res.per_axis_write_timestamps) == 6
    assert res.timestamp_end >= res.timestamp_start


def test_six_motor_controller_read_only_mode_allows_reads_and_blocks_all_writes():
    from pus6_motor_driver.safety import HardwareMode, ReadOnlyModeError

    bus = FakeBus()
    ctrl = SixMotorController(bus=bus, hardware_mode=HardwareMode.READ_ONLY)
    ctrl.connect()

    # 1. Reading limits and states is fully allowed in READ_ONLY mode
    limits = ctrl.read_limits()
    assert len(limits) == 6
    snapshot = ctrl.read_state_snapshot()
    assert len(snapshot.states) == 6
    assert ctrl.write_count == 0

    # 2. configure_all is strictly rejected (0 writes)
    with pytest.raises(ReadOnlyModeError, match="strictly forbidden in READ_ONLY mode"):
        ctrl.configure_all()
    assert ctrl.write_count == 0

    # 3. command_positions is strictly rejected (0 writes)
    with pytest.raises(ReadOnlyModeError, match="strictly forbidden in READ_ONLY mode"):
        ctrl.command_positions([10, 20, 30, 40, 50, 60])
    assert ctrl.write_count == 0

    # 4. command_single_axis_for_bringup is strictly rejected (0 writes)
    with pytest.raises(ReadOnlyModeError, match="forbidden in READ_ONLY mode"):
        ctrl.command_single_axis_for_bringup(motor_id=1, target_step=20)
    assert ctrl.write_count == 0

    # 5. pause_all is strictly rejected (0 writes)
    with pytest.raises(ReadOnlyModeError, match="forbidden in READ_ONLY mode"):
        ctrl.pause_all()
    assert ctrl.write_count == 0

    # 6. close / shutdown has 0 writes
    ctrl.close()
    assert ctrl.write_count == 0
    writes = [e for e in bus.events if e[1] == CMD_WR_REGISTER]
    assert len(writes) == 0


def test_six_motor_controller_single_axis_bringup_writes_only_target_motor():
    from pus6_motor_driver.safety import HardwareMode

    bus = FakeBus()
    ctrl = SixMotorController(bus=bus, hardware_mode=HardwareMode.ACTIVE)
    ctrl.connect()
    assert ctrl.write_count == 0

    # Move ID2 to step 75. ID1, ID3, ID4, ID5, ID6 must receive ZERO writes!
    res = ctrl.command_single_axis_for_bringup(motor_id=2, target_step=75)
    assert res is not None
    assert ctrl.write_count == 1

    # Verify write events in fake bus
    writes = [e for e in bus.events if e[1] == CMD_WR_REGISTER]
    assert len(writes) == 1
    assert writes[0][0] == 2  # motor ID2
    assert writes[0][2] == TARGET_POSITION_ADDR
    assert writes[0][3] == [75]

    # Verify observers (ID1, 3, 4, 5, 6) received 0 writes
    observer_writes = [e for e in bus.events if e[1] == CMD_WR_REGISTER and e[0] != 2]
    assert len(observer_writes) == 0


def test_six_motor_controller_mock_mode_zero_real_serial():
    from pus6_motor_driver.safety import HardwareMode

    # Initialize controller in MOCK mode without passing bus
    ctrl = SixMotorController(hardware_mode=HardwareMode.MOCK)
    ctrl.connect()
    assert ctrl.is_connected
    assert ctrl.hardware_mode == HardwareMode.MOCK
    ctrl.close()
