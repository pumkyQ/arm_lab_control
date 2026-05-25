from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

from .can_bus import CanFrame
from .motor_protocol import ParsedFeedback


class Cia402Mode(IntEnum):
    PROFILE_POSITION = 1
    PROFILE_VELOCITY = 3
    PROFILE_TORQUE = 4
    HOMING = 6
    CYCLIC_SYNC_POSITION = 8
    CYCLIC_SYNC_VELOCITY = 9
    CYCLIC_SYNC_TORQUE = 10


SUPPORTED_MODE_BITS = {
    Cia402Mode.PROFILE_POSITION: 0,
    Cia402Mode.PROFILE_VELOCITY: 2,
    Cia402Mode.PROFILE_TORQUE: 3,
    Cia402Mode.HOMING: 5,
    Cia402Mode.CYCLIC_SYNC_POSITION: 8,
    Cia402Mode.CYCLIC_SYNC_VELOCITY: 9,
    Cia402Mode.CYCLIC_SYNC_TORQUE: 10,
}


class Cia402Controlword(IntEnum):
    SHUTDOWN = 0x0006
    SWITCH_ON = 0x0007
    ENABLE_OPERATION = 0x000F
    DISABLE_OPERATION = 0x0007
    QUICK_STOP = 0x0002
    FAULT_RESET = 0x0080


@dataclass(frozen=True)
class Cia402Object:
    index: int
    subindex: int = 0


@dataclass(frozen=True)
class SdoResponse:
    node_id: int
    index: int
    subindex: int
    value: Optional[int] = None
    size: Optional[int] = None
    abort_code: Optional[int] = None


CONTROLWORD = Cia402Object(0x6040)
STATUSWORD = Cia402Object(0x6041)
MODES_OF_OPERATION = Cia402Object(0x6060)
MODES_OF_OPERATION_DISPLAY = Cia402Object(0x6061)
CONTROLWORD_AXIS2 = Cia402Object(0x6840)
STATUSWORD_AXIS2 = Cia402Object(0x6841)
MODES_OF_OPERATION_AXIS2 = Cia402Object(0x6860)
MODES_OF_OPERATION_DISPLAY_AXIS2 = Cia402Object(0x6861)
POSITION_ACTUAL = Cia402Object(0x6064)
VELOCITY_ACTUAL = Cia402Object(0x606C)
TARGET_TORQUE = Cia402Object(0x6071)
TORQUE_ACTUAL = Cia402Object(0x6077)
CURRENT_ACTUAL = Cia402Object(0x6078)
TARGET_POSITION = Cia402Object(0x607A)
TARGET_VELOCITY = Cia402Object(0x60FF)
SUPPORTED_DRIVE_MODES = Cia402Object(0x6502)
IDENTITY_VENDOR_ID = Cia402Object(0x1018, 1)
IDENTITY_PRODUCT_CODE = Cia402Object(0x1018, 2)
IDENTITY_REVISION = Cia402Object(0x1018, 3)
IDENTITY_SERIAL = Cia402Object(0x1018, 4)
Q_AXIS_VOLTAGE_AXIS1 = Cia402Object(0x2103)
Q_AXIS_VOLTAGE_AXIS2 = Cia402Object(0x2903)
TARGET_ANALOG_INPUT_VOLTAGE1_AXIS1 = Cia402Object(0x20FE, 5)
TARGET_ANALOG_INPUT_VOLTAGE2_AXIS1 = Cia402Object(0x20FE, 6)
TARGET_ANALOG_INPUT_VOLTAGE1_AXIS2 = Cia402Object(0x28FE, 5)
TARGET_ANALOG_INPUT_VOLTAGE2_AXIS2 = Cia402Object(0x28FE, 6)
ACTUAL_CURRENT_AXIS1 = Cia402Object(0x2181)
Q_AXIS_DEMAND_CURRENT_AXIS1 = Cia402Object(0x2184)
Q_AXIS_ACTUAL_CURRENT_AXIS1 = Cia402Object(0x2186)
ACTUAL_CURRENT_AXIS2 = Cia402Object(0x2981)
Q_AXIS_DEMAND_CURRENT_AXIS2 = Cia402Object(0x2984)
Q_AXIS_ACTUAL_CURRENT_AXIS2 = Cia402Object(0x2986)


class Cia402Protocol:
    """CiA402 helpers using the default WELCON PDO mapping from the EDS files."""

    def make_enable(self, node_id: int) -> CanFrame:
        return self.make_rpdo1(
            node_id=node_id,
            controlword=Cia402Controlword.ENABLE_OPERATION,
            mode=Cia402Mode.PROFILE_TORQUE,
            target_position=0,
        )

    def make_disable(self, node_id: int) -> CanFrame:
        return self.make_rpdo1(
            node_id=node_id,
            controlword=Cia402Controlword.DISABLE_OPERATION,
            mode=Cia402Mode.PROFILE_TORQUE,
            target_position=0,
        )

    def make_voltage_command(self, node_id: int, voltage_v: float) -> CanFrame:
        return self.make_target_analog_input_voltage_mv_sdo(
            node_id=node_id,
            voltage_mv=round(voltage_v * 1000.0),
            axis=1,
        )

    def make_q_axis_voltage_raw_sdo(self, node_id: int, voltage_raw: int, axis: int = 1) -> CanFrame:
        obj = self._q_axis_voltage_object(axis)
        return self.make_sdo_write_i32(node_id, obj, voltage_raw)

    def make_q_axis_voltage_mv_sdo(self, node_id: int, voltage_mv: int, axis: int = 1) -> CanFrame:
        return self.make_q_axis_voltage_raw_sdo(node_id=node_id, voltage_raw=voltage_mv, axis=axis)

    def make_q_axis_voltage_read_sdo(self, node_id: int, axis: int = 1) -> CanFrame:
        return self.make_sdo_read(node_id, self._q_axis_voltage_object(axis))

    def make_target_analog_input_voltage_mv_sdo(
        self,
        node_id: int,
        voltage_mv: int,
        axis: int = 1,
        channel: int = 1,
    ) -> CanFrame:
        obj = self._target_analog_input_voltage_object(axis=axis, channel=channel)
        return self.make_sdo_write_i16(node_id, obj, voltage_mv)

    def make_target_analog_input_voltage_read_sdo(
        self,
        node_id: int,
        axis: int = 1,
        channel: int = 1,
    ) -> CanFrame:
        return self.make_sdo_read(node_id, self._target_analog_input_voltage_object(axis=axis, channel=channel))

    def make_actual_current_read_sdo(self, node_id: int, axis: int) -> CanFrame:
        return self.make_sdo_read(node_id, ACTUAL_CURRENT_AXIS1 if axis == 1 else ACTUAL_CURRENT_AXIS2)

    def make_q_axis_demand_current_read_sdo(self, node_id: int, axis: int) -> CanFrame:
        return self.make_sdo_read(node_id, Q_AXIS_DEMAND_CURRENT_AXIS1 if axis == 1 else Q_AXIS_DEMAND_CURRENT_AXIS2)

    def make_q_axis_actual_current_read_sdo(self, node_id: int, axis: int) -> CanFrame:
        return self.make_sdo_read(node_id, Q_AXIS_ACTUAL_CURRENT_AXIS1 if axis == 1 else Q_AXIS_ACTUAL_CURRENT_AXIS2)

    def make_nmt_start(self, node_id: int = 0) -> CanFrame:
        return CanFrame(0x000, bytes([0x01, node_id & 0x7F]))

    def make_nmt_pre_operational(self, node_id: int = 0) -> CanFrame:
        return CanFrame(0x000, bytes([0x80, node_id & 0x7F]))

    def make_nmt_reset_communication(self, node_id: int = 0) -> CanFrame:
        return CanFrame(0x000, bytes([0x82, node_id & 0x7F]))

    def make_sdo_read(self, node_id: int, obj: Cia402Object) -> CanFrame:
        data = bytes([0x40]) + obj.index.to_bytes(2, "little") + bytes([obj.subindex, 0, 0, 0, 0])
        return CanFrame(0x600 + node_id, data)

    def parse_sdo_response(self, frame: CanFrame) -> Optional[SdoResponse]:
        if not 0x580 <= frame.can_id <= 0x5FF or len(frame.data) < 4:
            return None

        node_id = frame.can_id - 0x580
        command = frame.data[0]
        index = int.from_bytes(frame.data[1:3], "little", signed=False)
        subindex = frame.data[3]

        if command == 0x80:
            if len(frame.data) < 8:
                return None
            abort_code = int.from_bytes(frame.data[4:8], "little", signed=False)
            return SdoResponse(node_id=node_id, index=index, subindex=subindex, abort_code=abort_code)

        size_by_command = {
            0x4F: 1,
            0x4B: 2,
            0x47: 3,
            0x43: 4,
            0x60: 0,
        }
        if command not in size_by_command:
            return None

        size = size_by_command[command]
        value = None if size == 0 else int.from_bytes(frame.data[4 : 4 + size], "little", signed=False)
        return SdoResponse(node_id=node_id, index=index, subindex=subindex, value=value, size=size)

    def make_sdo_write_i8(self, node_id: int, obj: Cia402Object, value: int) -> CanFrame:
        return self._make_sdo_write(node_id, obj, value, size=1, signed=True)

    def make_sdo_write_u16(self, node_id: int, obj: Cia402Object, value: int) -> CanFrame:
        return self._make_sdo_write(node_id, obj, value, size=2, signed=False)

    def make_sdo_write_i16(self, node_id: int, obj: Cia402Object, value: int) -> CanFrame:
        return self._make_sdo_write(node_id, obj, value, size=2, signed=True)

    def make_sdo_write_i32(self, node_id: int, obj: Cia402Object, value: int) -> CanFrame:
        return self._make_sdo_write(node_id, obj, value, size=4, signed=True)

    def make_rpdo1(
        self,
        node_id: int,
        controlword: int,
        mode: int,
        target_position: int,
    ) -> CanFrame:
        data = (
            int(controlword).to_bytes(2, "little", signed=False)
            + int(mode).to_bytes(1, "little", signed=True)
            + int(target_position).to_bytes(4, "little", signed=True)
        )
        return CanFrame(0x200 + node_id, data)

    def make_rpdo2(self, node_id: int, target_velocity: int, profile_velocity: int = 0) -> CanFrame:
        data = int(target_velocity).to_bytes(4, "little", signed=True)
        data += int(profile_velocity).to_bytes(4, "little", signed=True)
        return CanFrame(0x300 + node_id, data)

    def make_target_torque_sdo(self, node_id: int, target_torque: int) -> CanFrame:
        return self.make_sdo_write_i16(node_id, TARGET_TORQUE, target_torque)

    def make_mode_sdo(self, node_id: int, mode: Cia402Mode) -> CanFrame:
        return self.make_sdo_write_i8(node_id, MODES_OF_OPERATION, int(mode))

    def make_controlword_sdo(self, node_id: int, controlword: Cia402Controlword | int) -> CanFrame:
        return self.make_sdo_write_u16(node_id, CONTROLWORD, int(controlword))

    def make_axis_mode_sdo(self, node_id: int, axis: int, mode: int) -> CanFrame:
        return self.make_sdo_write_i8(node_id, self._mode_object(axis), mode)

    def make_axis_mode_read_sdo(self, node_id: int, axis: int) -> CanFrame:
        return self.make_sdo_read(node_id, self._mode_object(axis))

    def make_axis_controlword_sdo(self, node_id: int, axis: int, controlword: Cia402Controlword | int) -> CanFrame:
        return self.make_sdo_write_u16(node_id, self._controlword_object(axis), int(controlword))

    def make_axis_statusword_read_sdo(self, node_id: int, axis: int) -> CanFrame:
        return self.make_sdo_read(node_id, self._statusword_object(axis))

    def decode_supported_modes(self, supported_modes_raw: int) -> set[Cia402Mode]:
        return {
            mode
            for mode, bit in SUPPORTED_MODE_BITS.items()
            if supported_modes_raw & (1 << bit)
        }

    def is_mode_supported(self, supported_modes_raw: int, mode: Cia402Mode) -> bool:
        bit = SUPPORTED_MODE_BITS[mode]
        return bool(supported_modes_raw & (1 << bit))

    def _q_axis_voltage_object(self, axis: int) -> Cia402Object:
        if axis == 1:
            return Q_AXIS_VOLTAGE_AXIS1
        if axis == 2:
            return Q_AXIS_VOLTAGE_AXIS2
        raise ValueError("axis must be 1 or 2 for the WELCON WE2x EDS")

    def _controlword_object(self, axis: int) -> Cia402Object:
        if axis == 1:
            return CONTROLWORD
        if axis == 2:
            return CONTROLWORD_AXIS2
        raise ValueError("axis must be 1 or 2 for the WELCON WE2x EDS")

    def _statusword_object(self, axis: int) -> Cia402Object:
        if axis == 1:
            return STATUSWORD
        if axis == 2:
            return STATUSWORD_AXIS2
        raise ValueError("axis must be 1 or 2 for the WELCON WE2x EDS")

    def _mode_object(self, axis: int) -> Cia402Object:
        if axis == 1:
            return MODES_OF_OPERATION
        if axis == 2:
            return MODES_OF_OPERATION_AXIS2
        raise ValueError("axis must be 1 or 2 for the WELCON WE2x EDS")

    def _target_analog_input_voltage_object(self, axis: int, channel: int) -> Cia402Object:
        if axis == 1 and channel == 1:
            return TARGET_ANALOG_INPUT_VOLTAGE1_AXIS1
        if axis == 1 and channel == 2:
            return TARGET_ANALOG_INPUT_VOLTAGE2_AXIS1
        if axis == 2 and channel == 1:
            return TARGET_ANALOG_INPUT_VOLTAGE1_AXIS2
        if axis == 2 and channel == 2:
            return TARGET_ANALOG_INPUT_VOLTAGE2_AXIS2
        raise ValueError("axis and channel must be 1 or 2 for the WELCON WE2x EDS")

    def parse_feedback(self, frame: CanFrame) -> Optional[ParsedFeedback]:
        node_id = frame.can_id & 0x7F

        if 0x180 <= frame.can_id <= 0x1FF:
            return self._parse_tpdo1(node_id, frame.data)
        if 0x280 <= frame.can_id <= 0x2FF:
            return self._parse_tpdo2(node_id, frame.data)
        if 0x380 <= frame.can_id <= 0x3FF:
            return self._parse_tpdo3(node_id, frame.data)

        return None

    def _parse_tpdo1(self, node_id: int, data: bytes) -> Optional[ParsedFeedback]:
        if len(data) < 7:
            return None
        statusword = int.from_bytes(data[0:2], "little", signed=False)
        mode_display = int.from_bytes(data[2:3], "little", signed=True)
        digital_inputs = int.from_bytes(data[3:7], "little", signed=False)
        return ParsedFeedback(
            node_id=node_id,
            raw_status=statusword,
            mode_display=mode_display,
            digital_inputs=digital_inputs,
        )

    def _parse_tpdo2(self, node_id: int, data: bytes) -> Optional[ParsedFeedback]:
        if len(data) < 8:
            return None
        velocity = int.from_bytes(data[0:4], "little", signed=True)
        position = int.from_bytes(data[4:8], "little", signed=True)
        return ParsedFeedback(node_id=node_id, position_raw=position, velocity_raw=velocity)

    def _parse_tpdo3(self, node_id: int, data: bytes) -> Optional[ParsedFeedback]:
        if len(data) < 4:
            return None
        torque = int.from_bytes(data[0:2], "little", signed=True)
        error_code = int.from_bytes(data[2:4], "little", signed=False)
        return ParsedFeedback(node_id=node_id, torque_raw=torque, error_code=error_code)

    def _make_sdo_write(
        self,
        node_id: int,
        obj: Cia402Object,
        value: int,
        size: int,
        signed: bool,
    ) -> CanFrame:
        command_by_size = {1: 0x2F, 2: 0x2B, 4: 0x23}
        if size not in command_by_size:
            raise ValueError("expedited SDO write size must be 1, 2, or 4 bytes")
        payload = int(value).to_bytes(size, "little", signed=signed).ljust(4, b"\x00")
        data = bytes([command_by_size[size]]) + obj.index.to_bytes(2, "little")
        data += bytes([obj.subindex]) + payload
        return CanFrame(0x600 + node_id, data)
