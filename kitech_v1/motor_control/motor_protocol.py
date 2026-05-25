from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol

from .can_bus import CanFrame


@dataclass(frozen=True)
class ParsedFeedback:
    node_id: int
    position_rad: Optional[float] = None
    velocity_rad_s: Optional[float] = None
    current_a: Optional[float] = None
    voltage_v: Optional[float] = None
    raw_status: Optional[int] = None
    position_raw: Optional[int] = None
    velocity_raw: Optional[int] = None
    torque_raw: Optional[int] = None
    error_code: Optional[int] = None
    mode_display: Optional[int] = None
    digital_inputs: Optional[int] = None


class MotorProtocol(Protocol):
    """Driver-specific conversion between physical commands and CAN frames."""

    def make_enable(self, node_id: int) -> CanFrame:
        ...

    def make_disable(self, node_id: int) -> CanFrame:
        ...

    def make_voltage_command(self, node_id: int, voltage_v: float) -> CanFrame:
        ...

    def parse_feedback(self, frame: CanFrame) -> Optional[ParsedFeedback]:
        ...


class PlaceholderVoltageProtocol:
    """Template protocol.

    Replace the CAN IDs, byte order, and scale factors with values from the
    motor driver manual before commanding hardware.
    """

    def __init__(
        self,
        command_base_id: int,
        feedback_base_id: int,
        voltage_scale_v_per_lsb: float,
        max_voltage_v: float,
    ) -> None:
        self.command_base_id = command_base_id
        self.feedback_base_id = feedback_base_id
        self.voltage_scale_v_per_lsb = voltage_scale_v_per_lsb
        self.max_voltage_v = max_voltage_v

    def make_enable(self, node_id: int) -> CanFrame:
        raise NotImplementedError("fill in the driver-specific enable frame")

    def make_disable(self, node_id: int) -> CanFrame:
        raise NotImplementedError("fill in the driver-specific disable frame")

    def make_voltage_command(self, node_id: int, voltage_v: float) -> CanFrame:
        limited = max(-self.max_voltage_v, min(self.max_voltage_v, voltage_v))
        raw = int(round(limited / self.voltage_scale_v_per_lsb))
        data = raw.to_bytes(2, byteorder="little", signed=True)
        return CanFrame(can_id=self.command_base_id + node_id, data=data)

    def parse_feedback(self, frame: CanFrame) -> Optional[ParsedFeedback]:
        node_id = frame.can_id - self.feedback_base_id
        if node_id <= 0:
            return None
        if len(frame.data) < 8:
            return None

        position_raw = int.from_bytes(frame.data[0:2], byteorder="little", signed=True)
        velocity_raw = int.from_bytes(frame.data[2:4], byteorder="little", signed=True)
        current_raw = int.from_bytes(frame.data[4:6], byteorder="little", signed=True)
        voltage_raw = int.from_bytes(frame.data[6:8], byteorder="little", signed=True)

        return ParsedFeedback(
            node_id=node_id,
            position_rad=float(position_raw),
            velocity_rad_s=float(velocity_raw),
            current_a=float(current_raw),
            voltage_v=voltage_raw * self.voltage_scale_v_per_lsb,
        )
