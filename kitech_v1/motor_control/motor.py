from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from .can_bus import CanFrame, SocketCanBus
from .motor_protocol import MotorProtocol, ParsedFeedback


@dataclass
class MotorState:
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
    last_update_s: Optional[float] = None


class Motor:
    def __init__(self, node_id: int, bus: SocketCanBus, protocol: MotorProtocol) -> None:
        if node_id <= 0:
            raise ValueError("node_id must be positive")
        self.node_id = node_id
        self.bus = bus
        self.protocol = protocol
        self.state = MotorState()

    def enable(self) -> None:
        self.bus.send(self.protocol.make_enable(self.node_id))

    def disable(self) -> None:
        self.bus.send(self.protocol.make_disable(self.node_id))

    def set_voltage(self, voltage_v: float) -> None:
        self.bus.send(self.protocol.make_voltage_command(self.node_id, voltage_v))

    def send_frame(self, frame: CanFrame) -> None:
        self.bus.send(frame)

    def handle_frame(self, frame: CanFrame) -> bool:
        feedback = self.protocol.parse_feedback(frame)
        if feedback is None or feedback.node_id != self.node_id:
            return False
        self._apply_feedback(feedback)
        return True

    def poll_feedback(self, timeout: float = 0.0) -> bool:
        frame = self.bus.recv(timeout=timeout)
        if frame is None:
            return False
        return self.handle_frame(frame)

    def _apply_feedback(self, feedback: ParsedFeedback) -> None:
        if feedback.position_rad is not None:
            self.state.position_rad = feedback.position_rad
        if feedback.velocity_rad_s is not None:
            self.state.velocity_rad_s = feedback.velocity_rad_s
        if feedback.current_a is not None:
            self.state.current_a = feedback.current_a
        if feedback.voltage_v is not None:
            self.state.voltage_v = feedback.voltage_v
        if feedback.raw_status is not None:
            self.state.raw_status = feedback.raw_status
        if feedback.position_raw is not None:
            self.state.position_raw = feedback.position_raw
        if feedback.velocity_raw is not None:
            self.state.velocity_raw = feedback.velocity_raw
        if feedback.torque_raw is not None:
            self.state.torque_raw = feedback.torque_raw
        if feedback.error_code is not None:
            self.state.error_code = feedback.error_code
        if feedback.mode_display is not None:
            self.state.mode_display = feedback.mode_display
        if feedback.digital_inputs is not None:
            self.state.digital_inputs = feedback.digital_inputs
        self.state.last_update_s = time.monotonic()
