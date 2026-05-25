from __future__ import annotations

from dataclasses import dataclass

from .can_bus import CanFrame
from .motor_protocol import ParsedFeedback


@dataclass(frozen=True)
class WelconKi2aHardwareSpec:
    axes_per_drive: int = 1
    input_voltage_v: float = 12.0
    continuous_current_a_rms: float = 0.1
    peak_current_a_rms: float = 0.2
    current_loop_hz: int = 20_000
    velocity_position_loop_hz: int = 4_000
    can_classic_min_bitrate: int = 125_000
    can_classic_max_bitrate: int = 1_000_000
    can_fd_data_bitrate: int = 5_000_000


class WelconKi2aProtocol:
    """WELCON KI2A protocol skeleton.

    The hardware manual defines electrical limits and CAN wiring, but it does
    not define command COB-IDs, object indexes, byte layout, or scale factors.
    Fill this class from the communication/protocol manual before use.
    """

    hardware = WelconKi2aHardwareSpec()

    def make_enable(self, node_id: int) -> CanFrame:
        raise NotImplementedError("need WELCON KI2A enable command frame")

    def make_disable(self, node_id: int) -> CanFrame:
        raise NotImplementedError("need WELCON KI2A disable command frame")

    def make_voltage_command(self, node_id: int, voltage_v: float) -> CanFrame:
        raise NotImplementedError("need WELCON KI2A voltage command frame and scale factor")

    def parse_feedback(self, frame: CanFrame) -> ParsedFeedback | None:
        raise NotImplementedError("need WELCON KI2A feedback frame layout and scale factors")
