from __future__ import annotations

import select
import socket
import struct
import time
from dataclasses import dataclass
from typing import Optional


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes = b""
    is_extended_id: bool = False
    is_remote_frame: bool = False
    is_error_frame: bool = False
    timestamp: Optional[float] = None

    def __post_init__(self) -> None:
        if len(self.data) > 8:
            raise ValueError("classic CAN frames can contain at most 8 data bytes")
        if self.can_id < 0:
            raise ValueError("CAN ID must be non-negative")
        if self.is_extended_id and self.can_id > CAN_EFF_MASK:
            raise ValueError("extended CAN ID must be <= 0x1fffffff")
        if not self.is_extended_id and self.can_id > CAN_SFF_MASK:
            raise ValueError("standard CAN ID must be <= 0x7ff")

    def to_socketcan(self) -> bytes:
        raw_id = self.can_id
        if self.is_extended_id:
            raw_id |= CAN_EFF_FLAG
        if self.is_remote_frame:
            raw_id |= CAN_RTR_FLAG
        if self.is_error_frame:
            raw_id |= CAN_ERR_FLAG

        return struct.pack(CAN_FRAME_FORMAT, raw_id, len(self.data), self.data.ljust(8, b"\x00"))

    @classmethod
    def from_socketcan(cls, packet: bytes, timestamp: Optional[float] = None) -> "CanFrame":
        raw_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, packet[:CAN_FRAME_SIZE])
        is_extended_id = bool(raw_id & CAN_EFF_FLAG)
        can_id = raw_id & (CAN_EFF_MASK if is_extended_id else CAN_SFF_MASK)

        return cls(
            can_id=can_id,
            data=data[:dlc],
            is_extended_id=is_extended_id,
            is_remote_frame=bool(raw_id & CAN_RTR_FLAG),
            is_error_frame=bool(raw_id & CAN_ERR_FLAG),
            timestamp=timestamp,
        )


class SocketCanBus:
    """Small SocketCAN wrapper for classic CAN frames."""

    def __init__(self, channel: str = "can0", receive_timeout: float = 0.0) -> None:
        self.channel = channel
        self.receive_timeout = receive_timeout
        self._socket: Optional[socket.socket] = None

    def open(self) -> None:
        if self._socket is not None:
            return

        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((self.channel,))
        sock.setblocking(False)
        self._socket = sock

    def close(self) -> None:
        if self._socket is None:
            return
        self._socket.close()
        self._socket = None

    def __enter__(self) -> "SocketCanBus":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def send(self, frame: CanFrame) -> None:
        sock = self._require_socket()
        sock.send(frame.to_socketcan())

    def recv(self, timeout: Optional[float] = None) -> Optional[CanFrame]:
        sock = self._require_socket()
        wait_time = self.receive_timeout if timeout is None else timeout
        readable, _, _ = select.select([sock], [], [], wait_time)
        if not readable:
            return None

        packet = sock.recv(CAN_FRAME_SIZE)
        return CanFrame.from_socketcan(packet, timestamp=time.monotonic())

    def _require_socket(self) -> socket.socket:
        if self._socket is None:
            raise RuntimeError("CAN bus is not open. Call open() first.")
        return self._socket
