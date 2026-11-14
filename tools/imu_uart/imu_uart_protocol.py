"""Parser for the 1 kHz IMU UART stream of the Crazyflie firmware.

The frame layout is defined in src/hal/interface/imu_uart_proto.h. This module
has no dependencies outside the standard library.
"""

import struct
from dataclasses import dataclass
from typing import List, Optional

SYNC = b"\xa5\x5a"
FRAME_VERSION = 1
FLAG_GYRO_CALIBRATED = 1 << 0

# sync, version, flags, seq, time_us, acc xyz, gyro xyz, crc
_FRAME = struct.Struct("<2sBBHQ6fH")
FRAME_SIZE = _FRAME.size
_CRC_START = 2
_CRC_END = FRAME_SIZE - 2

assert FRAME_SIZE == 40


def _build_crc_table() -> List[int]:
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        table.append(crc & 0xFFFF)
    return table


_CRC_TABLE = _build_crc_table()


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[(crc >> 8) ^ byte]
    return crc


@dataclass
class ImuFrame:
    seq: int
    time_us: int  # microseconds since the Unix epoch, lidar time base
    acc_g: tuple  # x, y, z in G, not low pass filtered
    gyro_dps: tuple  # x, y, z in deg/s, not low pass filtered
    flags: int

    @property
    def gyro_calibrated(self) -> bool:
        return bool(self.flags & FLAG_GYRO_CALIBRATED)


class FrameParser:
    """Turns a byte stream into frames, resynchronizing after corruption.

    Counters:
        frames        valid frames returned so far
        lost_frames   frames missing according to gaps in the sequence number
        crc_errors    candidate frames rejected because of a bad CRC or version
        skipped_bytes bytes discarded while searching for a frame start
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._last_seq: Optional[int] = None
        self.frames = 0
        self.lost_frames = 0
        self.crc_errors = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> List[ImuFrame]:
        self._buffer += data
        buffer = self._buffer
        frames = []
        position = 0

        while True:
            start = buffer.find(SYNC, position)
            if start < 0:
                # Keep a trailing byte that may be the first half of a sync word
                keep = 1 if buffer.endswith(SYNC[:1]) else 0
                self.skipped_bytes += len(buffer) - position - keep
                position = len(buffer) - keep
                break

            self.skipped_bytes += start - position
            position = start
            if len(buffer) - start < FRAME_SIZE:
                break

            frame = self._decode(buffer, start)
            if frame is None:
                # Not a frame, the sync word was payload data or the frame is
                # corrupt. Search again from the next byte.
                self.crc_errors += 1
                self.skipped_bytes += 1
                position = start + 1
                continue

            frames.append(frame)
            position = start + FRAME_SIZE

        del buffer[:position]
        return frames

    def _decode(self, buffer: bytearray, start: int) -> Optional[ImuFrame]:
        raw = bytes(buffer[start:start + FRAME_SIZE])
        _, version, flags, seq, time_us, ax, ay, az, gx, gy, gz, crc = _FRAME.unpack(raw)
        if version != FRAME_VERSION or crc != crc16(raw[_CRC_START:_CRC_END]):
            return None

        if self._last_seq is not None:
            self.lost_frames += (seq - self._last_seq - 1) & 0xFFFF
        self._last_seq = seq
        self.frames += 1

        return ImuFrame(seq, time_us, (ax, ay, az), (gx, gy, gz), flags)
