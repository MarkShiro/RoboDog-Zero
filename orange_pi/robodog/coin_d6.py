"""COIN-D6 / CSPC TOF packet reader, independently implemented from SDK format.

Protocol reference: CSPC ROS2 SDK `waitPackage` for M1CT_TOF, distributed in
https://github.com/sachisti/OnStage26/tree/main/support/original_coind6_ros_module
Hardware capture is still required to confirm this kit's firmware variant.
"""
import struct
from typing import Iterable, List, Tuple

HEADER = b"\xAA\x55"
START_COMMAND = b"\xAA\x55\xF0\x0F"


class CoinD6Parser:
    """Bounded incremental reader for 10-byte headers and N three-byte points."""

    def __init__(self):
        self.buffer = bytearray()
        self.good_packets = 0
        self.bad_packets = 0

    def feed(self, data: bytes) -> Iterable[Tuple[bool, List[Tuple[float, float]]]]:
        self.buffer.extend(data)
        if len(self.buffer) > 4096:
            del self.buffer[:-4096]
        while len(self.buffer) >= 10:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = self.buffer[-1:]
                return
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 10:
                return
            count = self.buffer[3]
            # The supplied SDK's TOF packet size is 10 + 3*N; 40 samples
            # produce its stated 130-byte intensity packet.
            if count < 1 or count > 64 or not (self.buffer[4] & 1) or not (self.buffer[6] & 1):
                self.bad_packets += 1
                del self.buffer[0]
                continue
            size = 10 + 3 * count
            if len(self.buffer) < size:
                return
            frame = bytes(self.buffer[:size])
            start_word, stop_word, given = struct.unpack_from("<HHH", frame, 4)
            value = 0x55AA ^ start_word ^ stop_word ^ (frame[2] | count << 8)
            for index in range(10, size, 3):
                value ^= frame[index]
                value ^= frame[index + 1] | frame[index + 2] << 8
            if value != given:
                self.bad_packets += 1
                del self.buffer[0]
                continue
            del self.buffer[:size]
            self.good_packets += 1
            first = (start_word >> 1) / 64.0
            last = (stop_word >> 1) / 64.0
            if not (0 <= first < 360 and 0 <= last < 360):
                self.bad_packets += 1
                continue
            span = (last - first) % 360.0
            if span > 60.0:
                self.bad_packets += 1
                continue
            points = []
            for number in range(count):
                at = 10 + 3 * number
                millimeters = frame[at + 2] * 64 + (frame[at + 1] >> 2)
                if 50 <= millimeters <= 12000:
                    angle = (first + (span * number / (count - 1) if count > 1 else 0)) % 360.0
                    points.append((angle, millimeters / 1000.0))
            yield bool(frame[2] & 1), points
