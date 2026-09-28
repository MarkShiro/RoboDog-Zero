"""Wire format shared by the Pro Mini, Mega and Orange Pi.

CRC checks accidental corruption. It is not authentication.
"""
from dataclasses import dataclass
from typing import Optional


def crc8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x07 if value & 0x80 else 0)) & 0xFF
    return value


def encode(body: str) -> bytes:
    payload = body.encode("ascii")
    if not payload or len(payload) > 48:
        raise ValueError("frame body size")
    return payload + b"*%02X\n" % crc8(payload)


def decode(line: bytes) -> str:
    if not line.endswith(b"\n") or len(line) > 56:
        raise ValueError("line length or ending")
    line = line.rstrip(b"\r\n")
    if line.count(b"*") != 1:
        raise ValueError("star")
    body, checksum = line.split(b"*")
    if not body or len(checksum) != 2:
        raise ValueError("body/checksum")
    if any(c not in b"0123456789abcdefABCDEF" for c in checksum):
        raise ValueError("checksum digits")
    try:
        code = int(checksum, 16)
        text = body.decode("ascii")
    except (ValueError, UnicodeError):
        raise ValueError("non-ascii or checksum")
    if any(c < 32 or c > 126 for c in body) or code != crc8(body):
        raise ValueError("invalid bytes or CRC")
    return text


def decimal(token: str, maximum: int, signed: bool = False) -> int:
    sign = token.startswith("-") and signed
    digits = token[1:] if sign else token
    if not digits or not digits.isascii() or not digits.isdecimal():
        raise ValueError("decimal field")
    number = int(token)
    if number < (-maximum if signed else 0) or number > maximum:
        raise ValueError("decimal range")
    return number


def hexadecimal(token: str, maximum: int) -> int:
    if not token or len(token) > 4 or any(c not in "0123456789abcdefABCDEF" for c in token):
        raise ValueError("hex field")
    number = int(token, 16)
    if number > maximum:
        raise ValueError("hex range")
    return number


@dataclass(frozen=True)
class Control:
    seq: int
    buttons: int
    remote_mv: int
    speed: int
    mode: int


@dataclass(frozen=True)
class Telemetry:
    seq: int
    state: int
    battery_mv: int
    flags: int


@dataclass(frozen=True)
class Imu:
    roll: int
    pitch: int
    temperature: int
    flags: int


def control(body: str) -> Control:
    p = body.split(",")
    if len(p) != 6 or p[0] != "C":
        raise ValueError("C fields")
    return Control(decimal(p[1], 65535), hexadecimal(p[2], 1023),
                   decimal(p[3], 65535), decimal(p[4], 5), decimal(p[5], 255))


def telemetry(body: str) -> Telemetry:
    p = body.split(",")
    if len(p) != 5 or p[0] != "T":
        raise ValueError("T fields")
    return Telemetry(decimal(p[1], 65535), decimal(p[2], 5),
                     decimal(p[3], 65535), hexadecimal(p[4], 65535))


def imu(body: str) -> Imu:
    p = body.split(",")
    if len(p) != 5 or p[0] != "I":
        raise ValueError("I fields")
    return Imu(decimal(p[1], 180, True), decimal(p[2], 180, True),
               decimal(p[3], 150, True), hexadecimal(p[4], 65535))


def sequence_newer(seq: int, previous: int) -> bool:
    delta=(seq-previous)&0xFFFF
    return 0 < delta < 0x8000


class Framer:
    """Small bounded LF framer; discard overflow until the next LF."""
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.overflow = False

    def feed(self, data: bytes):
        for byte in data:
            if byte == 10:
                if self.buffer and not self.overflow:
                    yield bytes(self.buffer) + b"\n"
                self.buffer.clear()
                self.overflow = False
            elif not self.overflow:
                if len(self.buffer) >= 55:
                    self.buffer.clear()
                    self.overflow = True
                else:
                    self.buffer.append(byte)
