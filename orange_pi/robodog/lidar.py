"""LiDAR adapter boundary. Vendor packet format is intentionally not guessed.

The COIN-D6 USB-UART kit can be captured at 230400 baud. Once its vendor SDK
is available, an adapter writes newline-delimited JSON scans atomically:
{"points": [[angle_degrees, distance_meters], ...]}.
"""
import json
import math
import threading
import time
from pathlib import Path
from typing import List, Tuple
from .state import Store
from .coin_d6 import CoinD6Parser, START_COMMAND


def validate_scan(record) -> List[Tuple[float,float]]:
    if not isinstance(record,dict) or not isinstance(record.get("points"),list):
        raise ValueError("expected points array")
    points=[]
    if len(record["points"])>5000:
        raise ValueError("scan too large")
    for pair in record["points"]:
        if not isinstance(pair,list) or len(pair)!=2:
            raise ValueError("point needs angle and distance")
        angle,distance=pair
        if (not isinstance(angle,(int,float)) or isinstance(angle,bool) or
            not isinstance(distance,(int,float)) or isinstance(distance,bool) or
            not math.isfinite(angle) or not math.isfinite(distance)):
            raise ValueError("non-finite point")
        if 0<=angle<360 and .05<=distance<=12:
            points.append((float(angle),float(distance)))
    return points


def follow_jsonl(path: str,store: Store,stop: threading.Event) -> None:
    file=Path(path)
    position=0
    inode=None
    while not stop.is_set():
        try:
            stat=file.stat()
            if inode!=stat.st_ino or stat.st_size<position:
                position=0; inode=stat.st_ino
            with file.open("r",encoding="utf-8") as stream:
                stream.seek(position)
                while True:
                    line=stream.readline()
                    if not line: break
                    if not line.endswith("\n"):
                        break # wait for an entire scan
                    position=stream.tell()
                    if len(line)>150000:
                        raise ValueError("scan line too large")
                    points=validate_scan(json.loads(line))
                    if points:
                        store.update(lidar=points,lidar_at=time.monotonic(),
                                     lidar_label="COIN-D6: {} точек".format(len(points)),
                                     lidar_status="Данные LiDAR поступают")
        except (OSError,ValueError,json.JSONDecodeError) as error:
            store.update(lidar_status="LiDAR: нет скана",last_error=str(error))
        stop.wait(.1)


def capture_raw(port: str,baud: int,out: str,seconds: int = 5,
                send_start: bool = False) -> int:
    import serial
    if not 1<=seconds<=30:
        raise ValueError("capture duration must be 1..30 seconds")
    count=0
    with serial.Serial(port,baudrate=baud,timeout=.1) as device, open(out,"wb") as target:
        if send_start:
            time.sleep(1.0)
            device.write(START_COMMAND)
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            data=device.read(4096)
            if data:
                target.write(data)
                count+=len(data)
            if count>2_000_000:
                break
    return count


def follow_coin_d6(port: str, baud: int, store: Store,
                   stop: threading.Event, send_start: bool = True) -> None:
    """Read USB-UART scans using the TOF frame format in the CSPC ROS2 SDK."""
    import serial
    parser = CoinD6Parser()
    while not stop.is_set():
        try:
            with serial.Serial(port, baudrate=baud, timeout=.1, write_timeout=.2) as device:
                store.update(lidar_status="Coin D6: USB порт открыт")
                if send_start and not stop.wait(1.0):
                    device.write(START_COMMAND)
                points = []
                last_packet = time.monotonic()
                while not stop.is_set():
                    data = device.read(512)
                    for is_start, packet in parser.feed(data):
                        now = time.monotonic()
                        wrapped = bool(points and packet and points[-1][0] > 340 and packet[0][0] < 20)
                        if (is_start or wrapped) and len(points) >= 40:
                            store.update(lidar=points[-800:], lidar_at=now,
                                         lidar_label="COIN-D6: {} точек".format(len(points)),
                                         lidar_status="Coin D6: {} пакетов, {} ошибок".format(
                                             parser.good_packets, parser.bad_packets))
                            points = []
                        points.extend(packet)
                        if len(points) > 1200:
                            points = points[-800:]
                        last_packet = now
                    if time.monotonic() - last_packet > 2.0:
                        store.update(lidar_status="Coin D6: нет корректных пакетов",
                                     last_error="Проверьте USB, 230400, питание и версию протокола")
        except (OSError, serial.SerialException, ValueError) as error:
            store.update(lidar_status="Coin D6: порт недоступен", last_error=str(error))
        stop.wait(1.0)
