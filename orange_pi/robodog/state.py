from dataclasses import dataclass, field
from threading import Lock
from time import monotonic
from typing import List, Optional, Tuple
from .protocol import Control, Imu, Telemetry


@dataclass
class Snapshot:
    telemetry: Optional[Telemetry] = None
    telemetry_at: float = 0.0
    imu: Optional[Imu] = None
    imu_at: float = 0.0
    control: Optional[Control] = None
    control_at: float = 0.0
    control_line: bytes = b""
    lidar: List[Tuple[float, float]] = field(default_factory=list)
    lidar_at: float = 0.0
    lidar_label: str = "Нет данных"
    mega_status: str = "Mega не подключена"
    remote_status: str = "Пульт не подключён"
    lidar_status: str = "LiDAR не подключён"
    last_error: str = ""


class Store:
    def __init__(self) -> None:
        self._lock = Lock()
        self._value = Snapshot()

    def update(self, **changes) -> None:
        with self._lock:
            for key, value in changes.items():
                if not hasattr(self._value, key):
                    raise KeyError(key)
                setattr(self._value, key, value)

    def snapshot(self) -> Snapshot:
        with self._lock:
            return Snapshot(**vars(self._value))
