from abc import ABC, abstractmethod
from typing import Callable, Dict, Optional, Sequence, Tuple
from core.models import ConnectionState, WHEEL_IDS, WheelPidMetrics


def parse_pid_array(
    data: Sequence[float],
) -> Optional[Tuple[float, float, float, Dict[str, WheelPidMetrics]]]:
    """Desempacota o Float64MultiArray (23 valores) de /tigo/pid_debug."""
    if len(data) < 23:
        return None
    wheels: Dict[str, WheelPidMetrics] = {}
    for idx, wid in enumerate(WHEEL_IDS):
        off = 3 + idx * 5
        wheels[wid] = WheelPidMetrics(
            setpoint=float(data[off]),
            measured=float(data[off + 1]),
            error=float(data[off + 2]),
            ff_pwm=float(data[off + 3]),
            total_pwm=float(data[off + 4]),
        )
    return float(data[0]), float(data[1]), float(data[2]), wheels


class TelemetryCallbacks:
    """Callbacks acionados pelas threads dos provedores para alimentar o TelemetryStore."""

    def __init__(
        self,
        on_status: Callable[[ConnectionState, str], None],
        on_cmd_vel: Callable[[float, float, float], None],
        on_odom: Callable[[float, float, float, float, float, float], None],
        on_pid: Callable[[float, float, float, Dict[str, WheelPidMetrics]], None],
    ) -> None:
        self.on_status = on_status
        self.on_cmd_vel = on_cmd_vel
        self.on_odom = on_odom
        self.on_pid = on_pid


class TelemetryProvider(ABC):
    @abstractmethod
    def start(self) -> None:
        """Inicia a coleta de dados em segundo plano."""

    @abstractmethod
    def stop(self) -> None:
        """Encerra a coleta de dados e libera recursos."""

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """Envia comando /cmd_vel (opcional conforme o provedor)."""
        _ = (vx, vy, wz)

    def send_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        """Envia ganhos (Kp, Ki, Kd) para o controlador (opcional conforme o provedor)."""
        _ = (kp, ki, kd)
