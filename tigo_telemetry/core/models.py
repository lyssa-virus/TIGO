from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple

WHEEL_IDS: Tuple[str, ...] = ("FL", "FR", "RL", "RR")

WHEEL_LABELS: Dict[str, str] = {
    "FL": "Dianteira Esq. (FL)",
    "FR": "Dianteira Dir. (FR)",
    "RL": "Traseira Esq. (RL)",
    "RR": "Traseira Dir. (RR)",
}


class TelemetryMode(str, Enum):
    MOCK = "mock"
    ROS2_NATIVE = "ros2_native"
    ROSBRIDGE = "rosbridge"


class ConnectionState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"
    MOCK = "mock"


class MockScenario(str, Enum):
    STEP_X = "step_x"
    STRAFE_Y = "strafe_y"
    ROTATE_Z = "rotate_z"
    SQUARE_MECANUM = "square_mecanum"
    CIRCLE_OMNI = "circle_omni"
    MANUAL = "manual"


MOCK_SCENARIO_LABELS: Dict[MockScenario, str] = {
    MockScenario.STEP_X: "Degrau Frontal (vx)",
    MockScenario.STRAFE_Y: "Strafe Lateral (vy)",
    MockScenario.ROTATE_Z: "Rotação Pura (wz)",
    MockScenario.SQUARE_MECANUM: "Quadrado Mecanum",
    MockScenario.CIRCLE_OMNI: "Círculo Omnidirecional",
    MockScenario.MANUAL: "Controle Manual (Botões)",
}


@dataclass(frozen=True)
class CmdVelSample:
    t: float
    linear_x: float
    linear_y: float
    angular_z: float


@dataclass(frozen=True)
class OdomSample:
    t: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class WheelPidMetrics:
    setpoint: float
    measured: float
    error: float
    ff_pwm: float
    total_pwm: float


@dataclass(frozen=True)
class PidSample:
    t: float
    kp: float
    ki: float
    kd: float
    wheels: Dict[str, WheelPidMetrics]


@dataclass(frozen=True)
class TopicNamesConfig:
    cmd_vel: str = "/cmd_vel"
    odom: str = "/odom"
    pid: str = "/tigo/pid_debug"
    pid_gains: str = "/tigo/pid_gains"


@dataclass(frozen=True)
class ConnectionConfig:
    host: str = "localhost"
    port: int = 9090
    topics: TopicNamesConfig = TopicNamesConfig()


@dataclass(frozen=True)
class TopicRates:
    cmd_vel_hz: float
    odom_hz: float
    pid_hz: float


@dataclass(frozen=True)
class TelemetrySnapshot:
    mode: TelemetryMode
    connection_state: ConnectionState
    status_detail: str
    paused: bool
    sample_count: int
    rates: TopicRates
    cmd_vel: Optional[CmdVelSample]
    odom: Optional[OdomSample]
    pid: Optional[PidSample]


@dataclass(frozen=True)
class CsvRow:
    timestamp_iso: str
    t_sec: float
    cmd_vx: Optional[float]
    cmd_vy: Optional[float]
    cmd_wz: Optional[float]
    odom_x: Optional[float]
    odom_y: Optional[float]
    odom_yaw_deg: Optional[float]
    odom_vx: Optional[float]
    odom_vy: Optional[float]
    odom_wz: Optional[float]
    kp: Optional[float]
    ki: Optional[float]
    kd: Optional[float]
    fl_setpoint: Optional[float]
    fl_measured: Optional[float]
    fl_error: Optional[float]
    fl_pwm: Optional[float]
    fr_setpoint: Optional[float]
    fr_measured: Optional[float]
    fr_error: Optional[float]
    fr_pwm: Optional[float]
    rl_setpoint: Optional[float]
    rl_measured: Optional[float]
    rl_error: Optional[float]
    rl_pwm: Optional[float]
    rr_setpoint: Optional[float]
    rr_measured: Optional[float]
    rr_error: Optional[float]
    rr_pwm: Optional[float]
