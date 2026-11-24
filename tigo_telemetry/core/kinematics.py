import math
from typing import Dict, Tuple

# Parâmetros geométricos e de controle extraídos de firmware/controle_tigo/controle_tigo.ino
WHEEL_RADIUS_M = 0.041
LX_M = 0.082
LY_M = 0.106
K_GEOM_M = LX_M + LY_M  # 0.188 m
W_MAX_RAD_S = 34.55
PWM_MAX = 255.0
PWM_MIN = 20.0
MAX_I_PWM = 80.0
DEADBAND_VEL = 0.10
DEFAULT_KP = 4.0
DEFAULT_KI = 10.0
DEFAULT_KD = 0.02


def clamp(val: float, low: float, high: float) -> float:
    return max(low, min(high, val))


def inverse_mecanum_kinematics(vx: float, vy: float, wz: float) -> Dict[str, float]:
    """Calcula velocidades angulares alvo (rad/s) para as 4 rodas mecanum."""
    return {
        "FL": (vx - vy - K_GEOM_M * wz) / WHEEL_RADIUS_M,
        "FR": (vx + vy + K_GEOM_M * wz) / WHEEL_RADIUS_M,
        "RL": (vx + vy - K_GEOM_M * wz) / WHEEL_RADIUS_M,
        "RR": (vx - vy + K_GEOM_M * wz) / WHEEL_RADIUS_M,
    }


def forward_mecanum_kinematics(wheels: Dict[str, float]) -> Tuple[float, float, float]:
    """Calcula velocidades (vx, vy, wz) do chassi a partir das 4 rodas."""
    fl = wheels["FL"]
    fr = wheels["FR"]
    rl = wheels["RL"]
    rr = wheels["RR"]
    vx = (WHEEL_RADIUS_M / 4.0) * (fl + fr + rl + rr)
    vy = (WHEEL_RADIUS_M / 4.0) * (-fl + fr + rl - rr)
    wz = (WHEEL_RADIUS_M / (4.0 * K_GEOM_M)) * (-fl + fr - rl + rr)
    return vx, vy, wz


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Converte quaternion ROS (x, y, z, w) em ângulo yaw (radianos)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
