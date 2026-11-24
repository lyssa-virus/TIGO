from dataclasses import dataclass
import math
from typing import List, Optional, Tuple


@dataclass
class MecanumOdomState:
    """Estado acumulado de pose e encoders para cálculo incremental de odometria."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    prev_ticks: Optional[Tuple[int, int, int, int]] = None


def parse_encoder_line(line: str) -> Optional[Tuple[int, int, int, int]]:
    """Extrai (fl, fr, rl, rr) de uma linha serial no formato 'E,fl,fr,rl,rr'."""
    if not line.startswith('E,'):
        return None
    parts = line.strip().split(',')
    if len(parts) != 5:
        return None
    try:
        return int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])
    except ValueError:
        return None


def update_mecanum_odom(
    state: MecanumOdomState,
    ticks: Tuple[int, int, int, int],
    dt: float,
    wheel_radius: float,
    lx: float,
    ly: float,
    ticks_per_rev: float,
) -> Optional[Tuple[float, float, float]]:
    """
    Atualiza a pose (x, y, yaw) em metros/radianos e retorna (vx, vy, wz).

    Na primeira leitura apenas inicializa prev_ticks para evitar saltos iniciais.
    """
    if state.prev_ticks is None:
        state.prev_ticks = ticks
        return None

    if dt <= 1e-4 or ticks_per_rev <= 0.0:
        state.prev_ticks = ticks
        return None

    d_fl = ticks[0] - state.prev_ticks[0]
    d_fr = ticks[1] - state.prev_ticks[1]
    d_rl = ticks[2] - state.prev_ticks[2]
    d_rr = ticks[3] - state.prev_ticks[3]
    state.prev_ticks = ticks

    rad_per_tick = (2.0 * math.pi) / ticks_per_rev
    w_fl = (d_fl * rad_per_tick) / dt
    w_fr = (d_fr * rad_per_tick) / dt
    w_rl = (d_rl * rad_per_tick) / dt
    w_rr = (d_rr * rad_per_tick) / dt

    k_geom = lx + ly
    vx = (wheel_radius / 4.0) * (w_fl + w_fr + w_rl + w_rr)
    vy = (wheel_radius / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
    wz = (wheel_radius / (4.0 * k_geom)) * (-w_fl + w_fr - w_rl + w_rr)

    d_yaw = wz * dt
    mid_yaw = state.yaw + 0.5 * d_yaw
    cos_y = math.cos(mid_yaw)
    sin_y = math.sin(mid_yaw)

    state.x += (vx * cos_y - vy * sin_y) * dt
    state.y += (vx * sin_y + vy * cos_y) * dt
    state.yaw = math.atan2(
        math.sin(state.yaw + d_yaw), math.cos(state.yaw + d_yaw)
    )
    return vx, vy, wz


def parse_pid_line(line: str) -> Optional[List[float]]:
    """
    Converte a linha serial 'P,kp,ki,kd,spFL,pvFL,ffFL,cmdFL,...' (19 floats)
    para o vetor de 23 floats publicado em /tigo/pid_debug.
    """
    if not line.startswith('P,'):
        return None
    parts = line.strip().split(',')
    if len(parts) != 20:
        return None
    try:
        vals = [float(p) for p in parts[1:]]
    except ValueError:
        return None

    out: List[float] = [vals[0], vals[1], vals[2]]
    for wheel_idx in range(4):
        base = 3 + wheel_idx * 4
        sp = vals[base]
        pv = vals[base + 1]
        ff_pwm = vals[base + 2]
        cmd_pwm = vals[base + 3]
        out.extend([sp, pv, sp - pv, ff_pwm, cmd_pwm])
    return out
