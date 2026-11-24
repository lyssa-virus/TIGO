import math
import random
import threading
import time
from typing import Dict, Tuple

from core.kinematics import (
    DEADBAND_VEL,
    DEFAULT_KD,
    DEFAULT_KI,
    DEFAULT_KP,
    MAX_I_PWM,
    PWM_MAX,
    W_MAX_RAD_S,
    clamp,
    forward_mecanum_kinematics,
    inverse_mecanum_kinematics,
)
from core.models import ConnectionState, MockScenario, WHEEL_IDS, WheelPidMetrics
from providers.base import TelemetryCallbacks, TelemetryProvider


class _WheelSimState:
    def __init__(self, gain_factor: float) -> None:
        self.omega = 0.0
        self.accel_state = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.gain_factor = gain_factor


class MockRobotSimulator(TelemetryProvider):
    """
    Simulador físico Mecanum + PID offline baseado no firmware controle_tigo.ino.
    Permite testar /cmd_vel, /odom e /tigo/pid_debug em 20–100 Hz com o robô desligado.
    """

    def __init__(
        self,
        callbacks: TelemetryCallbacks,
        scenario: MockScenario = MockScenario.STEP_X,
        rate_hz: int = 50,
    ) -> None:
        self._callbacks = callbacks
        self._lock = threading.Lock()
        self._scenario = scenario
        self._rate_hz = int(clamp(rate_hz, 10, 100))
        self._kp = DEFAULT_KP
        self._ki = DEFAULT_KI
        self._kd = DEFAULT_KD
        self._manual_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        self._sim_time = 0.0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._wheels: Dict[str, _WheelSimState] = {
            "FL": _WheelSimState(0.96),
            "FR": _WheelSimState(1.03),
            "RL": _WheelSimState(0.98),
            "RR": _WheelSimState(1.01),
        }

        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def set_scenario(self, scenario: MockScenario) -> None:
        with self._lock:
            self._scenario = scenario

    def set_rate_hz(self, rate_hz: int) -> None:
        with self._lock:
            self._rate_hz = int(clamp(rate_hz, 10, 100))
            hz = self._rate_hz
        self._callbacks.on_status(
            ConnectionState.MOCK, f"Modo Simulação ativo ({hz} Hz)"
        )

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        with self._lock:
            self._kp = kp
            self._ki = ki
            self._kd = kd

    def send_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            self._manual_cmd = (vx, vy, wz)
            self._scenario = MockScenario.MANUAL

    def send_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        self.set_gains(kp, ki, kd)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._callbacks.on_status(
            ConnectionState.MOCK, f"Modo Simulação ativo ({self._rate_hz} Hz)"
        )
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

    def _run_loop(self) -> None:
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            with self._lock:
                hz = self._rate_hz
            dt = 1.0 / hz
            self.step(dt)
            next_tick += dt
            sleep_sec = next_tick - time.perf_counter()
            if sleep_sec > 0:
                self._stop_event.wait(sleep_sec)
            else:
                next_tick = time.perf_counter()

    def _compute_command(self, t: float, scenario: MockScenario) -> Tuple[float, float, float]:
        if scenario == MockScenario.MANUAL:
            with self._lock:
                return self._manual_cmd
        cycle = t % 8.0
        if scenario == MockScenario.STEP_X:
            if cycle < 3.0:
                return 0.45, 0.0, 0.0
            if cycle < 4.5:
                return 0.0, 0.0, 0.0
            if cycle < 7.0:
                return -0.35, 0.0, 0.0
            return 0.0, 0.0, 0.0
        if scenario == MockScenario.STRAFE_Y:
            if cycle < 3.5:
                return 0.0, 0.40, 0.0
            if cycle < 4.5:
                return 0.0, 0.0, 0.0
            return 0.0, -0.40, 0.0
        if scenario == MockScenario.ROTATE_Z:
            if cycle < 3.5:
                return 0.0, 0.0, 1.20
            if cycle < 4.5:
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, -1.20
        if scenario == MockScenario.SQUARE_MECANUM:
            if cycle < 2.0:
                return 0.40, 0.0, 0.0
            if cycle < 4.0:
                return 0.0, 0.40, 0.0
            if cycle < 6.0:
                return -0.40, 0.0, 0.0
            return 0.0, -0.40, 0.0
        return 0.35, 0.10 * math.sin(t * 0.8), 0.65

    def step(self, dt: float) -> None:
        with self._lock:
            scenario = self._scenario
            kp, ki, kd = self._kp, self._ki, self._kd

        self._sim_time += dt
        vx, vy, wz = self._compute_command(self._sim_time, scenario)
        self._callbacks.on_cmd_vel(vx, vy, wz)

        targets = inverse_mecanum_kinematics(vx, vy, wz)
        measured_wheels: Dict[str, float] = {}
        pid_wheels: Dict[str, WheelPidMetrics] = {}

        stopped = abs(vx) < 0.001 and abs(vy) < 0.001 and abs(wz) < 0.001

        for wid in WHEEL_IDS:
            st = self._wheels[wid]
            setpoint = targets[wid]
            measured = st.omega + (random.random() - 0.5) * 0.16

            ff_pwm = 0.0
            total_pwm = 0.0
            error = setpoint - measured

            if stopped or abs(setpoint) < 0.01:
                st.integral = 0.0
                st.prev_error = 0.0
                error = -measured
            else:
                ff_pwm = (setpoint / W_MAX_RAD_S) * PWM_MAX
                eff_err = 0.0 if abs(error) < DEADBAND_VEL else error
                p_term = kp * eff_err

                max_int = (MAX_I_PWM / ki) if ki > 0.0001 else 0.0
                prev_out = ff_pwm + p_term
                sat_pos = prev_out >= PWM_MAX and eff_err > 0.0
                sat_neg = prev_out <= -PWM_MAX and eff_err < 0.0
                if not sat_pos and not sat_neg:
                    st.integral = clamp(st.integral + eff_err * dt, -max_int, max_int)

                i_term = clamp(ki * st.integral, -MAX_I_PWM, MAX_I_PWM)
                d_term = kd * ((eff_err - st.prev_error) / dt) if dt > 0.001 else 0.0
                st.prev_error = eff_err
                total_pwm = clamp(ff_pwm + p_term + i_term + d_term, -PWM_MAX, PWM_MAX)

            # Dinâmica de 2ª ordem do motor para exibir overshoot e tempo de acomodação
            target_omega = (total_pwm / PWM_MAX) * W_MAX_RAD_S * st.gain_factor
            wn = 12.0
            zeta = 0.62
            accel = wn * wn * (target_omega - st.omega) - 2.0 * zeta * wn * st.accel_state
            st.accel_state += accel * dt
            st.omega += st.accel_state * dt

            measured_wheels[wid] = st.omega
            pid_wheels[wid] = WheelPidMetrics(
                setpoint=setpoint,
                measured=measured,
                error=error,
                ff_pwm=ff_pwm,
                total_pwm=total_pwm,
            )

        self._callbacks.on_pid(kp, ki, kd, pid_wheels)

        body_vx, body_vy, body_wz = forward_mecanum_kinematics(measured_wheels)
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        self._x += (body_vx * cos_y - body_vy * sin_y) * dt
        self._y += (body_vx * sin_y + body_vy * cos_y) * dt
        self._yaw = math.atan2(
            math.sin(self._yaw + body_wz * dt),
            math.cos(self._yaw + body_wz * dt),
        )
        self._callbacks.on_odom(self._x, self._y, self._yaw, body_vx, body_vy, body_wz)
