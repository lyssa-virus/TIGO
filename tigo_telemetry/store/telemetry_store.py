from datetime import datetime, timezone
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

from core.csv_export import export_rows_to_csv
from core.models import (
    CmdVelSample,
    ConnectionConfig,
    ConnectionState,
    CsvRow,
    MockScenario,
    OdomSample,
    PidSample,
    TelemetryMode,
    TelemetrySnapshot,
    TopicRates,
    WheelPidMetrics,
)
from core.ring_buffer import RateTracker, RingBuffer
from providers.base import TelemetryCallbacks, TelemetryProvider
from providers.mock_simulator import MockRobotSimulator
from providers.ros2_native import Ros2NativeClient
from providers.rosbridge_ws import RosbridgeWsClient

CHART_BUFFER_CAPACITY = 2500
CSV_BUFFER_CAPACITY = 30000
TRAIL_MAX_POINTS = 4000
TRAIL_MIN_STEP_SQ = 0.015 * 0.015


class TelemetryStore:
    """
    Centraliza a ingestão em 20–100 Hz fora da thread de UI do Qt, fornecendo
    leituras rápidas para gráficos (25 FPS) e cards numéricos (10 Hz).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._session_start = time.monotonic()

        self._mode = TelemetryMode.MOCK
        self._conn_state = ConnectionState.MOCK
        self._status_detail = "Modo Simulação ativo"
        self._paused = False

        self._config = ConnectionConfig()
        self._mock_scenario = MockScenario.STEP_X
        self._mock_rate_hz = 50

        self._provider: Optional[TelemetryProvider] = None
        self._mock_sim: Optional[MockRobotSimulator] = None
        self._active_teleop_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        self._cmd_buf = RingBuffer[CmdVelSample](CHART_BUFFER_CAPACITY)
        self._odom_buf = RingBuffer[OdomSample](CHART_BUFFER_CAPACITY)
        self._pid_buf = RingBuffer[PidSample](CHART_BUFFER_CAPACITY)
        self._csv_buf = RingBuffer[CsvRow](CSV_BUFFER_CAPACITY)
        self._trail: List[Tuple[float, float]] = []
        self._ideal_trail: List[Tuple[float, float]] = []
        self._ideal_x = 0.0
        self._ideal_y = 0.0
        self._ideal_yaw = 0.0
        self._last_cmd_t: Optional[float] = None

        self._latest_cmd: Optional[CmdVelSample] = None
        self._latest_odom: Optional[OdomSample] = None
        self._latest_pid: Optional[PidSample] = None

        self._cmd_rate = RateTracker()
        self._odom_rate = RateTracker()
        self._pid_rate = RateTracker()

    def start(self) -> None:
        self._start_provider()

    def stop(self) -> None:
        self._stop_provider()

    def _build_callbacks(self) -> TelemetryCallbacks:
        return TelemetryCallbacks(
            on_status=self._on_status,
            on_cmd_vel=self._on_cmd_vel,
            on_odom=self._on_odom,
            on_pid=self._on_pid,
        )

    def _on_status(self, state: ConnectionState, detail: str) -> None:
        with self._lock:
            self._conn_state = state
            self._status_detail = detail

    def _on_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        now = time.monotonic()
        with self._lock:
            self._cmd_rate.tick(now)
            if self._paused:
                return
            sample = CmdVelSample(
                t=now - self._session_start,
                linear_x=vx,
                linear_y=vy,
                angular_z=wz,
            )
            self._latest_cmd = sample
            self._cmd_buf.push(sample)
            self._integrate_ideal_locked(sample.t, vx, vy, wz)
            self._push_csv_locked(sample.t)

    def _on_odom(
        self, x: float, y: float, yaw: float, vx: float, vy: float, wz: float
    ) -> None:
        now = time.monotonic()
        with self._lock:
            self._odom_rate.tick(now)
            if self._paused:
                return
            sample = OdomSample(
                t=now - self._session_start,
                x=x,
                y=y,
                yaw=yaw,
                vx=vx,
                vy=vy,
                wz=wz,
            )
            self._latest_odom = sample
            self._odom_buf.push(sample)
            self._append_trail_point(self._trail, x, y)

    def _on_pid(
        self, kp: float, ki: float, kd: float, wheels: Dict[str, WheelPidMetrics]
    ) -> None:
        now = time.monotonic()
        with self._lock:
            self._pid_rate.tick(now)
            if self._paused:
                return
            sample = PidSample(
                t=now - self._session_start,
                kp=kp,
                ki=ki,
                kd=kd,
                wheels=wheels,
            )
            self._latest_pid = sample
            self._pid_buf.push(sample)
            self._push_csv_locked(sample.t)

    def _append_trail_point(
        self, trail: List[Tuple[float, float]], x: float, y: float
    ) -> None:
        if trail:
            px, py = trail[-1]
            dx = x - px
            dy = y - py
            if dx * dx + dy * dy < TRAIL_MIN_STEP_SQ:
                return
        trail.append((x, y))
        if len(trail) > TRAIL_MAX_POINTS:
            trail.pop(0)

    def _integrate_ideal_locked(
        self, t_sec: float, vx: float, vy: float, wz: float
    ) -> None:
        if self._last_cmd_t is None:
            if self._latest_odom is not None:
                self._ideal_x = self._latest_odom.x
                self._ideal_y = self._latest_odom.y
                self._ideal_yaw = self._latest_odom.yaw
            self._last_cmd_t = t_sec
            self._ideal_trail.append((self._ideal_x, self._ideal_y))
            return

        dt = t_sec - self._last_cmd_t
        self._last_cmd_t = t_sec
        if dt <= 0.0 or dt > 1.0:
            return

        d_yaw = wz * dt
        mid_yaw = self._ideal_yaw + 0.5 * d_yaw
        cos_y = math.cos(mid_yaw)
        sin_y = math.sin(mid_yaw)
        self._ideal_x += (vx * cos_y - vy * sin_y) * dt
        self._ideal_y += (vx * sin_y + vy * cos_y) * dt
        self._ideal_yaw = math.atan2(
            math.sin(self._ideal_yaw + d_yaw),
            math.cos(self._ideal_yaw + d_yaw),
        )
        self._append_trail_point(self._ideal_trail, self._ideal_x, self._ideal_y)

    def _push_csv_locked(self, t_sec: float) -> None:
        cmd = self._latest_cmd
        odom = self._latest_odom
        pid = self._latest_pid
        fl = pid.wheels["FL"] if pid else None
        fr = pid.wheels["FR"] if pid else None
        rl = pid.wheels["RL"] if pid else None
        rr = pid.wheels["RR"] if pid else None

        self._csv_buf.push(
            CsvRow(
                timestamp_iso=datetime.now(timezone.utc).isoformat(),
                t_sec=t_sec,
                cmd_vx=cmd.linear_x if cmd else None,
                cmd_vy=cmd.linear_y if cmd else None,
                cmd_wz=cmd.angular_z if cmd else None,
                odom_x=odom.x if odom else None,
                odom_y=odom.y if odom else None,
                odom_yaw_deg=math.degrees(odom.yaw) if odom else None,
                odom_vx=odom.vx if odom else None,
                odom_vy=odom.vy if odom else None,
                odom_wz=odom.wz if odom else None,
                kp=pid.kp if pid else None,
                ki=pid.ki if pid else None,
                kd=pid.kd if pid else None,
                fl_setpoint=fl.setpoint if fl else None,
                fl_measured=fl.measured if fl else None,
                fl_error=fl.error if fl else None,
                fl_pwm=fl.total_pwm if fl else None,
                fr_setpoint=fr.setpoint if fr else None,
                fr_measured=fr.measured if fr else None,
                fr_error=fr.error if fr else None,
                fr_pwm=fr.total_pwm if fr else None,
                rl_setpoint=rl.setpoint if rl else None,
                rl_measured=rl.measured if rl else None,
                rl_error=rl.error if rl else None,
                rl_pwm=rl.total_pwm if rl else None,
                rr_setpoint=rr.setpoint if rr else None,
                rr_measured=rr.measured if rr else None,
                rr_error=rr.error if rr else None,
                rr_pwm=rr.total_pwm if rr else None,
            )
        )

    def _start_provider(self) -> None:
        self._stop_provider()
        callbacks = self._build_callbacks()
        with self._lock:
            mode = self._mode
            config = self._config
            scenario = self._mock_scenario
            rate_hz = self._mock_rate_hz

        if mode == TelemetryMode.MOCK:
            sim = MockRobotSimulator(callbacks, scenario=scenario, rate_hz=rate_hz)
            self._mock_sim = sim
            self._provider = sim
            sim.start()
        elif mode == TelemetryMode.ROS2_NATIVE:
            self._mock_sim = None
            client = Ros2NativeClient(config, callbacks)
            self._provider = client
            client.start()
        else:
            self._mock_sim = None
            client = RosbridgeWsClient(config, callbacks)
            self._provider = client
            client.start()

    def _stop_provider(self) -> None:
        with self._lock:
            self._active_teleop_cmd = (0.0, 0.0, 0.0)
        if self._provider is not None:
            self._provider.stop()
            self._provider = None
        self._mock_sim = None

    def set_mode(self, mode: TelemetryMode) -> None:
        with self._lock:
            if self._mode == mode:
                return
            self._mode = mode
        self.reset_data()
        self._start_provider()

    def update_connection_config(self, config: ConnectionConfig, reconnect: bool = True) -> None:
        with self._lock:
            self._config = config
            mode = self._mode
        if reconnect and mode != TelemetryMode.MOCK:
            self._start_provider()

    def reconnect(self) -> None:
        with self._lock:
            mode = self._mode
        if mode != TelemetryMode.MOCK:
            self._start_provider()

    def set_mock_scenario(self, scenario: MockScenario) -> None:
        with self._lock:
            self._mock_scenario = scenario
            sim = self._mock_sim
        if sim is not None:
            sim.set_scenario(scenario)

    def set_mock_rate_hz(self, hz: int) -> None:
        with self._lock:
            self._mock_rate_hz = hz
            sim = self._mock_sim
        if sim is not None:
            sim.set_rate_hz(hz)

    def set_mock_gains(self, kp: float, ki: float, kd: float) -> None:
        with self._lock:
            sim = self._mock_sim
        if sim is not None:
            sim.set_gains(kp, ki, kd)

    def get_mock_scenario(self) -> MockScenario:
        with self._lock:
            return self._mock_scenario

    def send_teleop_cmd(self, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            self._active_teleop_cmd = (vx, vy, wz)
            if self._mode == TelemetryMode.MOCK:
                self._mock_scenario = MockScenario.MANUAL
            provider = self._provider
        if provider is not None:
            provider.send_cmd_vel(vx, vy, wz)

    def send_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        with self._lock:
            provider = self._provider
        if provider is not None:
            provider.send_pid_gains(kp, ki, kd)

    def tick_teleop_keepalive(self) -> None:
        with self._lock:
            if self._mode == TelemetryMode.MOCK:
                return
            vx, vy, wz = self._active_teleop_cmd
            provider = self._provider
        if provider is not None and (
            abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(wz) > 1e-4
        ):
            provider.send_cmd_vel(vx, vy, wz)

    def toggle_paused(self) -> bool:
        with self._lock:
            self._paused = not self._paused
            return self._paused

    def is_paused(self) -> bool:
        with self._lock:
            return self._paused

    def clear_trail(self) -> None:
        with self._lock:
            self._trail.clear()
            self._ideal_trail.clear()
            self._last_cmd_t = None

    def reset_data(self) -> None:
        with self._lock:
            self._session_start = time.monotonic()
            self._cmd_buf.clear()
            self._odom_buf.clear()
            self._pid_buf.clear()
            self._csv_buf.clear()
            self._trail.clear()
            self._ideal_trail.clear()
            self._ideal_x = 0.0
            self._ideal_y = 0.0
            self._ideal_yaw = 0.0
            self._last_cmd_t = None
            self._latest_cmd = None
            self._latest_odom = None
            self._latest_pid = None
            self._cmd_rate.reset()
            self._odom_rate.reset()
            self._pid_rate.reset()

    def export_csv(self, file_path: str) -> int:
        with self._lock:
            rows = self._csv_buf.to_list()
        return export_rows_to_csv(file_path, rows)

    def get_config(self) -> ConnectionConfig:
        with self._lock:
            return self._config

    def get_snapshot(self) -> TelemetrySnapshot:
        now = time.monotonic()
        with self._lock:
            return TelemetrySnapshot(
                mode=self._mode,
                connection_state=self._conn_state,
                status_detail=self._status_detail,
                paused=self._paused,
                sample_count=len(self._csv_buf),
                rates=TopicRates(
                    cmd_vel_hz=self._cmd_rate.get_hz(now),
                    odom_hz=self._odom_rate.get_hz(now),
                    pid_hz=self._pid_rate.get_hz(now),
                ),
                cmd_vel=self._latest_cmd,
                odom=self._latest_odom,
                pid=self._latest_pid,
            )

    def get_cmd_vel_series(
        self, window_sec: float = 15.0
    ) -> Tuple[List[float], List[List[float]]]:
        with self._lock:
            items = self._cmd_buf.to_list()
        if not items:
            return [], [[], [], []]
        cutoff = items[-1].t - window_sec
        ts: List[float] = []
        vx: List[float] = []
        vy: List[float] = []
        wz: List[float] = []
        for s in items:
            if s.t < cutoff:
                continue
            ts.append(s.t)
            vx.append(s.linear_x)
            vy.append(s.linear_y)
            wz.append(s.angular_z)
        return ts, [vx, vy, wz]

    def get_pid_tracking_series(
        self, wheel: str, window_sec: float = 15.0
    ) -> Tuple[List[float], List[List[float]]]:
        with self._lock:
            items = self._pid_buf.to_list()
        if not items:
            return [], [[], []]
        cutoff = items[-1].t - window_sec
        ts: List[float] = []
        sp: List[float] = []
        pv: List[float] = []
        for s in items:
            if s.t < cutoff:
                continue
            w = s.wheels[wheel]
            ts.append(s.t)
            sp.append(w.setpoint)
            pv.append(w.measured)
        return ts, [sp, pv]

    def get_pid_effort_series(
        self, wheel: str, window_sec: float = 15.0
    ) -> Tuple[List[float], List[List[float]]]:
        with self._lock:
            items = self._pid_buf.to_list()
        if not items:
            return [], [[], [], []]
        cutoff = items[-1].t - window_sec
        ts: List[float] = []
        err: List[float] = []
        pwm: List[float] = []
        ff: List[float] = []
        for s in items:
            if s.t < cutoff:
                continue
            w = s.wheels[wheel]
            ts.append(s.t)
            err.append(w.error)
            pwm.append(w.total_pwm)
            ff.append(w.ff_pwm)
        return ts, [err, pwm, ff]

    def get_odom_trail(
        self,
    ) -> Tuple[
        List[Tuple[float, float]],
        List[Tuple[float, float]],
        Optional[OdomSample],
        Tuple[float, float, float],
    ]:
        with self._lock:
            return (
                list(self._trail),
                list(self._ideal_trail),
                self._latest_odom,
                (self._ideal_x, self._ideal_y, self._ideal_yaw),
            )
