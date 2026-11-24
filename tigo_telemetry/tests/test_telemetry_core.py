import math
import os
import sys
import tempfile
import unittest

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

from core.csv_export import export_rows_to_csv
from core.kinematics import (
    forward_mecanum_kinematics,
    inverse_mecanum_kinematics,
    quaternion_to_yaw,
)
from core.models import CsvRow, WHEEL_IDS
from core.ring_buffer import RingBuffer
from providers.base import TelemetryCallbacks, parse_pid_array
from providers.mock_simulator import MockRobotSimulator
from providers.rosbridge_ws import encode_ws_frame
from store.telemetry_store import TelemetryStore


class TestTelemetryCore(unittest.TestCase):
    def test_mecanum_kinematics_roundtrip(self) -> None:
        vx, vy, wz = 0.42, -0.18, 0.75
        wheels = inverse_mecanum_kinematics(vx, vy, wz)
        rvx, rvy, rwz = forward_mecanum_kinematics(wheels)
        self.assertAlmostEqual(vx, rvx, places=6)
        self.assertAlmostEqual(vy, rvy, places=6)
        self.assertAlmostEqual(wz, rwz, places=6)

    def test_quaternion_to_yaw(self) -> None:
        # 90 graus em torno do eixo Z: q = (0, 0, sin(pi/4), cos(pi/4))
        angle = quaternion_to_yaw(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
        self.assertAlmostEqual(angle, math.pi / 2, places=6)

    def test_ring_buffer_capacity(self) -> None:
        buf = RingBuffer[int](3)
        for i in range(5):
            buf.push(i)
        self.assertEqual(buf.to_list(), [2, 3, 4])
        self.assertEqual(buf.latest(), 4)

    def test_parse_pid_array(self) -> None:
        arr = [4.0, 10.0, 0.02] + [1.0, 0.9, 0.1, 30.0, 35.0] * 4
        parsed = parse_pid_array(arr)
        self.assertIsNotNone(parsed)
        kp, ki, kd, wheels = parsed  # type: ignore[misc]
        self.assertEqual((kp, ki, kd), (4.0, 10.0, 0.02))
        self.assertEqual(set(wheels.keys()), set(WHEEL_IDS))
        self.assertAlmostEqual(wheels["FL"].total_pwm, 35.0)

    def test_mock_simulator_step_emits_all_streams(self) -> None:
        counts = {"cmd": 0, "odom": 0, "pid": 0}
        cb = TelemetryCallbacks(
            on_status=lambda _s, _d: None,
            on_cmd_vel=lambda _vx, _vy, _wz: counts.__setitem__("cmd", counts["cmd"] + 1),
            on_odom=lambda *_args: counts.__setitem__("odom", counts["odom"] + 1),
            on_pid=lambda *_args: counts.__setitem__("pid", counts["pid"] + 1),
        )
        sim = MockRobotSimulator(cb)
        for _ in range(10):
            sim.step(0.02)
        self.assertEqual(counts, {"cmd": 10, "odom": 10, "pid": 10})

    def test_mock_simulator_manual_teleop_commands(self) -> None:
        last_cmd = []
        cb = TelemetryCallbacks(
            on_status=lambda _s, _d: None,
            on_cmd_vel=lambda vx, vy, wz: last_cmd.append((vx, vy, wz)),
            on_odom=lambda *_args: None,
            on_pid=lambda *_args: None,
        )
        sim = MockRobotSimulator(cb)
        for cmd in (
            (0.35, 0.0, 0.0),
            (-0.35, 0.0, 0.0),
            (0.0, 0.35, 0.0),
            (0.0, -0.35, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 0.0, -1.0),
            (0.0, 0.0, 0.0),
        ):
            sim.send_cmd_vel(*cmd)
            sim.step(0.02)
            self.assertEqual(last_cmd[-1], cmd)

    def test_mock_simulator_pid_gains_update(self) -> None:
        pids = []
        cb = TelemetryCallbacks(
            on_status=lambda _s, _d: None,
            on_cmd_vel=lambda *_args: None,
            on_odom=lambda *_args: None,
            on_pid=lambda kp, ki, kd, _w: pids.append((kp, ki, kd)),
        )
        sim = MockRobotSimulator(cb)
        sim.send_pid_gains(6.5, 12.0, 0.05)
        sim.step(0.02)
        self.assertEqual(pids[-1], (6.5, 12.0, 0.05))

    def test_store_tracks_ideal_and_measured_trajectories(self) -> None:
        store = TelemetryStore()
        store._on_cmd_vel(0.5, 0.0, 0.0)
        store._session_start -= 0.5  # simula 0.5s decorrido
        store._on_cmd_vel(0.5, 0.0, 0.0)
        store._on_odom(0.22, 0.02, 0.0, 0.45, 0.0, 0.0)

        measured, ideal, odom_pose, ideal_pose = store.get_odom_trail()
        self.assertEqual(len(measured), 1)
        self.assertEqual(len(ideal), 2)
        self.assertIsNotNone(odom_pose)
        self.assertAlmostEqual(ideal_pose[0], 0.25, delta=0.03)

        store.clear_trail()
        measured2, ideal2, _, _ = store.get_odom_trail()
        self.assertEqual(measured2, [])
        self.assertEqual(ideal2, [])

    def test_ws_frame_encoding_masked(self) -> None:
        frame = encode_ws_frame(b"hello")
        self.assertEqual(frame[0], 0x81)
        self.assertEqual(frame[1], 0x80 | 5)
        self.assertEqual(len(frame), 2 + 4 + 5)

    def test_csv_export(self) -> None:
        row = CsvRow(
            timestamp_iso="2026-11-24T16:00:00Z",
            t_sec=1.25,
            cmd_vx=0.4,
            cmd_vy=0.0,
            cmd_wz=0.0,
            odom_x=0.5,
            odom_y=0.0,
            odom_yaw_deg=0.0,
            odom_vx=0.39,
            odom_vy=0.0,
            odom_wz=0.0,
            kp=4.0,
            ki=10.0,
            kd=0.02,
            fl_setpoint=9.75,
            fl_measured=9.5,
            fl_error=0.25,
            fl_pwm=80.0,
            fr_setpoint=9.75,
            fr_measured=9.5,
            fr_error=0.25,
            fr_pwm=80.0,
            rl_setpoint=9.75,
            rl_measured=9.5,
            rl_error=0.25,
            rl_pwm=80.0,
            rr_setpoint=9.75,
            rr_measured=9.5,
            rr_error=0.25,
            rr_pwm=80.0,
        )
        with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as tmp:
            path = tmp.name
        try:
            written = export_rows_to_csv(path, [row])
            self.assertEqual(written, 1)
            with open(path, encoding="utf-8") as fh:
                lines = fh.read().strip().splitlines()
            self.assertEqual(len(lines), 2)
            self.assertIn("cmd_vx", lines[0])
        finally:
            os.remove(path)


if __name__ == "__main__":
    unittest.main()
