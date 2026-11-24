import math
import os
import sys
import unittest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from tigo_base.serial_protocol import (
    MecanumOdomState,
    parse_encoder_line,
    parse_pid_line,
    update_mecanum_odom,
)


class TestSerialProtocol(unittest.TestCase):
    def test_parse_encoder_line(self) -> None:
        self.assertEqual(parse_encoder_line('E,10,-20,30,-40'), (10, -20, 30, -40))
        self.assertIsNone(parse_encoder_line('E,10,abc,30,40'))
        self.assertIsNone(parse_encoder_line('P,1,2,3'))

    def test_update_mecanum_odom_forward_and_turn(self) -> None:
        state = MecanumOdomState()
        # Primeira leitura apenas inicializa prev_ticks
        self.assertIsNone(
            update_mecanum_odom(state, (100, 100, 100, 100), 0.05, 0.041, 0.082, 0.106, 1319.1)
        )
        # Avança 1 volta completa nas 4 rodas em 1.0 s -> x = 2 * pi * 0.041 m
        vel = update_mecanum_odom(
            state,
            (100 + 1319, 100 + 1319, 100 + 1319, 100 + 1319),
            1.0,
            0.041,
            0.082,
            0.106,
            1319.1,
        )
        self.assertIsNotNone(vel)
        vx, vy, wz = vel  # type: ignore[misc]
        expected_dist = 2.0 * math.pi * 0.041
        self.assertAlmostEqual(vx, expected_dist, places=3)
        self.assertAlmostEqual(vy, 0.0, places=5)
        self.assertAlmostEqual(wz, 0.0, places=5)
        self.assertAlmostEqual(state.x, expected_dist, places=3)
        self.assertAlmostEqual(state.y, 0.0, places=5)

    def test_parse_pid_line_expands_to_23_floats(self) -> None:
        line = (
            'P,4.000,10.000,0.0200,'
            '10.00,9.50,80.0,85.0,'
            '10.00,9.80,80.0,82.0,'
            '10.00,9.40,80.0,86.0,'
            '10.00,9.90,80.0,81.0'
        )
        vec = parse_pid_line(line)
        self.assertIsNotNone(vec)
        assert vec is not None
        self.assertEqual(len(vec), 23)
        self.assertEqual(vec[0:3], [4.0, 10.0, 0.02])
        # FL: sp=10.0, pv=9.5, err=0.5, ff=80.0, cmd=85.0
        self.assertAlmostEqual(vec[3], 10.0)
        self.assertAlmostEqual(vec[4], 9.5)
        self.assertAlmostEqual(vec[5], 0.5)
        self.assertAlmostEqual(vec[6], 80.0)
        self.assertAlmostEqual(vec[7], 85.0)


if __name__ == '__main__':
    unittest.main()
