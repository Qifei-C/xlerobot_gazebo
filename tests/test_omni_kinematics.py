import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "src" / "xlerobot_gazebo" / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import base_twist_utils as utils


class TestOmniKinematics(unittest.TestCase):
    def test_body_twist_to_omni_wheel_speeds_matches_expected_layout(self) -> None:
        wheel_speeds = utils.body_twist_to_omni_wheel_speeds(
            vx=0.0,
            vy=0.3,
            wz=0.0,
            wheel_count=3,
            wheel_offset=0.0,
            robot_radius=0.111,
            wheel_radius=0.0508,
        )
        expected = [0.3 / 0.0508, -0.15 / 0.0508, -0.15 / 0.0508]
        for actual, target in zip(wheel_speeds, expected):
            self.assertTrue(math.isclose(actual, target, rel_tol=1e-6))

    def test_omni_round_trip_recovers_body_twist(self) -> None:
        twist = (0.2, -0.1, 0.7)
        wheel_speeds = utils.body_twist_to_omni_wheel_speeds(
            *twist,
            wheel_count=3,
            wheel_offset=0.0,
            robot_radius=0.111,
            wheel_radius=0.0508,
        )
        recovered = utils.omni_wheel_speeds_to_body_twist(
            wheel_speeds,
            wheel_count=3,
            wheel_offset=0.0,
            robot_radius=0.111,
            wheel_radius=0.0508,
        )
        for actual, target in zip(recovered, twist):
            self.assertTrue(math.isclose(actual, target, rel_tol=1e-6, abs_tol=1e-6))

    def test_apply_omni_wheel_speed_limit_uses_same_sign_convention_as_controller(self) -> None:
        limited = utils.apply_omni_wheel_speed_limit(
            0.0,
            0.3,
            1.57,
            wheel_count=3,
            wheel_offset=0.0,
            robot_radius=0.111,
            wheel_radius=0.0508,
            max_wheel_speed=6.0,
        )
        wheel_speeds = utils.body_twist_to_omni_wheel_speeds(
            *limited,
            wheel_count=3,
            wheel_offset=0.0,
            robot_radius=0.111,
            wheel_radius=0.0508,
        )
        self.assertLessEqual(max(abs(speed) for speed in wheel_speeds), 6.0 + 1e-6)


if __name__ == "__main__":
    unittest.main()
