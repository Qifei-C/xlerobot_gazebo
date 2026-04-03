import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "src" / "xlerobot_gazebo" / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import base_twist_utils as utils


class TestBaseTwistUtils(unittest.TestCase):
    def test_clamp_body_twist_limits_planar_and_angular_speed(self) -> None:
        vx, vy, wz = utils.clamp_body_twist(0.6, 0.8, 4.0, 0.5, 1.5)
        self.assertTrue(math.isclose(math.hypot(vx, vy), 0.5, rel_tol=1e-6))
        self.assertTrue(math.isclose(wz, 1.5, rel_tol=1e-6))

    def test_apply_acceleration_limits_ramps_toward_target(self) -> None:
        current = (0.0, 0.0, 0.0)
        target = (1.0, 0.0, 3.0)
        vx, vy, wz = utils.apply_acceleration_limits(current, target, 0.1, 0.6, 2.0)
        self.assertTrue(math.isclose(vx, 0.06, rel_tol=1e-6))
        self.assertTrue(math.isclose(vy, 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(wz, 0.2, rel_tol=1e-6))

    def test_apply_omni_wheel_speed_limit_scales_body_twist(self) -> None:
        vx, vy, wz = utils.apply_omni_wheel_speed_limit(
            1.0,
            0.0,
            0.0,
            wheel_count=3,
            wheel_offset=0.0,
            robot_radius=0.23,
            wheel_radius=0.06,
            max_wheel_speed=5.0,
        )
        self.assertLess(math.hypot(vx, vy), 1.0)
        wheel_speeds = []
        for index in range(3):
            alpha = index * (2.0 * math.pi / 3.0)
            wheel_speeds.append(
                (-math.sin(alpha) * vx + math.cos(alpha) * vy - 0.23 * wz) / 0.06
            )
        self.assertLessEqual(max(abs(speed) for speed in wheel_speeds), 5.0 + 1e-6)

    def test_limit_body_twist_applies_body_and_wheel_envelope(self) -> None:
        config = utils.LimitConfig(
            max_planar_speed=0.3,
            max_angular_speed=1.57,
            max_planar_acceleration=10.0,
            max_angular_acceleration=10.0,
            max_wheel_speed=3.0,
            wheel_count=3,
            wheel_offset=0.0,
            robot_radius=0.23,
            wheel_radius=0.06,
        )
        vx, vy, wz = utils.limit_body_twist(
            (0.0, 0.0, 0.0),
            (0.3, 0.3, 0.0),
            1.0,
            config,
        )
        self.assertLessEqual(math.hypot(vx, vy), 0.3 + 1e-6)
        self.assertLessEqual(abs(wz), 1.57 + 1e-6)

    def test_official_base_action_to_twist_uses_deg_per_second_for_theta(self) -> None:
        vx, vy, wz = utils.official_base_action_to_twist(
            {"x.vel": 0.2, "y.vel": -0.1, "theta.vel": 90.0}
        )
        self.assertTrue(math.isclose(vx, 0.2, rel_tol=1e-6))
        self.assertTrue(math.isclose(vy, -0.1, rel_tol=1e-6))
        self.assertTrue(math.isclose(wz, math.pi / 2.0, rel_tol=1e-6))

    def test_odom_twist_to_official_base_observation_returns_deg_per_second(self) -> None:
        observation = utils.odom_twist_to_official_base_observation(0.2, -0.1, math.pi / 2.0)
        self.assertEqual(observation["x.vel"], 0.2)
        self.assertEqual(observation["y.vel"], -0.1)
        self.assertTrue(math.isclose(observation["theta.vel"], 90.0, rel_tol=1e-6))

    def test_official_position_to_radians_in_degrees_mode(self) -> None:
        radians = utils.official_position_to_radians(90.0, -math.pi, math.pi, True, False)
        self.assertTrue(math.isclose(radians, math.pi / 2.0, rel_tol=1e-6))

    def test_official_position_to_radians_in_normalized_mode(self) -> None:
        radians = utils.official_position_to_radians(0.0, -2.0, 2.0, False, False)
        self.assertTrue(math.isclose(radians, 0.0, abs_tol=1e-9))
        upper = utils.official_position_to_radians(100.0, -2.0, 2.0, False, False)
        self.assertTrue(math.isclose(upper, 2.0, abs_tol=1e-9))

    def test_official_position_to_radians_for_gripper_range(self) -> None:
        radians = utils.official_position_to_radians(50.0, 0.0, 1.7, False, True)
        self.assertTrue(math.isclose(radians, 0.85, rel_tol=1e-6))

    def test_radians_to_official_position_in_normalized_mode(self) -> None:
        position = utils.radians_to_official_position(0.0, -2.0, 2.0, False, False)
        self.assertTrue(math.isclose(position, 0.0, abs_tol=1e-9))

    def test_radians_to_official_position_for_gripper_range(self) -> None:
        position = utils.radians_to_official_position(0.85, 0.0, 1.7, False, True)
        self.assertTrue(math.isclose(position, 50.0, rel_tol=1e-6))


if __name__ == "__main__":
    unittest.main()
