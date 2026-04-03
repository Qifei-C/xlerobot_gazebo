import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "src" / "xlerobot_gazebo" / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import base_twist_utils as utils


class TestPlanarOdometryUtils(unittest.TestCase):
    def test_yaw_to_quaternion_components_for_pi(self) -> None:
        qx, qy, qz, qw = utils.yaw_to_quaternion_components(math.pi)
        self.assertTrue(math.isclose(qx, 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(qy, 0.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(abs(qz), 1.0, rel_tol=1e-6))
        self.assertTrue(math.isclose(qw, 0.0, abs_tol=1e-6))

    def test_extract_planar_joint_state_reads_pose_and_twist(self) -> None:
        state = utils.extract_planar_joint_state(
            [
                "root_x_axis_joint",
                "unused_joint",
                "root_y_axis_joint",
                "root_z_rotation_joint",
            ],
            [1.0, 99.0, 2.0, 0.5],
            [0.1, 0.0, 0.2, 0.3],
        )
        self.assertEqual(state, (1.0, 2.0, 0.5, 0.1, 0.2, 0.3))

    def test_extract_planar_joint_state_returns_none_when_joint_missing(self) -> None:
        state = utils.extract_planar_joint_state(
            ["root_x_axis_joint", "root_y_axis_joint"],
            [1.0, 2.0],
            [0.1, 0.2],
        )
        self.assertIsNone(state)


if __name__ == "__main__":
    unittest.main()
