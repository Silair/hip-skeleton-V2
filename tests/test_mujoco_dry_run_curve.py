import importlib.util
import math
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "mujoco_dry_run_curve.py"
spec = importlib.util.spec_from_file_location("mujoco_dry_run_curve", SCRIPT)
mujoco_dry_run_curve = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mujoco_dry_run_curve)


class MujocoDryRunCurveTests(unittest.TestCase):
    def test_compute_pd_torque_tracks_sine_and_clips(self):
        torque = mujoco_dry_run_curve.compute_pd_torque(
            qpos=0.0,
            qvel=0.0,
            time_s=0.0,
            amplitude_rad=math.radians(25.0),
            frequency_hz=0.8,
            kp=35.0,
            kd=4.0,
            torque_limit=3.0,
        )

        self.assertAlmostEqual(torque, 3.0)

    def test_replay_row_zero_virtual_right_uses_single_leg_only(self):
        row = mujoco_dry_run_curve.make_replay_row(
            time_s=1.25,
            qpos=0.2,
            qvel=-0.4,
            virtual_right="zero",
        )

        self.assertEqual(row["TimeS"], "1.250000000")
        self.assertEqual(row["LeftJointPosRad"], "0.2")
        self.assertEqual(row["RightJointPosRad"], "0")
        self.assertEqual(row["LeftJointVelRadS"], "-0.4")
        self.assertEqual(row["RightJointVelRadS"], "0")
        self.assertEqual(row["Healthy"], "1")
        self.assertEqual(row["Enabled"], "1")

    def test_replay_row_copy_virtual_right_preserves_phase_signal(self):
        row = mujoco_dry_run_curve.make_replay_row(
            time_s=0.0,
            qpos=0.2,
            qvel=0.4,
            virtual_right="copy",
        )

        self.assertEqual(row["LeftJointPosRad"], "0.2")
        self.assertEqual(row["RightJointPosRad"], "0.2")
        self.assertEqual(row["LeftJointVelRadS"], "0.4")
        self.assertEqual(row["RightJointVelRadS"], "0.4")


if __name__ == "__main__":
    unittest.main()
