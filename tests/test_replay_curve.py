import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "make_replay_curve.py"


spec = importlib.util.spec_from_file_location("make_replay_curve", SCRIPT)
make_replay_curve = importlib.util.module_from_spec(spec)
spec.loader.exec_module(make_replay_curve)


class ReplayCurveGenerationTest(unittest.TestCase):
    def read_rows(self, path):
        with path.open(newline="", encoding="utf-8") as f:
            return list(csv.DictReader(f))

    def test_sine_curve_has_required_replay_columns_and_signal(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "sine.csv"
            rc = make_replay_curve.main([
                "--output", str(output),
                "--scenario", "sine",
                "--duration-s", "1.0",
                "--rate-hz", "10",
                "--amplitude-rad", "0.4",
                "--frequency-hz", "1.0",
            ])
            self.assertEqual(rc, 0)
            rows = self.read_rows(output)
            self.assertEqual(len(rows), 11)
            self.assertEqual(rows[0]["TimeS"], "0.000000000")
            self.assertIn("LeftJointVelRadS", rows[0])
            # At t=0.25s, left + right should reconstruct the designed scalar signal.
            quarter = rows[2]
            signal = float(quarter["LeftJointPosRad"]) + float(quarter["RightJointPosRad"])
            self.assertGreater(signal, 0.35)

    def test_custom_expressions_let_user_design_curve_shape(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "custom.csv"
            make_replay_curve.main([
                "--output", str(output),
                "--scenario", "custom",
                "--duration-s", "0.5",
                "--rate-hz", "4",
                "--left-expr", "0.1 + 0.2*sin(tau*t)",
                "--right-expr", "-0.05 + 0.1*cos(tau*t)",
            ])
            rows = self.read_rows(output)
            self.assertEqual(len(rows), 3)
            self.assertAlmostEqual(float(rows[0]["LeftJointPosRad"]), 0.1, places=9)
            self.assertAlmostEqual(float(rows[0]["RightJointPosRad"]), 0.05, places=9)


    def test_abrupt_stop_holds_nonzero_pose_after_stop_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "abrupt_stop.csv"
            make_replay_curve.main([
                "--output", str(output),
                "--scenario", "abrupt_stop",
                "--duration-s", "4.0",
                "--rate-hz", "10",
                "--amplitude-rad", "0.4",
                "--frequency-hz", "0.5",
                "--stop-time-s", "1.25",
            ])
            rows = self.read_rows(output)
            stopped_rows = [r for r in rows if float(r["TimeS"]) >= 1.5]
            self.assertGreater(len(stopped_rows), 5)
            held = [float(r["LeftJointPosRad"]) + float(r["RightJointPosRad"]) for r in stopped_rows]
            self.assertGreater(abs(held[0]), 0.15)
            self.assertLess(max(held) - min(held), 1e-9)
            self.assertTrue(all(abs(float(r["LeftJointVelRadS"])) < 1e-9 for r in stopped_rows[1:]))
            self.assertTrue(all(abs(float(r["RightJointVelRadS"])) < 1e-9 for r in stopped_rows[1:]))

    def test_amplitude_ramp_grows_then_shrinks_signal_envelope(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "amplitude_ramp.csv"
            make_replay_curve.main([
                "--output", str(output),
                "--scenario", "amplitude_ramp",
                "--duration-s", "6.0",
                "--rate-hz", "20",
                "--amplitude-rad", "0.4",
                "--min-amplitude-rad", "0.08",
                "--frequency-hz", "0.5",
            ])
            rows = self.read_rows(output)
            first = max(abs(float(r["LeftJointPosRad"]) + float(r["RightJointPosRad"])) for r in rows[:40])
            middle = max(abs(float(r["LeftJointPosRad"]) + float(r["RightJointPosRad"])) for r in rows[45:80])
            last = max(abs(float(r["LeftJointPosRad"]) + float(r["RightJointPosRad"])) for r in rows[-40:])
            self.assertGreater(middle, first)
            self.assertGreater(middle, last)

    def test_asymmetric_curve_keeps_distinct_left_and_right_shapes(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "asymmetric.csv"
            make_replay_curve.main([
                "--output", str(output),
                "--scenario", "asymmetric",
                "--duration-s", "1.0",
                "--rate-hz", "20",
                "--amplitude-rad", "0.4",
                "--frequency-hz", "1.0",
            ])
            rows = self.read_rows(output)
            diffs = [abs(float(r["LeftJointPosRad"]) - float(r["RightJointPosRad"])) for r in rows]
            self.assertGreater(max(diffs), 0.02)


if __name__ == "__main__":
    unittest.main()
