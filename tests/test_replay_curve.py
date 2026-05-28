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


if __name__ == "__main__":
    unittest.main()
