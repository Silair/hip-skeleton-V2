import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "fixed_link_diagnostics.py"


def load_tool():
    spec = importlib.util.spec_from_file_location("fixed_link_diagnostics", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


FIELDS = [
    "MonoTimeS",
    "AssistState",
    "MotionConfidence",
    "StopProbability",
    "PhaseSignalRad",
    "FilteredPhaseSignalRad",
    "SpreadDeg",
    "TorqueScale",
    "AllowOutput",
    "LeftJointPos",
    "RightJointPos",
    "LeftJointVel",
    "RightJointVel",
    "LeftRawMotorPos",
    "RightRawMotorPos",
    "LeftRawMotorVel",
    "RightRawMotorVel",
    "LeftTorqueCmd",
    "RightTorqueCmd",
]


def write_rows(path: Path, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        for row in rows:
            full = {field: "0" for field in FIELDS}
            full.update(row)
            writer.writerow(full)


class FixedLinkDiagnosticsTest(unittest.TestCase):
    def test_fixed_link_summary_flags_raw_motor_drift_and_spikes(self):
        tool = load_tool()
        rows = []
        for i in range(10):
            rows.append(
                {
                    "MonoTimeS": str(i * 0.02),
                    "AllowOutput": "0",
                    "LeftRawMotorPos": str(1.0 + i * 0.006),
                    "LeftRawMotorVel": "0.12" if i == 5 else "0.01",
                    "LeftTorqueCmd": "0",
                }
            )

        summary = tool.summarize_fixed_link(rows, side="left")

        self.assertEqual(summary["side"], "left")
        self.assertGreater(summary["raw_motor_pos_delta_rad"], 0.05)
        self.assertEqual(summary["position_assessment"], "FAIL")
        self.assertEqual(summary["velocity_assessment"], "WARN")
        self.assertIn("raw_motor_position_drift_over_0p05_rad", summary["findings"])
        self.assertIn("raw_motor_velocity_spike_over_0p1_rad_s", summary["findings"])

    def test_stop_response_flags_active_motion_after_stop_intent(self):
        tool = load_tool()
        rows = [
            {
                "MonoTimeS": "0.00",
                "AssistState": "3",
                "StopProbability": "0.1",
                "AllowOutput": "1",
                "LeftRawMotorVel": "0.04",
                "RightRawMotorVel": "0.03",
                "LeftTorqueCmd": "0.2",
                "RightTorqueCmd": "0",
            },
            {
                "MonoTimeS": "0.02",
                "AssistState": "3",
                "StopProbability": "0.92",
                "AllowOutput": "1",
                "LeftRawMotorVel": "0.24",
                "RightRawMotorVel": "0.21",
                "LeftTorqueCmd": "0.3",
                "RightTorqueCmd": "0",
            },
            {
                "MonoTimeS": "0.04",
                "AssistState": "4",
                "StopProbability": "0.95",
                "AllowOutput": "0",
                "LeftRawMotorVel": "0.16",
                "RightRawMotorVel": "0.02",
                "LeftTorqueCmd": "0",
                "RightTorqueCmd": "0",
            },
        ]

        summary = tool.summarize_stop_response(rows)

        self.assertEqual(summary["status"], "FAIL")
        self.assertEqual(summary["active_or_ramp_after_stop_intent_count"], 1)
        self.assertEqual(summary["moving_after_output_disabled_count"], 1)

    def test_compare_zeroed_runs_reports_spread_bias(self):
        tool = load_tool()
        zeroed = [
            {"MonoTimeS": "0", "SpreadDeg": "3", "MotionConfidence": "0.1", "StopProbability": "0.9"},
            {"MonoTimeS": "0.02", "SpreadDeg": "4", "MotionConfidence": "0.1", "StopProbability": "0.9"},
        ]
        unzeroed = [
            {"MonoTimeS": "0", "SpreadDeg": "28", "MotionConfidence": "0.7", "StopProbability": "0.2"},
            {"MonoTimeS": "0.02", "SpreadDeg": "30", "MotionConfidence": "0.8", "StopProbability": "0.2"},
        ]

        comparison = tool.compare_zeroing(zeroed, unzeroed)

        self.assertGreater(comparison["initial_spread_delta_deg"], 20.0)
        self.assertGreater(comparison["mean_motion_confidence_delta"], 0.5)
        self.assertIn("unzeroed_spread_bias_over_10_deg", comparison["findings"])

    def test_cli_writes_json_report(self):
        tool = load_tool()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            csv_path = tmp_path / "fixed.csv"
            out_path = tmp_path / "report.json"
            write_rows(
                csv_path,
                [
                    {"MonoTimeS": "0.00", "LeftRawMotorPos": "1.0", "LeftRawMotorVel": "0.0"},
                    {"MonoTimeS": "0.02", "LeftRawMotorPos": "1.004", "LeftRawMotorVel": "0.01"},
                ],
            )

            exit_code = tool.main([str(csv_path), "--side", "left", "--output", str(out_path)])

            self.assertEqual(exit_code, 0)
            self.assertTrue(out_path.exists())


if __name__ == "__main__":
    unittest.main()
