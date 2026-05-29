import importlib.util
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "compare_metrics.py"


def load_compare_metrics():
    spec = importlib.util.spec_from_file_location("compare_metrics", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CompareMetricsTest(unittest.TestCase):
    def test_compare_before_after_reports_tracking_and_adaptation_deltas(self):
        comparer = load_compare_metrics()
        before = {
            "phase_rmse_percent": 10.0,
            "frequency_rmse_hz": 0.25,
            "frequency_adaptation_time_mean_s": 0.60,
            "combined_adaptation_time_max_s": 0.80,
        }
        after = {
            "phase_rmse_percent": 7.0,
            "frequency_rmse_hz": 0.15,
            "frequency_adaptation_time_mean_s": 0.35,
            "combined_adaptation_time_max_s": 0.50,
        }

        summary = comparer.compare_metric_sets(before, after)

        self.assertEqual(summary["status"], "IMPROVED")
        rows = {row["key"]: row for row in summary["metrics"]}
        self.assertAlmostEqual(rows["phase_rmse_percent"]["delta"], -3.0)
        self.assertAlmostEqual(rows["frequency_adaptation_time_mean_s"]["relative_change_percent"], -41.6666666667)
        self.assertEqual(rows["frequency_rmse_hz"]["direction"], "lower_is_better")

    def test_cli_writes_json_summary(self):
        comparer = load_compare_metrics()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            before_path = tmp_path / "before.json"
            after_path = tmp_path / "after.json"
            out_path = tmp_path / "comparison.json"
            before_path.write_text(json.dumps({"phase_rmse_percent": 10.0}), encoding="utf-8")
            after_path.write_text(json.dumps({"phase_rmse_percent": 8.0}), encoding="utf-8")

            with redirect_stdout(StringIO()):
                rc = comparer.main([str(before_path), str(after_path), "--json-output", str(out_path)])

            self.assertEqual(rc, 0)
            written = json.loads(out_path.read_text(encoding="utf-8"))
            self.assertEqual(written["status"], "IMPROVED")


if __name__ == "__main__":
    unittest.main()
