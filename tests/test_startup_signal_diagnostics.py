#!/usr/bin/env python3
import importlib.util
import unittest
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "tools" / "startup_signal_diagnostics.py"
    spec = importlib.util.spec_from_file_location("startup_signal_diagnostics", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class StartupSignalDiagnosticsTest(unittest.TestCase):
    def test_flags_low_spread_and_velocity(self):
        mod = load_module()
        rows = [
            {
                "MonoTimeS": "0.00",
                "SpreadDeg": "2.0",
                "SignedPhaseVelocityDegS": "3.0",
                "PhaseSignalRad": "0.01",
                "FilteredPhaseSignalRad": "0.01",
                "AnchorCandidate": "0",
                "AnchorDetected": "0",
            },
            {
                "MonoTimeS": "0.02",
                "SpreadDeg": "2.5",
                "SignedPhaseVelocityDegS": "-2.0",
                "PhaseSignalRad": "-0.01",
                "FilteredPhaseSignalRad": "-0.01",
                "AnchorCandidate": "0",
                "AnchorDetected": "0",
            },
        ]
        summary = mod.startup_signal_diagnostics(rows, start_s=0.0, end_s=4.0, target_frequency_hz=0.6)
        self.assertTrue(summary["spread_below_threshold"])
        self.assertTrue(summary["velocity_below_threshold"])
        self.assertIn("amplitude_low_spread_below_threshold", summary["likely_causes"])
        self.assertIn("velocity_low_below_threshold", summary["likely_causes"])


if __name__ == "__main__":
    unittest.main()
