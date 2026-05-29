import csv
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "analyze_run.py"


def load_analyzer():
    spec = importlib.util.spec_from_file_location("analyze_run", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class AnalyzeRunTest(unittest.TestCase):
    def write_synthetic_log(self, path: Path, frequency_hz: float = 0.8):
        fields = [
            "SyncSessionId", "StreamId", "LoopSeq", "EpochMs", "MonoTimeS", "DtS", "Healthy",
            "AssistState", "FreezeState", "MotionConfidence", "StopProbability", "Phase", "PhaseValid",
            "AnchorDetected", "Frequency", "Amplitude", "AoSignalEstimateRad", "AoSignalErrorRad", "PhaseSignalRad", "FilteredPhaseSignalRad",
            "SpreadDeg", "PhaseVelocityDegS", "SignedPhaseVelocityDegS", "FreezeRequested",
            "PhaseTrackingEnabled", "RecoveryActive", "TorqueScale", "AllowOutput", "LeftJointPos",
            "RightJointPos", "LeftJointVel", "RightJointVel", "LeftTorqueCmd", "RightTorqueCmd",
        ]
        dt = 0.02
        lead = 0.20
        with path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for i in range(500):
                t = i * dt
                phase = (2.0 * math.pi * frequency_hz * t) % (2.0 * math.pi)
                signal = math.sin(phase)
                signed_vel = 2.0 * math.pi * frequency_hz * math.cos(phase)
                right = 3.0 * max(0.0, math.cos(phase + lead))
                left = 3.0 * max(0.0, -math.cos(phase + lead))
                writer.writerow({
                    "SyncSessionId": "test", "StreamId": "synthetic", "LoopSeq": i + 1,
                    "EpochMs": 1000 + int(t * 1000), "MonoTimeS": t, "DtS": dt, "Healthy": 1,
                    "AssistState": 3, "FreezeState": 0, "MotionConfidence": 0.9, "StopProbability": 0.1,
                    "Phase": phase, "PhaseValid": 1, "AnchorDetected": 0, "Frequency": frequency_hz,
                    "Amplitude": 1.0, "AoSignalEstimateRad": signal, "AoSignalErrorRad": 0.0,
                    "PhaseSignalRad": signal, "FilteredPhaseSignalRad": signal,
                    "SpreadDeg": abs(signal) * 57.2957795130823,
                    "PhaseVelocityDegS": abs(signed_vel) * 57.2957795130823,
                    "SignedPhaseVelocityDegS": signed_vel * 57.2957795130823,
                    "FreezeRequested": 0, "PhaseTrackingEnabled": 1, "RecoveryActive": 0,
                    "TorqueScale": 1.0, "AllowOutput": 1, "LeftJointPos": signal / 2.0,
                    "RightJointPos": signal / 2.0, "LeftJointVel": signed_vel / 2.0,
                    "RightJointVel": signed_vel / 2.0, "LeftTorqueCmd": left, "RightTorqueCmd": right,
                })

    def test_analyzer_computes_metrics_and_writes_reports(self):
        analyzer = load_analyzer()
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            log_path = tmp_path / "run.csv"
            self.write_synthetic_log(log_path)
            result = analyzer.analyze_log(log_path, output_dir=tmp_path / "report")

            metrics = result["metrics"]
            self.assertGreater(metrics["sample_count"], 400)
            self.assertLess(metrics["phase_rmse_percent"], 2.0)
            self.assertLess(metrics["frequency_rmse_hz"], 0.25)
            self.assertLessEqual(metrics["max_abs_torque_nm"], 3.01)
            self.assertLess(metrics["ao_reconstruction_rmse_rad"], 1e-9)
            self.assertEqual(metrics["freeze_torque_violation_count"], 0)
            self.assertTrue((tmp_path / "report" / "metrics.json").exists())
            html_path = tmp_path / "report" / "index.html"
            self.assertTrue(html_path.exists())
            html = html_path.read_text(encoding="utf-8")
            self.assertIn("Phase RMSE", html)
            self.assertIn("控制效果判定", html)

            evaluation_path = tmp_path / "report" / "evaluation.json"
            self.assertTrue(evaluation_path.exists())
            evaluation = json.loads(evaluation_path.read_text(encoding="utf-8"))
            self.assertEqual(evaluation["status"], "PASS")
            self.assertEqual(evaluation["groups"]["safety_gating"]["status"], "PASS")
            self.assertEqual(result["evaluation"]["status"], "PASS")

    def test_evaluator_fails_safety_and_torque_violations(self):
        analyzer = load_analyzer()
        metrics = {
            "reference_event_count": 8,
            "phase_rmse_percent": 2.0,
            "phase_abs_error_p95_percent": 4.0,
            "frequency_rmse_hz": 0.03,
            "ao_reconstruction_rmse_rad": 0.01,
            "convergence_time_s_at_5_percent": 1.0,
            "max_abs_torque_nm": 9.5,
            "max_torque_rate_nm_s": 30.0,
            "freeze_torque_violation_count": 1,
            "disallow_output_torque_violation_count": 0,
            "stationary_false_assist_s": 0.0,
            "simultaneous_high_torque_ratio": 0.0,
            "peak_torque_phase_mae_deg": 8.0,
        }

        evaluation = analyzer.evaluate_metrics(metrics)

        self.assertEqual(evaluation["status"], "FAIL")
        failed_keys = {check["key"] for check in evaluation["failed_checks"]}
        self.assertIn("max_abs_torque_nm", failed_keys)
        self.assertIn("freeze_torque_violation_count", failed_keys)
        self.assertEqual(evaluation["groups"]["safety_gating"]["status"], "FAIL")

    def test_evaluator_skips_conditional_torque_phase_when_no_torque(self):
        analyzer = load_analyzer()
        metrics = {
            "reference_event_count": 8,
            "phase_rmse_percent": 2.0,
            "phase_abs_error_p95_percent": 4.0,
            "frequency_rmse_hz": 0.03,
            "ao_reconstruction_rmse_rad": 0.01,
            "convergence_time_s_at_5_percent": 1.0,
            "max_abs_torque_nm": 0.0,
            "max_torque_rate_nm_s": 0.0,
            "freeze_torque_violation_count": 0,
            "disallow_output_torque_violation_count": 0,
            "stationary_false_assist_s": 0.0,
            "simultaneous_high_torque_ratio": 0.0,
            "peak_torque_phase_mae_deg": None,
        }

        evaluation = analyzer.evaluate_metrics(metrics)

        self.assertEqual(evaluation["status"], "PASS")
        peak_checks = [check for check in evaluation["groups"]["torque_assist"]["checks"] if check["key"] == "peak_torque_phase_mae_deg"]
        self.assertEqual(peak_checks[0]["status"], "SKIP")


    def test_stopping_torque_is_excluded_from_peak_phase_and_reported_as_residual(self):
        analyzer = load_analyzer()
        rows = []
        dt = 0.02
        lead = 0.20
        for i in range(80):
            t = i * dt
            phase = (2.0 * math.pi * 0.8 * t) % (2.0 * math.pi)
            signal = math.sin(phase)
            signed_vel = 2.0 * math.pi * 0.8 * math.cos(phase)
            state = 3 if i < 50 else 4
            stop_prob = 0.1 if i < 50 else 0.95
            right = 3.0 * max(0.0, math.cos(phase + lead)) if i < 50 else 1.5
            left = 3.0 * max(0.0, -math.cos(phase + lead)) if i < 50 else 0.0
            if i >= 50:
                # Deliberately hold a residual Stopping torque at a bad phase; it should
                # be reported as residual stop torque, not normal gait peak timing error.
                phase = math.pi
            rows.append({
                "MonoTimeS": str(t), "DtS": str(dt), "AssistState": str(state),
                "StopProbability": str(stop_prob), "Phase": str(phase), "Frequency": "0.8",
                "FilteredPhaseSignalRad": str(signal), "SignedPhaseVelocityDegS": str(signed_vel * 57.2957795130823),
                "SpreadDeg": str(abs(signal) * 57.2957795130823), "AoSignalErrorRad": "0",
                "FreezeRequested": "0", "AllowOutput": "1", "LeftTorqueCmd": str(left), "RightTorqueCmd": str(right),
            })

        metrics, _, _, _ = analyzer.compute_metrics(rows)

        self.assertLess(metrics["peak_torque_phase_mae_deg"], 15.0)
        self.assertGreater(metrics["post_stop_peak_torque_nm"], 1.0)
        self.assertGreater(metrics["post_stop_peak_torque_ratio"], 0.4)


    def test_phase_metrics_ignore_stopping_gap_between_walk_segments(self):
        analyzer = load_analyzer()
        rows = []
        dt = 0.02
        frequency = 0.8
        total = 12.0
        for i in range(int(total / dt)):
            t = i * dt
            walking = t < 4.0 or t >= 8.0
            walk_t = t if t < 4.0 else max(0.0, t - 4.0)
            phase = (2.0 * math.pi * frequency * walk_t) % (2.0 * math.pi)
            signal = math.sin(phase) if walking else 0.0
            signed_vel = 2.0 * math.pi * frequency * math.cos(phase) if walking else 0.0
            if walking:
                assist_state = 3
                stop_probability = 0.1
                measured_phase = phase
                measured_frequency = frequency
            else:
                assist_state = 4
                stop_probability = 0.95
                # Deliberately bad held phase/frequency during the stop gap. These
                # must not pollute walking-only phase/frequency metrics.
                measured_phase = math.pi
                measured_frequency = 0.0
            rows.append({
                "MonoTimeS": str(t), "DtS": str(dt), "AssistState": str(assist_state),
                "StopProbability": str(stop_probability), "Phase": str(measured_phase), "Frequency": str(frequency if walking else measured_frequency),
                "FilteredPhaseSignalRad": str(signal), "SignedPhaseVelocityDegS": str(signed_vel * 57.2957795130823),
                "SpreadDeg": str(abs(signal) * 57.2957795130823), "AoSignalErrorRad": "0",
                "FreezeRequested": "0", "AllowOutput": "1", "LeftTorqueCmd": "0", "RightTorqueCmd": "0",
            })

        metrics, _, _, _ = analyzer.compute_metrics(rows)

        self.assertGreater(metrics["reference_event_count"], 8)
        self.assertLess(metrics["phase_rmse_percent"], 8.0)
        self.assertLess(metrics["frequency_rmse_hz"], 0.25)
        self.assertGreater(metrics["phase_eval_sample_count"], 100)
        self.assertLess(metrics["phase_eval_sample_count"], len(rows))


    def test_frequency_transition_adaptation_metrics_report_relock_time(self):
        analyzer = load_analyzer()
        rows = []
        dt = 0.02
        transition_t = 3.0
        for i in range(int(7.0 / dt)):
            t = i * dt
            ref_freq = 0.6 if t < transition_t else 1.2
            # Keep the biomechanical signal at the new true frequency after transition,
            # but simulate AO taking 0.40 s to relock its reported phase/frequency.
            phase_true = (2.0 * math.pi * (0.6 * min(t, transition_t) + 1.2 * max(0.0, t - transition_t))) % (2.0 * math.pi)
            adapting = transition_t <= t < transition_t + 0.40
            measured_freq = 0.75 if adapting else ref_freq
            measured_phase = (phase_true + (0.35 if adapting else 0.0)) % (2.0 * math.pi)
            signal = math.sin(phase_true)
            signed_vel = 2.0 * math.pi * ref_freq * math.cos(phase_true)
            rows.append({
                "MonoTimeS": str(t), "DtS": str(dt), "AssistState": "3",
                "StopProbability": "0.1", "Phase": str(measured_phase), "Frequency": str(measured_freq),
                "FilteredPhaseSignalRad": str(signal), "SignedPhaseVelocityDegS": str(signed_vel * 57.2957795130823),
                "SpreadDeg": str(abs(signal) * 57.2957795130823), "AoSignalErrorRad": "0",
                "FreezeRequested": "0", "AllowOutput": "1", "LeftTorqueCmd": "0", "RightTorqueCmd": "2",
            })

        metrics, _, _, _ = analyzer.compute_metrics(rows)

        self.assertGreaterEqual(metrics["frequency_transition_count"], 1)
        self.assertIsNotNone(metrics["frequency_adaptation_time_mean_s"])
        self.assertGreater(metrics["frequency_adaptation_time_mean_s"], 0.2)
        self.assertLess(metrics["frequency_adaptation_time_mean_s"], 0.8)
        self.assertGreater(metrics["phase_adaptation_time_mean_s"], 0.2)
        self.assertLess(metrics["phase_adaptation_time_mean_s"], 0.8)
        self.assertTrue(metrics["frequency_transition_windows"])

    def test_anchor_update_metrics_count_runtime_fields(self):
        analyzer = load_analyzer()
        rows = [
            {
                "MonoTimeS": "0.00", "DtS": "0.02", "AssistState": "3", "StopProbability": "0.1",
                "Phase": "0", "Frequency": "0.8", "FilteredPhaseSignalRad": "0",
                "SignedPhaseVelocityDegS": "20", "SpreadDeg": "20", "AoSignalErrorRad": "0",
                "FreezeRequested": "0", "AllowOutput": "1", "LeftTorqueCmd": "0", "RightTorqueCmd": "0",
                "AnchorFrequencyUpdated": "1", "AnchorRejected": "0",
                "AnchorMeasuredFrequencyHz": "0.9", "OmegaCorrectionHz": "0.10",
            },
            {
                "MonoTimeS": "0.02", "DtS": "0.02", "AssistState": "4", "StopProbability": "0.95",
                "Phase": "0", "Frequency": "0.8", "FilteredPhaseSignalRad": "0",
                "SignedPhaseVelocityDegS": "-20", "SpreadDeg": "20", "AoSignalErrorRad": "0",
                "FreezeRequested": "0", "AllowOutput": "1", "LeftTorqueCmd": "0", "RightTorqueCmd": "0",
                "AnchorFrequencyUpdated": "0", "AnchorRejected": "1",
                "AnchorMeasuredFrequencyHz": "0.0", "OmegaCorrectionHz": "0.00",
            },
        ]

        metrics, _, _, _ = analyzer.compute_metrics(rows)

        self.assertEqual(metrics["anchor_update_count"], 1)
        self.assertEqual(metrics["rejected_anchor_count"], 1)
        self.assertEqual(metrics["false_anchor_during_stop_count"], 1)
        self.assertAlmostEqual(metrics["omega_jump_p95_hz"], 0.10)


if __name__ == "__main__":
    unittest.main()
