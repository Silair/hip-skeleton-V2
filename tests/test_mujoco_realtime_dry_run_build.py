import pathlib
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MUJOCO_PY_ROOT = pathlib.Path("/home/lin/miniconda3/envs/mujoco/lib/python3.11/site-packages/mujoco")
AO_INCLUDE = pathlib.Path("/home/lin/code/2026.4.14hs_exoskeleton/include")
MODEL = pathlib.Path("/home/lin/exo_mujoco_sim/models/hip_exo_1dof.xml")


class MujocoRealtimeDryRunBuildTests(unittest.TestCase):
    def test_mujoco_realtime_dry_run_compiles(self):
        if not (MUJOCO_PY_ROOT / "include/mujoco/mujoco.h").exists():
            self.skipTest("MuJoCo headers are not available in the local conda environment")
        if not (MUJOCO_PY_ROOT / "libmujoco.so.3.9.0").exists():
            self.skipTest("MuJoCo shared library is not available in the local conda environment")

        with tempfile.TemporaryDirectory() as tmpdir:
            output = pathlib.Path(tmpdir) / "mujoco_realtime_dry_run"
            cmd = [
                "g++",
                "-std=c++17",
                "-I.",
                f"-I{AO_INCLUDE}",
                f"-I{MUJOCO_PY_ROOT / 'include'}",
                "tools/mujoco_realtime_dry_run.cpp",
                "control/GaitFeatureExtractor.cpp",
                "control/PhaseEstimator.cpp",
                "control/IntentDetector.cpp",
                "control/FreezeManager.cpp",
                "control/AssistStateMachine.cpp",
                "control/TorqueProfile.cpp",
                "control/StopDetector.cpp",
                "control/StopTorqueLimiter.cpp",
                "logging/ExoLogger.cpp",
                "logging/Clock.cpp",
                str(MUJOCO_PY_ROOT / "libmujoco.so.3.9.0"),
                "-Wl,-rpath," + str(MUJOCO_PY_ROOT),
                "-o",
                str(output),
            ]

            subprocess.run(cmd, cwd=ROOT, check=True)
            completed = subprocess.run([str(output), "--help"], cwd=ROOT, text=True, capture_output=True, check=True)
            self.assertIn("MuJoCo realtime dry-run", completed.stdout)
            self.assertIn("--apply-v2-torque-scale", completed.stdout)
            self.assertIn("--driver-scenario", completed.stdout)
            self.assertIn("--stop-start-s", completed.stdout)
            self.assertIn("--stop-duration-s", completed.stdout)

            if not MODEL.exists():
                self.skipTest("local MuJoCo 1DOF model is not available")
            run_dir = pathlib.Path(tmpdir) / "logs"
            run = subprocess.run(
                [
                    str(output),
                    "--mjcf",
                    str(MODEL),
                    "--output-dir",
                    str(run_dir),
                    "--prefix",
                    "test_closed_loop",
                    "--duration-s",
                    "0.05",
                    "--apply-v2-torque-scale",
                    "0.05",
                    "--driver-scenario",
                    "stop_go",
                    "--stop-start-s",
                    "0.02",
                    "--stop-duration-s",
                    "0.02",
                ],
                cwd=ROOT,
                text=True,
                capture_output=True,
                check=True,
            )
            self.assertIn("V2 torque scale applied to MuJoCo exo actuator: 0.05", run.stdout)


if __name__ == "__main__":
    unittest.main()
