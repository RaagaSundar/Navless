# /// script
# requires-python = ">=3.11"
# dependencies = ["pyulog==1.3.0"]
# ///
"""Data contract tests: registration must be evidence based; never truth fitted."""

import importlib.util, json, math, tempfile, unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "poses", Path(__file__).with_name("build_localization_poses.py")
)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class PosesTest(unittest.TestCase):
    def fixture(self, root):
        p = Path(root) / "test-run"
        f = p / "flight"
        f.mkdir(parents=True)
        (f / "mission.log").write_text("no registration evidence")
        (f / "planned-route.json").write_text(
            json.dumps({"launch_origin_enu": [-2, 1, 0]})
        )
        (p / "settings.json").write_text(json.dumps({"start_bearing_deg": 0}))
        (f / "track.json").write_text(
            json.dumps(
                {
                    "samples": [
                        {
                            "t": 1,
                            "x": 100,
                            "y": 200,
                            "z": 9,
                            "yaw": 0,
                            "armed": False,
                            "landed": True,
                        },
                        {
                            "t": 2,
                            "x": 101,
                            "y": 201,
                            "z": 10,
                            "yaw": 1,
                            "armed": False,
                            "landed": True,
                        },
                    ]
                }
            )
        )
        (f / "live-telemetry.jsonl").write_text(
            json.dumps(
                {
                    "kind": "local_position",
                    "timestamp": 1000000,
                    "timestamp_sample": 1000000,
                    "x": 3,
                    "y": 4,
                    "z": 5,
                    "heading": 0.6,
                    "xy_valid": True,
                    "z_valid": True,
                }
            )
            + "\n"
        )
        return p

    def test_unknown_registration_keeps_native_axes(self):
        with tempfile.TemporaryDirectory() as td:
            out = mod.build(self.fixture(td), 5)
            r = out["samples"][0]["sources"]["px4_fused"]
            self.assertEqual(r["xyz"], [3, 4, 5])
            self.assertEqual(r["frame"], "PX4 floating FRD")
            self.assertEqual(r["registration_status"], "unregistered")
            self.assertFalse(out["registration"]["truth_used_for_alignment"])

    def test_future_samples_never_appear_and_stale_is_explicit(self):
        with tempfile.TemporaryDirectory() as td:
            out = mod.build(self.fixture(td), 5)
            for sample in out["samples"]:
                for r in sample["sources"].values():
                    self.assertLessEqual(r["stamp_us"], sample["t"] * 1e6 + 1)
            self.assertEqual(
                out["samples"][-1]["sources"]["px4_fused"]["status"], "stale"
            )

    def test_sealed_uses_final_telemetry_not_mirror(self):
        with tempfile.TemporaryDirectory() as td:
            p = self.fixture(td)
            f = p / "flight"
            (f / "grade.json").write_text('{"passed":false}')
            final = {
                "kind": "vision_admitted",
                "timestamp": 1000000,
                "position": [7, 8, 9],
                "orientation_xyzw": [0, 0, 0, 1],
            }
            (f / "telemetry.jsonl").write_text(json.dumps(final) + "\n")
            out = mod.build(p, 5)
            self.assertEqual(out["audit_telemetry_source"], "telemetry.jsonl")
            self.assertEqual(
                out["samples"][0]["sources"]["admitted_vo"]["native_xyz"], [7, 8, 9]
            )

    def test_return_origin_withheld_until_frozen_manifest(self):
        with tempfile.TemporaryDirectory() as td:
            p = self.fixture(td)
            (p / "mission-leg").write_text("return-standalone")
            out = mod.build(p, 5)
            self.assertEqual(out["registration"]["status"], "partial_or_unregistered")
            self.assertIsNone(out["registration"]["return_world_from_odom"])

    def test_normal_runner_entrypoint_watch_is_passive_and_atomic(self):
        import subprocess

        with tempfile.TemporaryDirectory() as td:
            p = self.fixture(td)
            entry = Path(__file__).with_name("run_split_course.sh")
            proc = subprocess.run(
                [
                    "bash",
                    str(entry),
                    "--localization-only",
                    str(p),
                    "--watch",
                    "--interval",
                    "5",
                    "--max-updates",
                    "2",
                ],
                capture_output=True,
                text=True,
                timeout=45,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            out = json.loads((p / "localization-live.json").read_text())
            self.assertEqual(out["run_id"], "test-run")
            self.assertFalse(out["sealed"])
            self.assertEqual(list(p.glob("*.partial")), [])
            once = subprocess.run(
                ["bash", str(entry), "--localization-only", str(p), "--once"],
                capture_output=True,
                text=True,
                timeout=45,
            )
            self.assertEqual(once.returncode, 0, once.stderr)

    def test_public_watch_exits_on_sealed_grade_and_prevents_duplicate(self):
        import subprocess, fcntl

        with tempfile.TemporaryDirectory() as td:
            p = self.fixture(td)
            (p / "flight/grade.json").write_text('{"passed":false}')
            entry = Path(__file__).with_name("run_split_course.sh")
            done = subprocess.run(
                ["bash", str(entry), "--localization-only", str(p), "--watch"],
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(done.returncode, 0, done.stderr)
            state = json.loads((p / "localization-worker-state.json").read_text())
            self.assertEqual(state["status"], "sealed")
            self.assertEqual(state["updates"], 1)
            with (p / ".localization-worker.lock").open("w") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                duplicate = subprocess.run(
                    ["bash", str(entry), "--localization-only", str(p), "--watch"],
                    capture_output=True,
                    text=True,
                    timeout=30,
                )
                self.assertEqual(duplicate.returncode, 0)
                self.assertIn("no duplicate started", duplicate.stdout)

    def test_sealed_preserves_native_PX4_from_reader(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        import numpy as np

        with tempfile.TemporaryDirectory() as td:
            p = self.fixture(td)
            (p / "flight/grade.json").write_text('{"passed":false}')
            ulog = p / "px4/test.ulg"
            ulog.parent.mkdir()
            ulog.write_bytes(b"SYNTHETIC TEST ONLY: reader is mocked")
            stamps = np.arange(1000000, 2000001, 5000, dtype=np.uint64)
            n = len(stamps)
            data = {
                "timestamp": stamps,
                "timestamp_sample": stamps,
                "x": np.linspace(3, 4, n),
                "y": np.linspace(4, 5, n),
                "z": np.linspace(5, 6, n),
                "heading": np.zeros(n),
                "xy_valid": np.ones(n, dtype=np.uint8),
                "z_valid": np.ones(n, dtype=np.uint8),
            }
            fake = SimpleNamespace(get_dataset=lambda _: SimpleNamespace(data=data))
            with patch("pyulog.ULog", return_value=fake):
                out = mod.build(p, 20)
            lookup = {int(t): i for i, t in enumerate(stamps)}
            seen = set()
            for sample in out["samples"]:
                r = sample["sources"].get("px4_fused")
                if not r:
                    continue
                self.assertLessEqual(r["stamp_us"], sample["t"] * 1e6 + 1)
                i = lookup[r["stamp_us"]]
                seen.add(i)
                for key, value in zip(["x", "y", "z"], r["source_native_xyz"]):
                    self.assertEqual(value, float(data[key][i]))
            self.assertGreater(len(seen), 10)


if __name__ == "__main__":
    unittest.main()
