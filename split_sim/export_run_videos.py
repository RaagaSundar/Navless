"""Export sealed flight evidence on the Mac, outside the SITL launcher."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def export(run):
    track = json.loads((run / "flight/track.json").read_text())
    if not any(s.get("armed") and s.get("landed") is False and s.get("z", 0) > .25
               for s in track["samples"]):
        print("No recorded flight above 25 cm")
        return
    with (run / ".export-videos.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return
        failures = []
        if not (run / "chase-view.json").is_file():
            chase = HERE / "preview/render_chase.py"
            if subprocess.run(["uv", "run", "--python", "3.12", str(chase), str(run)]).returncode:
                failures.append("chase render failed")
        if not (run / "drone-view.json").is_file():
            # The passive sensor recorder can finish sealing after the PX4 grade.
            capture = run / "camera-imu-capture.json"
            deadline = time.monotonic() + 600
            while not capture.exists() and time.monotonic() < deadline:
                time.sleep(3)
            if capture.exists():
                manifest = json.loads(capture.read_text())
                database = Path(manifest["path"])
                if not database.is_absolute():
                    database = run / database
                if manifest.get("integrity") != "ok":
                    failures.append("sensor recording integrity check missing or failed")
                elif subprocess.run([sys.executable, str(HERE / "export_drone_video.py"),
                                     "--database", str(database), "--run-dir", str(run)]).returncode:
                    failures.append("onboard export failed")
            else:
                failures.append("no sealed sensor recording available")
        if failures:
            raise SystemExit(", ".join(failures))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--enqueue", action="store_true")
    args = parser.parse_args()
    run_dir = args.run.resolve(strict=True)
    if args.enqueue:
        # The parent owns the domain flight lock. The child inherits no such fd.
        with (run_dir / "video-export.log").open("a") as log:
            subprocess.Popen([sys.executable, "-u", __file__, str(run_dir)],
                             stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                             start_new_session=True, close_fds=True)
    else:
        os.nice(10)
        export(run_dir)
