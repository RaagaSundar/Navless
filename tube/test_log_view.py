"""log_view.py: one CSV row per tof_driver --view frame, as the frames stream past.

  python3 -m pytest tube/test_log_view.py      (or: cd tube && python3 test_log_view.py)
"""
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from tof_driver import view_text  # noqa: E402


def rows(csv_path):
    return [line.strip() for line in open(csv_path) if not line.startswith("#")]


def run(tmp_dir):
    out = os.path.join(tmp_dir, "view.csv")
    frames = view_text(0.183, 0.213, 9.6) + view_text(0.6, math.inf, 9.6) + view_text(0.15, 0.6, 9.6)
    shown = subprocess.run([sys.executable, os.path.join(HERE, "log_view.py"), out], input=frames.encode(),
                           capture_output=True, check=True).stdout.decode().replace("\r\n", "\n")
    assert shown == frames                       # the view passes through unchanged (Windows adds \r)
    return [r.split(",")[1:] for r in rows(out)]


def test_one_row_per_complete_frame(tmp_path):
    # a frame is complete once the next one starts: three frames in, two rows out
    assert run(str(tmp_path)) == [["18.3", "21.3"], ["60.0", ""]]


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as d:
        assert run(d) == [["18.3", "21.3"], ["60.0", ""]]
    print("ok")
