"""ros_summary: the sidecar's handoff file, /run/skybeta-status/ros_summary.json.

The sidecar (a container that joins the drone's loopback-only DDS) writes the
file atomically once a second; the agent only reads it. A missing file is not
an error -- it means "sidecar down" and the section says ``file_present:
false`` with nothing else (stale ROS data must not look current).
"""
import json
import os

from . import Collector

FILE_NAME = "ros_summary.json"


def read_summary(path):
    """-> dict with file_present / file_mtime plus the file's fields."""
    try:
        st = os.stat(path)
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
    except FileNotFoundError:
        return {"file_present": False, "file_mtime": None}
    doc = json.loads(text)  # a partial file cannot happen (os.replace); bad JSON is an error
    if not isinstance(doc, dict):
        raise ValueError("%s: not a JSON object" % path)
    out = {"file_present": True, "file_mtime": st.st_mtime}
    out.update(doc)
    return out


class RosSummaryCollector(Collector):
    name = "ros_summary"
    period_s = 1.0
    EMPTY = {"file_present": False, "file_mtime": None}

    def collect(self):
        return read_summary(os.path.join(self.ctx.rundir, FILE_NAME))
