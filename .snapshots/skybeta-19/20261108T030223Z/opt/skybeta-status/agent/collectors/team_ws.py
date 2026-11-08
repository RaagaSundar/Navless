"""team_ws: the team's workspace on the host (bind-mounted at /root/team_ws).

Same notion of "foreign-owned" as doctor.sh's team-ws check: any entry not
owned by the agent's uid (created as root inside a container and never chowned
back) makes the workspace uneditable from the host shell.
"""
import os
import re

from . import Collector, CommandError, run_cmd

PRUNE = {"build", "install", "log", ".git"}
MAX_ENTRIES = 20000
MAX_NAMES = 50
NAME_RE = re.compile(r"<name>\s*([^<\s]+)\s*</name>")


def scan_team_ws(path, uid=None):
    """Pure-ish scan of ``path`` (filesystem only, no subprocess)."""
    out = {"path": path, "exists": os.path.isdir(path), "packages": 0, "package_names": [],
           "overlay_built": False, "dockerfile": False, "foreign_owned_files": False,
           "src_last_modified": None, "src_file_count": 0, "truncated": False}
    if not out["exists"]:
        return out
    uid = os.getuid() if (uid is None and hasattr(os, "getuid")) else uid
    out["overlay_built"] = os.path.isfile(os.path.join(path, "install", "local_setup.bash"))
    out["dockerfile"] = os.path.isfile(os.path.join(path, "Dockerfile"))
    # Foreign-owned check covers the whole workspace (doctor's `find TEAM_WS ! -user`),
    # bounded like the src walk.
    seen = 0
    if uid is not None:
        for root, dirs, files in os.walk(path):
            for name in dirs + files:
                seen += 1
                try:
                    if os.lstat(os.path.join(root, name)).st_uid != uid:
                        out["foreign_owned_files"] = True
                        break
                except OSError:
                    continue
            if out["foreign_owned_files"] or seen > MAX_ENTRIES:
                break
    src = os.path.join(path, "src")
    count = 0
    newest = None
    for root, dirs, files in os.walk(src):
        dirs[:] = [d for d in dirs if d not in PRUNE]
        for name in files:
            count += 1
            full = os.path.join(root, name)
            try:
                mtime = os.lstat(full).st_mtime
            except OSError:
                continue
            newest = mtime if newest is None or mtime > newest else newest
            if name == "package.xml":
                out["packages"] += 1
                if len(out["package_names"]) < MAX_NAMES:
                    try:
                        with open(full, "r", encoding="utf-8", errors="replace") as f:
                            m = NAME_RE.search(f.read(65536))
                        if m:
                            out["package_names"].append(m.group(1))
                    except OSError:
                        pass
        if count > MAX_ENTRIES:
            out["truncated"] = True
            break
    out["src_file_count"] = count
    out["src_last_modified"] = newest
    out["package_names"].sort()
    return out


def git_info(path):
    if not os.path.exists(os.path.join(path, ".git")):
        return None
    git = ["git", "-C", path, "--no-optional-locks"]
    try:
        commit = run_cmd(git + ["rev-parse", "--short", "HEAD"], 5.0)[1].strip() or None
        branch = run_cmd(git + ["rev-parse", "--abbrev-ref", "HEAD"], 5.0)[1].strip() or None
        status = run_cmd(git + ["status", "--porcelain"], 5.0)[1]
    except CommandError:
        return {"commit": None, "branch": None, "dirty_files": None}
    return {"commit": commit, "branch": branch,
            "dirty_files": sum(1 for line in status.splitlines() if line.strip())}


class TeamWsCollector(Collector):
    name = "team_ws"
    period_s = 60.0
    EMPTY = dict(scan_team_ws("/nonexistent-team-ws"), path=None, git=None)

    def collect(self):
        ident = self.ctx.get("identity") or {}
        ws = (ident.get("drone_env") or {}).get("team_ws")
        if not ws:
            from .identity import home_dir  # noqa: PLC0415
            ws = os.path.join(home_dir(), "team_ws")
        out = scan_team_ws(ws)
        out["git"] = git_info(ws) if out["exists"] else None
        return out
