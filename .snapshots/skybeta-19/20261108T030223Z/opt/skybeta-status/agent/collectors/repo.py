"""repo: the skybeta checkout on this drone -- VERSION, commit, branch, dirty.

``--no-optional-locks`` keeps ``git status`` from trying to refresh the index:
the agent runs with ``ProtectHome=read-only`` and must never write the repo.
"""
import os

from . import Collector, CommandError, run_cmd

REPO_CANDIDATES = ("~/skybeta", "~/Downloads/skybeta")


def detect_repo(explicit=None):
    """``SKYBETA_REPO_DIR`` / --repo-dir if given, else ~/skybeta, else ~/Downloads/skybeta."""
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))
    for cand in REPO_CANDIDATES:
        path = os.path.expanduser(cand)
        if os.path.isfile(os.path.join(path, "host", "doctor.sh")):
            return path
    return os.path.expanduser(REPO_CANDIDATES[0])


def read_version_tag(repo):
    try:
        with open(os.path.join(repo, "VERSION"), "r", encoding="utf-8") as f:
            tag = "".join(f.read().split())
    except OSError:
        return None
    return tag or None


class RepoCollector(Collector):
    name = "repo"
    period_s = 60.0
    EMPTY = {"path": None, "present": False, "version_tag": None, "commit": None,
             "branch": None, "dirty": None}

    def collect(self):
        repo = self.ctx.repo_dir
        out = dict(self.EMPTY)
        out["path"] = repo
        out["present"] = os.path.isfile(os.path.join(repo, "host", "doctor.sh"))
        if not out["present"]:
            return out
        out["version_tag"] = read_version_tag(repo)
        git = ["git", "-C", repo, "--no-optional-locks"]
        try:
            out["commit"] = run_cmd(git + ["rev-parse", "--short", "HEAD"], 5.0)[1].strip() or None
            out["branch"] = run_cmd(git + ["rev-parse", "--abbrev-ref", "HEAD"], 5.0)[1].strip() or None
            status = run_cmd(git + ["status", "--porcelain"], 5.0)[1]
            out["dirty"] = bool(status.strip())
        except CommandError as e:
            # Not a git checkout (e.g. an unpacked tarball): report what we have.
            if "not a git repository" in str(e).lower():
                return out
            raise
        return out
