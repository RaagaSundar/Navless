"""image: is ``skybeta:<tag>`` present, and what exactly is it?

The tag is the skybeta repo's VERSION file on this drone (the single source of
truth host/run.sh uses); the agent never guesses one. ``skybeta-team:<tag>``
is reported too, because run.sh prefers it when the team has a Dockerfile.
"""
import json

from . import Collector, CommandError, run_cmd
from .repo import read_version_tag

LABEL_KEYS = ("io.skybeta.px4-firmware", "io.skybeta.px4-msgs-ref", "io.skybeta.ros-snapshot",
              "io.skybeta.xrce-agent", "io.skybeta.isaac-ros", "io.skybeta.px4-ros2-lib-ref")


def short_id(image_id):
    """'sha256:b378bb9b7bae7e...' -> 'b378bb9b7bae' (12 hex, like doctor/drones.tsv)."""
    if not image_id:
        return None
    return image_id.split(":", 1)[-1][:12]


def inspect_image(ref, timeout=10.0):
    """``docker image inspect ref`` -> the first element, or None if absent."""
    rc, out, err = run_cmd(["docker", "image", "inspect", ref], timeout, check=False)
    if rc != 0:
        if "no such image" in (err + out).lower():
            return None
        raise CommandError("docker image inspect %s exited %d: %s" % (ref, rc, err.strip()[:200]))
    items = json.loads(out)
    return items[0] if items else None


def parse_tags(text, repo="skybeta"):
    """``docker image ls --format '{{.Repository}}:{{.Tag}}'`` -> tags of ``repo``."""
    tags = []
    for line in text.splitlines():
        name, _, tag = line.strip().rpartition(":")
        if name == repo and tag and tag != "<none>" and tag not in tags:
            tags.append(tag)
    return tags


def image_section(tag, base, team, tags):
    """Build the /status ``image`` section from inspect results (pure)."""
    out = {
        "expected_ref": ("skybeta:%s" % tag) if tag else None,
        "present": base is not None,
        "id": None, "id_short": None, "repo_digests": [], "created": None,
        "size_bytes": None, "labels": {},
        "team_image": {"ref": ("skybeta-team:%s" % tag) if tag else None, "present": team is not None,
                       "id": None, "base_id_label": None},
        "skybeta_tags_present": tags,
    }
    if base is not None:
        labels = (base.get("Config") or {}).get("Labels") or {}
        out.update({
            "id": base.get("Id"),
            "id_short": short_id(base.get("Id")),
            "repo_digests": list(base.get("RepoDigests") or []),
            "created": base.get("Created"),
            "size_bytes": base.get("Size"),
            "labels": {k: labels[k] for k in LABEL_KEYS if k in labels},
        })
    if team is not None:
        labels = (team.get("Config") or {}).get("Labels") or {}
        out["team_image"]["id"] = team.get("Id")
        out["team_image"]["base_id_label"] = labels.get("io.skybeta.base-id")
    return out


class ImageCollector(Collector):
    name = "image"
    period_s = 60.0
    EMPTY = image_section(None, None, None, [])

    def collect(self):
        tag = read_version_tag(self.ctx.repo_dir)
        base = inspect_image("skybeta:%s" % tag) if tag else None
        team = inspect_image("skybeta-team:%s" % tag) if tag else None
        _, ls, _ = run_cmd(["docker", "image", "ls", "--format", "{{.Repository}}:{{.Tag}}",
                            "--filter", "reference=skybeta"], 10.0)
        return image_section(tag, base, team, parse_tags(ls))
