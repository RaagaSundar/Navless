"""Builds the /status document (schema_version 1, docs/status_schema.md).

Called per request: it is a dict merge plus age arithmetic under each
collector's lock, so /status always shows the newest snapshots and there is no
separate builder thread. Section ``age_s`` = monotonic now - the collector's
last success, so a wall-clock jump on the drone cannot fake freshness. The one
exception is ``ros.age_s``: ``time.time() - written_at`` (the sidecar is a
different process; both clocks are this host's wall clock).
"""
import time

SCHEMA_VERSION = 1

TOP_LEVEL_KEYS = ("schema_version", "generated_at", "agent", "identity", "repo", "image",
                  "docker", "stack", "sidecar", "ros", "doctor", "system", "team_ws")
META_KEYS = ("updated_at", "age_s", "error")

#: Keys every section must carry (besides META_KEYS). test_status_doc checks the
#: fixtures and a live build against this, and that each key is named in
#: docs/status_schema.md, so code, fixtures and doc cannot drift apart silently.
SECTION_KEYS = {
    "agent": ("version", "pid", "python", "started_at", "uptime_s", "bind", "fake", "errors"),
    "identity": ("hostname", "user", "model", "l4t_release", "l4t_revision", "kernel",
                 "boot_time", "uptime_s", "drone_env", "tailscale"),
    "repo": ("path", "present", "version_tag", "commit", "branch", "dirty"),
    "image": ("expected_ref", "present", "id", "id_short", "repo_digests", "created",
              "size_bytes", "labels", "team_image", "skybeta_tags_present"),
    "docker": ("daemon_ok", "server_version", "containers"),
    "stack": ("running", "container", "uart_holder", "image_ref", "image_id", "uptime_s",
              "ros_domain_id", "dds_scope", "tmux"),
    "sidecar": ("running", "container", "image_ref", "image_id", "uptime_s"),
    "ros": ("file_present", "file_mtime"),
    "doctor": ("args", "exit_code", "duration_s", "ran_at", "header", "pass", "warn", "fail",
               "ok", "checks", "unparsed_lines"),
    "system": ("tegrastats", "fallback", "disk", "power_mode"),
    "team_ws": ("path", "exists", "packages", "package_names", "overlay_built", "dockerfile",
                "foreign_owned_files", "src_last_modified", "src_file_count", "git"),
}
DRONE_ENV_KEYS = ("path", "present", "mtime", "ignored_keys", "drone_id", "team", "ros_domain_id",
                  "dds_scope", "camera", "team_ws", "competitor_pkg", "competitor_launch",
                  "domain_error")
TAILSCALE_KEYS = ("ipv4", "dns_name", "hostname", "tags", "backend_state", "version")
TEGRASTATS_KEYS = ("available", "age_s", "raw", "ram_used_mb", "ram_total_mb", "swap_used_mb",
                   "swap_total_mb", "cpu", "cpu_load_pct_avg", "gpu_load_pct", "gpu_freq_mhz",
                   "temps_c", "temp_max_c", "temp_max_zone", "power_mw")


def _meta(updated_at, mono, err, now_mono):
    return {"updated_at": updated_at,
            "age_s": round(now_mono - mono, 2) if mono is not None else None,
            "error": err}


def section(collector, now_mono, key=None):
    """One collector snapshot -> section dict (meta first, then the data).

    Never collected -> the collector's EMPTY template (nulls), so no key is
    ever missing. Collected then failed -> last good data + ``error``.
    """
    data, updated_at, mono, err = collector.snapshot()
    empty = collector.EMPTY
    if key is not None:
        data = data.get(key) if data else None
        empty = empty.get(key, {})
    out = _meta(updated_at, mono, err, now_mono)
    out.update(data if data is not None else empty)
    return out


def ros_section(collector, now):
    data, updated_at, mono, err = collector.snapshot()
    out = {"updated_at": updated_at, "age_s": None, "error": err}
    out.update(data if data is not None else collector.EMPTY)
    written = out.get("written_at")
    if out.get("file_present") and isinstance(written, (int, float)):
        out["age_s"] = round(now - written, 2)
    return out


def build(agent, now=None, now_mono=None):
    """The full document. ``agent`` provides ``collectors`` (name -> Collector),
    ``tegrastats`` (a reader with ``latest()``) and ``info()`` (agent section)."""
    now = time.time() if now is None else now
    now_mono = time.monotonic() if now_mono is None else now_mono
    c = agent.collectors
    identity = section(c["identity"], now_mono)
    ts = c["tailscale_self"].snapshot()[0]
    identity["tailscale"] = dict(ts) if ts else dict(c["tailscale_self"].EMPTY)
    system = section(c["system"], now_mono)
    system = dict({k: system[k] for k in META_KEYS}, tegrastats=agent.tegrastats.latest(),
                  **{k: v for k, v in system.items() if k not in META_KEYS})
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": round(now, 3),
        "agent": agent.info(now, now_mono),
        "identity": identity,
        "repo": section(c["repo"], now_mono),
        "image": section(c["image"], now_mono),
        "docker": section(c["docker_state"], now_mono, "docker"),
        "stack": section(c["docker_state"], now_mono, "stack"),
        "sidecar": section(c["docker_state"], now_mono, "sidecar"),
        "ros": ros_section(c["ros_summary"], now),
        "doctor": section(c["doctor"], now_mono),
        "system": system,
        "team_ws": section(c["team_ws"], now_mono),
    }
