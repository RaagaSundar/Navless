"""tailscale_self: this node's Tailscale IPv4, MagicDNS name, tags, state.

``tailscaled.sock`` is world-writable on the fleet image, so both commands
work unprivileged. The same ``tailscale ip -4`` lookup is what httpd binds to
(see ``resolve_tailscale_ip``), with an ``ip -4 -o addr`` fallback.
"""
import ipaddress
import json
import re
import shutil

from . import Collector, CommandError, run_cmd

CGNAT = ipaddress.ip_network("100.64.0.0/10")


def tailscale_bin():
    return shutil.which("tailscale") or "/usr/bin/tailscale"


def is_cgnat(ip):
    try:
        return ipaddress.ip_address(ip) in CGNAT
    except ValueError:
        return False


def parse_self(status):
    """``tailscale status --json`` document -> the identity.tailscale dict."""
    me = status.get("Self") or {}
    ips = me.get("TailscaleIPs") or status.get("TailscaleIPs") or []
    ipv4 = next((a for a in ips if ":" not in a), None)
    version = status.get("Version") or ""
    return {
        "ipv4": ipv4,
        "dns_name": (me.get("DNSName") or "").rstrip(".") or None,
        "hostname": me.get("HostName"),
        "tags": list(me.get("Tags") or []),
        "backend_state": status.get("BackendState"),
        "version": version.split("-")[0] or None,
    }


_IP_ADDR_RE = re.compile(r"^\d+:\s+(\S+)\s+inet\s+(\d+\.\d+\.\d+\.\d+)/\d+")


def parse_ip_addr(text, ifname="tailscale0"):
    """First 100.64/10 IPv4 on ``ifname`` from ``ip -4 -o addr`` output."""
    for line in text.splitlines():
        m = _IP_ADDR_RE.match(line.strip())
        if m and m.group(1) == ifname and is_cgnat(m.group(2)):
            return m.group(2)
    return None


def resolve_tailscale_ip(timeout=5.0):
    """The address to bind: ``tailscale ip -4``, else tailscale0's CGNAT addr, else None."""
    try:
        _, out, _ = run_cmd([tailscale_bin(), "ip", "-4"], timeout)
        for line in out.split():
            if is_cgnat(line.strip()):
                return line.strip()
    except CommandError:
        pass
    try:
        _, out, _ = run_cmd(["ip", "-4", "-o", "addr"], timeout)
        return parse_ip_addr(out)
    except CommandError:
        return None


class TailscaleSelfCollector(Collector):
    name = "tailscale_self"
    period_s = 30.0
    EMPTY = {"ipv4": None, "dns_name": None, "hostname": None, "tags": [],
             "backend_state": None, "version": None}

    def collect(self):
        _, out, _ = run_cmd([tailscale_bin(), "status", "--json"], 5.0)
        data = parse_self(json.loads(out))
        try:
            _, ip_out, _ = run_cmd([tailscale_bin(), "ip", "-4"], 5.0)
            first = ip_out.strip().split()
            if first:
                data["ipv4"] = first[0]
        except CommandError:
            pass
        return data
