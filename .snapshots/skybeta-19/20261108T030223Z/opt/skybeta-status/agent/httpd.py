"""Read-only HTTP server for /status, bound to the drone's Tailscale IPv4 only.

Routes: ``GET|HEAD /status`` (JSON, ``?pretty=1`` for humans), ``/healthz``
(``ok``), ``/version``. Unknown path -> 404, any other method -> 405. No
request ever selects a file, a command or a collector: the routes are fixed.

Binding (``BindManager``): the Tailscale IP is resolved with ``tailscale ip -4``
(fallback: tailscale0's address from ``ip -4 -o addr``). With
``net.ipv4.ip_nonlocal_bind=0`` a bind before tailscale0 is up fails with
EADDRNOTAVAIL -- treated exactly like "no IP yet": log once a minute, retry
every 5 s, never exit (systemd would only flap). A watchdog re-resolves every
30 s and rebinds when the IP changes. Never 0.0.0.0, never the WiFi address.
"""
import ipaddress
import json
import logging
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .collectors.tailscale_self import is_cgnat, resolve_tailscale_ip

log = logging.getLogger("skybeta-status")

ALLOWED_METHODS = "GET, HEAD"


class StatusServer(ThreadingHTTPServer):
    daemon_threads = True
    # SO_REUSEADDR on Windows lets two processes bind one port; only use it on POSIX.
    allow_reuse_address = os.name != "nt"

    def handle_error(self, request, client_address):
        # A client hanging up mid-request is routine (poll timeouts); not a traceback.
        exc = sys.exc_info()[1]
        if isinstance(exc, (ConnectionError, TimeoutError)):
            log.debug("client %s: %s", client_address, exc)
            return
        log.exception("error handling request from %s", client_address)


def make_handler(doc_source, version):
    """Handler class serving ``doc_source()`` (a callable returning a dict)."""

    class StatusHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        timeout = 10
        server_version = "skybeta-status/%s" % version

        def version_string(self):
            return self.server_version   # no Python version in the Server header

        def log_message(self, fmt, *args):  # noqa: D401 - stdlib signature
            log.debug("%s %s", self.address_string(), fmt % args)

        def _send(self, code, body, ctype, head_only=False, extra=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if not head_only:
                self.wfile.write(body)

        def _route(self, head_only):
            path, _, query = self.path.partition("?")
            if path == "/status":
                pretty = "pretty=1" in query.split("&")
                try:
                    doc = doc_source()
                    body = json.dumps(doc, indent=2 if pretty else None, sort_keys=False,
                                      allow_nan=False, default=str).encode("utf-8")
                except Exception as e:  # noqa: BLE001 - never kill the handler thread
                    log.exception("building /status failed")
                    body = json.dumps({"error": "status build failed: %s" % e}).encode()
                    return self._send(500, body, "application/json", head_only)
                return self._send(200, body, "application/json", head_only)
            if path == "/healthz":
                return self._send(200, b"ok\n", "text/plain; charset=utf-8", head_only)
            if path == "/version":
                body = json.dumps({"version": version, "schema_version": 1}).encode()
                return self._send(200, body, "application/json", head_only)
            self.close_connection = True
            return self._send(404, b"not found\n", "text/plain; charset=utf-8", head_only,
                              {"Connection": "close"})

        def do_GET(self):  # noqa: N802
            self._route(False)

        def do_HEAD(self):  # noqa: N802
            self._route(True)

        def _not_allowed(self):
            self.close_connection = True   # any request body stays unread
            self._send(405, b"method not allowed\n", "text/plain; charset=utf-8",
                       self.command == "HEAD", {"Allow": ALLOWED_METHODS, "Connection": "close"})

        def __getattr__(self, name):
            # BaseHTTPRequestHandler answers unknown methods with 501; the
            # contract here is 405 for every method except GET/HEAD.
            if name.startswith("do_"):
                return self._not_allowed
            raise AttributeError(name)

    return StatusHandler


def make_server(ip, port, handler):
    return StatusServer((ip, port), handler)


def check_bind_ip(ip, i_know=False):
    """--bind <ip> is for --fake/tests: loopback or 100.64/10 only, unless --i-know."""
    try:
        addr = ipaddress.ip_address(ip)
    except ValueError:
        raise ValueError("--bind %r is not an IP address (or 'tailscale')" % ip)
    if addr.is_loopback or is_cgnat(ip) or i_know:
        return ip
    raise ValueError("--bind %s is neither loopback nor a Tailscale (100.64.0.0/10) address; "
                     "pass --i-know to bind it anyway" % ip)


class BindManager:
    """Keeps one server on the resolved IP (and optionally one on 127.0.0.1)."""

    RETRY_S = 5.0
    WATCHDOG_S = 30.0
    LOG_EVERY_S = 60.0

    def __init__(self, handler, port, bind="tailscale", also_localhost=False, on_change=None,
                 resolver=resolve_tailscale_ip):
        self.handler = handler
        self.port = port
        self.bind = bind
        self.also_localhost = also_localhost
        self.on_change = on_change or (lambda ip, port: None)
        self.resolver = resolver
        self.current_ip = None
        self._srv = None
        self._local = None
        self._last_log = 0.0

    def _resolve(self):
        return self.resolver() if self.bind == "tailscale" else self.bind

    def _start(self, ip):
        srv = make_server(ip, self.port, self.handler)
        threading.Thread(target=srv.serve_forever, name="httpd-%s" % ip, daemon=True).start()
        return srv

    def _stop_main(self):
        if self._srv is not None:
            self._srv.shutdown()
            self._srv.server_close()
            self._srv = None
        self.current_ip = None

    def _rate_limited(self, msg, *args):
        now = time.monotonic()
        if now - self._last_log >= self.LOG_EVERY_S:
            log.warning(msg, *args)
            self._last_log = now

    def run(self, stop):
        if self.also_localhost and self.bind not in ("127.0.0.1",):
            try:
                self._local = self._start("127.0.0.1")
                log.info("also serving on 127.0.0.1:%d", self.port)
            except OSError as e:
                log.error("cannot bind 127.0.0.1:%d: %s", self.port, e)
        while not stop.is_set():
            ip = self._resolve()
            if ip != self.current_ip:
                if self.current_ip is not None:
                    log.info("bind IP changed %s -> %s; rebinding", self.current_ip, ip)
                self._stop_main()
                if ip:
                    try:
                        self._srv = self._start(ip)
                        self.current_ip = ip
                        self.port = self._srv.server_address[1]
                        log.info("serving /status on http://%s:%d", ip, self.port)
                        self._last_log = 0.0
                    except OSError as e:   # EADDRNOTAVAIL before tailscale0 is up, EADDRINUSE
                        self._rate_limited("cannot bind %s:%d (%s); retrying every %.0fs",
                                           ip, self.port, e, self.RETRY_S)
                else:
                    self._rate_limited("no Tailscale IPv4 yet (tailscale ip -4 / tailscale0); "
                                       "retrying every %.0fs", self.RETRY_S)
                self.on_change(self.current_ip, self.port)
            stop.wait(self.WATCHDOG_S if self.current_ip else self.RETRY_S)
        self._stop_main()
        if self._local is not None:
            self._local.shutdown()
            self._local.server_close()
