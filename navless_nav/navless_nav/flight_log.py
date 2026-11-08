"""course_mission's flight recorder: one CSV row per control tick, plus a row
for every log line, so a flight can be debugged afterwards without a bag.

    <dir>/<YYYYmmdd-HHMMSS>_course.csv          the rows (flushed every second)
    <dir>/<YYYYmmdd-HHMMSS>_course_params.json  every parameter the node ran with

<dir> is the node's flight_log_dir parameter, else $NAVLESS_FLIGHT_LOG_DIR,
else ~/team_ws/flight_logs (in the stack container ~ is /root, and
/root/team_ws is the host's ~/team_ws). Logging never stops the flight: any
error opening or writing the file turns the recorder off with one warning.
"""
import csv
import json
import os
import time

DEFAULT_DIR = '~/team_ws/flight_logs'

# One tick row. Positions are odom (ENU, metres) as this node flies them
# (odom minus odom_zero_offset) unless named raw_/px4_; yaws in degrees.
COLUMNS = [
    'wall_time', 't', 'kind', 'state', 'event',
    # where we are
    'x', 'y', 'z', 'yaw_deg', 'raw_x', 'raw_y', 'raw_z', 'odom_age', 'vio_ok', 'vo_state',
    # where we want to be
    'target_x', 'target_y', 'target_z', 'carrot_x', 'carrot_y', 'carrot_z', 'carrot_i', 'route_n',
    'sent_x', 'sent_y', 'sent_z', 'err_xy', 'err_z',
    # heading
    'heading_mode', 'cmd_yaw_deg', 'hold_yaw_deg', 'want_yaw_deg', 'heading_suspended', 'tube_heading',
    'px4_yaw_deg', 'px4_heading_resets',
    # stabilisation (hold drift correction), body frame and as applied
    'drift_body_x', 'drift_body_y', 'drift_x', 'drift_y', 'drift_learning',
    # wall correction (cloud_localize)
    'corr_dx', 'corr_dy', 'corr_dyaw_deg', 'corr_rx_age',
    # PX4 / flight_interface
    'armed', 'offboard', 'landed', 'px4_x', 'px4_y', 'px4_z', 'z_offset',
    'zero_x', 'zero_y', 'zero_z', 'prearm_passed',
    # tube and planner
    'in_tube_zone', 'blind_carrying', 'tof_shift', 'stereo_shift', 'replans',
]


def resolve_dir(param_value):
    d = str(param_value or '').strip() or os.environ.get('NAVLESS_FLIGHT_LOG_DIR', '') or DEFAULT_DIR
    return os.path.expanduser(d)


class FlightLog:
    def __init__(self, log_dir, params, warn=print, tag='course'):
        self.warn, self.f, self.w, self.path = warn, None, None, None
        self.last_flush = time.monotonic()
        try:
            os.makedirs(log_dir, exist_ok=True)
            stamp = time.strftime('%Y%m%d-%H%M%S')
            self.path = os.path.join(log_dir, f'{stamp}_{tag}.csv')
            with open(os.path.join(log_dir, f'{stamp}_{tag}_params.json'), 'w') as pf:
                json.dump(params, pf, indent=1, sort_keys=True, default=str)
            self.f = open(self.path, 'w', newline='')
            self.w = csv.DictWriter(self.f, fieldnames=COLUMNS, extrasaction='ignore')
            self.w.writeheader()
        except OSError as exc:
            self.off(f'flight log off: cannot write to {log_dir} ({exc})')

    @property
    def on(self):
        return self.w is not None

    def off(self, why):
        self.w = None
        try:
            if self.f:
                self.f.close()
        except (OSError, ValueError):
            pass
        self.f = None
        self.warn(why)

    def row(self, values):
        if self.w is None:
            return
        try:
            self.w.writerow({k: _cell(v) for k, v in values.items()})
            now = time.monotonic()
            if now - self.last_flush >= 1.0:
                self.f.flush()
                self.last_flush = now
        except (OSError, ValueError) as exc:
            self.off(f'flight log off: write failed ({exc})')

    def close(self):
        if self.f:
            try:
                self.f.flush()
                self.f.close()
            except (OSError, ValueError):
                pass
            self.f = None
        self.w = None


def _cell(v):
    if v is None:
        return ''
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, float):
        return '' if v != v else round(v, 4)       # NaN -> empty
    return v


class LoggingLogger:
    """Wraps the node's ROS logger: every line also goes into the flight log
    as an 'event' row. Each severity has its own call into the real logger
    (rclpy rejects one call site used with different severities)."""

    def __init__(self, real, record):
        self._real, self._record = real, record

    def info(self, msg, *a, **kw):
        self._record('info', msg)
        return self._real.info(msg, *a, **kw)

    def warn(self, msg, *a, **kw):
        self._record('warn', msg)
        return self._real.warn(msg, *a, **kw)

    warning = warn

    def error(self, msg, *a, **kw):
        self._record('error', msg)
        return self._real.error(msg, *a, **kw)

    def debug(self, msg, *a, **kw):
        fn = getattr(self._real, 'debug', None)
        return fn(msg, *a, **kw) if fn else None

    def __getattr__(self, name):
        return getattr(self._real, name)
