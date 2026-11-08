"""The dark tube: where the camera is taken to be blind, and what visual odometry does about it.

sensor_bridge.py (PX4's vision, drone profile) and fly_navless.py (the node's odometry) each run a
TubeBlackout on the true pose. They lose and regain tracking together and agree on any jump, as on the
drone: there the odometry bridge forwards whatever cuVSLAM publishes and bumps reset_counter on every
lost -> tracking transition.

The settings come from the environment, so one export before run_sim.sh and fly_navless.py covers both:
  DH_TUBE_BLACKOUT       off | stop | freeze  (1 = stop). While blind vo_state is 2, and odometry stops
                         (stop) or repeats the last tracked pose (freeze)
  DH_TUBE_RECOVER_DELAY  s after leaving the tube before tracking comes back (default 0)
  DH_TUBE_RESET_JUMP     what the pose does when tracking comes back: empty for no jump; dx,dy,dz in the
                         odom frame (m, x = the nose at power-up); or resume, which restarts from the last
                         tracked pose as if the drone had not moved while blind (no IMU fusion)
  DH_PAD_DRIFT           dx,dy,dz (odom m): visual-odometry drift at the landing pad. Once the true pose is
                         back within PAD_DRIFT_RADIUS of the pad after being 3 m+ away (the launch mark is
                         1.8 m from it), the error ramps in over
                         PAD_DRIFT_RAMP m of path, for PX4's vision and the node's odometry alike.
"""
import math
import os

import numpy as np

# Inside of the tube, world metres: x/y from its collision plates (models/dh_course), z below the inside of
# its top (73 in). Above that roof the camera sees the room, so the home leg can pass over it.
TUBE_X = (1.5748, 3.048)
TUBE_Y = (-0.3048, 1.2192)
TUBE_TOP = 1.84
MODES = ("off", "stop", "freeze")
PAD_XY = np.array([-2.042, -0.792])     # course_v3.json landing_pad_in centre, world m
PAD_DRIFT_RADIUS, PAD_DRIFT_RAMP = 2.0, 1.0


def in_tube(p):
    return TUBE_X[0] < p[0] < TUBE_X[1] and TUBE_Y[0] < p[1] < TUBE_Y[1] and p[2] < TUBE_TOP


def parse_mode(text):
    t = (text or "off").strip().lower()
    t = {"0": "off", "1": "stop", "": "off"}.get(t, t)
    if t not in MODES:
        raise ValueError(f"tube blackout {text!r}: use off, stop or freeze")
    return t


def parse_jump(text):
    """'' -> None, 'resume' -> 'resume', 'dx,dy,dz' -> np.array (odom frame, m)."""
    t = (text or "").strip().lower()
    if t in ("", "0", "none", "off"):
        return None
    if t == "resume":
        return t
    v = [float(x) for x in t.split(",")]
    if len(v) != 3:
        raise ValueError(f"tube reset jump {text!r}: use dx,dy,dz or resume")
    return np.array(v)


def env_settings():
    """(mode, recover_delay_s, jump text) from DH_TUBE_*: what sensor_bridge.py runs with."""
    return (parse_mode(os.environ.get("DH_TUBE_BLACKOUT")), float(os.environ.get("DH_TUBE_RECOVER_DELAY") or 0),
            os.environ.get("DH_TUBE_RESET_JUMP", ""))


def _crossing(a, b, n=14):
    """(t, p) where the straight segment between samples a and b crosses the tube boundary."""
    (ta, pa), (tb, pb) = a, b
    side, lo, hi = in_tube(pa), 0.0, 1.0
    for _ in range(n):
        mid = (lo + hi) / 2
        if in_tube(pa + (pb - pa) * mid) == side:
            lo = mid
        else:
            hi = mid
    k = (lo + hi) / 2
    return ta + (tb - ta) * k, pa + (pb - pa) * k


class TubeBlackout:
    """Feed step() every odometry sample (sim time, true world position).

    Tracking is lost at the sample that first lies in the tube and comes back recover_delay s after the path
    leaves it (re-entering first keeps it lost). Entry, exit and the recovery point are interpolated between
    samples, so two processes sampling at different instants agree on them to well under a millimetre.
    """

    def __init__(self, mode, yaw0, recover_delay=0.0, jump=None, log=None):
        self.mode = parse_mode(mode)
        self.delay = float(recover_delay)
        self.jump_text = jump if isinstance(jump, str) else None
        self.jump = parse_jump(jump) if isinstance(jump, str) or jump is None else np.asarray(jump, float)
        c, s = math.cos(yaw0), math.sin(yaw0)
        self.R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])   # odom -> world (power-up heading)
        self.log = log or (lambda text: None)
        self.offset = np.zeros(3)        # world-frame shift added to reported positions (the jumps so far)
        self.tracking = True
        self.prev = None                 # previous sample (t, p)
        self.entry = None                # (t, p) where the current blackout started
        self.exit = None                 # (t, p) where the path left the tube during it
        self.held = None                 # reported position frozen at the entry
        self.passes = []                 # one record per blackout, sim times
        drift = parse_jump(os.environ.get("DH_PAD_DRIFT", ""))
        self.drift = None if drift is None or isinstance(drift, str) else self.R @ drift   # world frame
        self.drift_path = None           # path length since the drift started, m
        self.drift_armed = False         # been 3 m+ from the pad (left the launch area)

    def drift_now(self, p):
        """The pad drift (world) for this true position; advances along the path."""
        if self.drift is None:
            return np.zeros(3)
        if self.drift_path is None:
            self.drift_armed |= bool(np.linalg.norm(p[:2] - PAD_XY) >= 3.0)
            if not self.drift_armed or np.linalg.norm(p[:2] - PAD_XY) >= PAD_DRIFT_RADIUS:
                return np.zeros(3)
            self.drift_path, self.drift_last = 0.0, p
            self.log(f"pad drift starts ramping in at world ({p[0]:.2f}, {p[1]:.2f})")
        self.drift_path += float(np.linalg.norm(p - self.drift_last))
        self.drift_last = p
        return self.drift * min(1.0, self.drift_path / PAD_DRIFT_RAMP)

    def describe(self):
        jump = ("none" if self.jump is None else self.jump if isinstance(self.jump, str)
                else ",".join(f"{v:g}" for v in self.jump) + " m (odom)")
        drift = "" if self.drift is None else f", pad drift {os.environ.get('DH_PAD_DRIFT')} m (odom)"
        return f"{self.mode}, recover delay {self.delay:g} s, reset jump {jump}{drift}"

    def step(self, t, p):
        """Returns (tracking, reported world position, or None when nothing is published)."""
        p = np.asarray(p, float)
        tracking, reported = self._step(t, p)
        return tracking, (None if reported is None else reported + self.drift_now(p))

    def _step(self, t, p):
        inside = self.mode != "off" and in_tube(p)
        prev, self.prev = self.prev, (t, p)
        if self.tracking:
            if not inside:
                return True, p + self.offset
            self.entry = _crossing(prev, (t, p)) if prev is not None and not in_tube(prev[1]) else (t, p)
            self.held, self.exit, self.tracking = self.entry[1] + self.offset, None, False
            self.passes.append({"lost_t": round(self.entry[0], 3), "left_tube_t": None, "back_t": None,
                                "blind_s": None, "jump_odom_m": None})
            self.log(f"t={t:.2f} camera blind in the tube ({self.mode}), entered at world "
                     f"({self.entry[1][0]:.2f}, {self.entry[1][1]:.2f}, {self.entry[1][2]:.2f})")
        elif inside:
            self.exit = None
        else:
            if self.exit is None:
                self.exit = _crossing(prev, (t, p))
                self.passes[-1]["left_tube_t"] = round(self.exit[0], 3)
            t_back = self.exit[0] + self.delay
            if t >= t_back:
                tp, pp = prev
                k = 1.0 if t <= tp else min(1.0, max(0.0, (t_back - tp) / (t - tp)))
                p_back = pp + (p - pp) * k
                if isinstance(self.jump, str):            # resume: restart from the last tracked pose
                    jump_w = self.entry[1] - p_back
                elif self.jump is not None:
                    jump_w = self.R @ self.jump
                else:
                    jump_w = np.zeros(3)
                self.offset = self.offset + jump_w
                self.tracking = True
                jo = self.R.T @ jump_w
                self.passes[-1].update(back_t=round(t_back, 3), blind_s=round(t_back - self.entry[0], 3),
                                       jump_odom_m=[round(float(v), 3) for v in jo])
                self.log(f"t={t:.2f} tracking back after {t_back - self.entry[0]:.2f} s blind; "
                         f"odometry jumps ({jo[0]:+.3f}, {jo[1]:+.3f}, {jo[2]:+.3f}) m (odom)")
                return True, p + self.offset
        return False, (self.held if self.mode == "freeze" else None)
