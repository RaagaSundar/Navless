"""dh/tube.py: two processes sampling at different instants must agree; modes, delay, jumps, roof.

  python3 -m pytest gazebo_sim/dh/test_tube_blackout.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dh.tube import TUBE_Y, TubeBlackout  # noqa: E402


def through(t):
    """North through the tube at 1 m/s, 0.5 m up, starting 1 m south of it."""
    return np.array([2.3, -1.3 + t, 0.5])


def fly(mode, phase=0.0, yaw0=0.0, delay=0.0, jump=None, path=through, t_end=4.0):
    """30 Hz samples from `phase`: (t, true p, tracking, reported p or None) each."""
    tb, out, t = TubeBlackout(mode, yaw0, delay, jump), [], phase
    while t < t_end:
        p = path(t)
        out.append((t, p) + tb.step(t, p))
        t += 1 / 30
    return tb, out


def test_two_samplers_agree():
    a, _ = fly("stop", 0.0, delay=0.4, jump="resume")
    b, _ = fly("stop", 0.0137, delay=0.4, jump="resume")
    (pa,), (pb,) = a.passes, b.passes
    for k in ("lost_t", "left_tube_t", "back_t"):
        assert abs(pa[k] - pb[k]) < 1e-3
    assert abs(pa["lost_t"] - (1.3 + TUBE_Y[0])) < 1e-3          # the entry plane
    assert abs(pa["left_tube_t"] - (1.3 + TUBE_Y[1])) < 1e-3     # the exit plane
    assert abs(pa["back_t"] - pa["left_tube_t"] - 0.4) < 1e-3
    assert np.allclose(pa["jump_odom_m"], pb["jump_odom_m"], atol=1e-3)
    assert np.allclose(pa["jump_odom_m"], [0, -(pa["back_t"] - pa["lost_t"]), 0], atol=2e-3)


def test_stop_publishes_nothing_then_applies_the_jump():
    tb, out = fly("stop", jump="resume")
    blind = [r for r in out if not r[2]]
    assert len(blind) > 30 and all(r[3] is None for r in blind)
    jump = np.array(tb.passes[0]["jump_odom_m"])                  # yaw0 0: odom axes are world axes
    after = [r for r in out if r[2] and r[0] > tb.passes[0]["back_t"]]
    assert after and all(np.allclose(r[3], r[1] + jump, atol=2e-3) for r in after)


def test_freeze_holds_the_entry_and_resume_continues_from_it():
    tb, out = fly("freeze", 0.005, jump="resume")
    held = [r[3] for r in out if not r[2]]
    assert len(held) > 30 and all(np.array_equal(h, held[0]) for h in held)
    assert abs(held[0][1] - TUBE_Y[0]) < 1e-3
    back = next(r for r in out if r[2] and r[0] > tb.passes[0]["lost_t"])
    assert np.linalg.norm(back[3] - held[0]) < 1 / 30 + 1e-3     # at most one sample of travel


def test_fixed_jump_is_in_the_odom_frame():
    yaw0 = math.radians(30)
    tb, out = fly("stop", yaw0=yaw0, jump="0.2,-0.1,0")
    R = np.array([[math.cos(yaw0), -math.sin(yaw0), 0], [math.sin(yaw0), math.cos(yaw0), 0], [0, 0, 1]])
    assert np.allclose(tb.passes[0]["jump_odom_m"], [0.2, -0.1, 0])
    assert np.allclose(out[-1][3] - out[-1][1], R @ [0.2, -0.1, 0])


def test_never_blind_above_the_roof_or_when_off():
    assert all(r[2] for r in fly("stop", path=lambda t: through(t) + [0, 0, 1.5])[1])
    assert all(r[2] for r in fly("off")[1])


def test_reentry_before_recovery_is_one_blackout():
    def wiggle(t):    # out of the north end, back in 0.2 s later, out again
        y = -1.3 + t if t < 2.6 else (1.3 - (t - 2.6) if t < 2.8 else 1.1 + (t - 2.8))
        return np.array([2.3, y, 0.5])
    tb, _ = fly("stop", delay=0.5, path=wiggle, t_end=5.0)
    assert len(tb.passes) == 1 and tb.passes[0]["back_t"] > 3.3


def test_still_blind_at_the_end():
    assert fly("stop", t_end=2.0)[0].passes[0]["back_t"] is None
