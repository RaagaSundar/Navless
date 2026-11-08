"""Checks for tube_test.py, plain asserts: python3 gazebo_sim/mirror/test_tube_test.py"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tube_test import TubeTest, odom_state  # noqa: E402


def frame(t, vo, age=0.01, still=0.0, jump=None, gate=None):
    return {"t": t, "vo_state": vo, "age": age, "vio": {"n": 0, "still": still, "jump": jump, "gate": gate}}


def test_odom_state():
    assert odom_state(frame(0, 1)) == "moving"
    assert odom_state(frame(0, 2, still=0.5)) == "frozen"
    assert odom_state(frame(0, 2, age=0.6, still=0.6)) == "stopped"
    assert odom_state({"t": 0, "vo_state": 2, "age": None}) == "stopped"


def test_never_lost():
    tt = TubeTest()
    for i in range(90):
        tt.feed(frame(i / 30, 1))
    s = tt.summary()
    assert s["losses"] == 0 and s["last"] is None and set(s["strip"]) <= {"T", "."}
    assert "never went to 2" in tt.report()[0]


def test_loss_with_stopped_odometry_and_an_exit_jump():
    tt, t = TubeTest(), 0.0
    for _ in range(30):                          # tracking
        tt.feed(frame(t, 1))
        t += 1 / 30
    for k in range(60):                          # 2 s lost; odometry stops, so its age grows
        tt.feed(frame(t, 2, age=0.03 + k / 30, still=0.03 + k / 30))
        t += 1 / 30
    for i in range(30):                          # back; cuVSLAM re-inits with a 15 cm step 0.1 s later
        tt.feed(frame(t, 1, jump=[3.1, 0.15] if i >= 3 else None))
        t += 1 / 30
    s = tt.summary()
    assert s["losses"] == 1 and not s["lost_now"]
    assert abs(s["last"]["dur_s"] - 2.0) < 0.05, s["last"]
    assert s["last"]["odom"]["stopped"] > 1.5 and s["last"]["jump_m"] == 0.15, s["last"]
    assert "S" in s["strip"] and "T" in s["strip"]
    r = "\n".join(tt.report())
    assert "odometry stopped" in r and "15 cm jump" in r, r


def test_frozen_odometry_and_gate_counts():
    tt = TubeTest()

    def g(a, r):
        return {"accepted": a, "rejected": r, "age": 0.1}
    tt.feed(frame(0.0, 1, gate=g(100, 0)))
    for k in range(30):                          # lost; messages keep coming with the same pose
        tt.feed(frame(0.1 + k / 30, 2, still=0.1 + k / 30, gate=g(100, 0)))
    tt.feed(frame(1.2, 1, gate=g(101, 0)))
    s = tt.summary()
    assert s["last"]["odom"].get("frozen", 0) > 0.5, s["last"]
    assert s["gate"] == {"accepted": 101, "rejected": 0, "rejected_since_start": 0}
    assert "frozen pose" in "\n".join(tt.report())
    tt.reset()
    assert tt.summary()["losses"] == 0 and tt.summary()["gate"] is None


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in tests:
        fn()
    print(f"tube_test checks: {len(tests)} passed")
