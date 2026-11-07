"""VIO-loss test: what cuVSLAM and its odometry do when tracking is lost, as it may be in the dark tube.

The rules don't allow walking through any part of the course, so run it off the course: cover the lenses
for a few seconds, or carry the drone through a dark box on the bench, or replay frames recorded in flight.
Reads the frames navless_nav's mirror_relay sends: live in mirror.py's "Tube test" panel, or afterwards
from a file made with `mirror.py --record`. It answers what decides blind_carry (PR #18) and whether PX4
should be fed through Koji's gate:
  - each stretch of vo_state 2 (lost): when, and for how long;
  - what /visual_slam/tracking/odometry did meanwhile: stopped (no messages), frozen (messages with the
    same pose, bit for bit) or moving;
  - the largest pose jump during it or just after (one step over 8 cm), usually on the way out;
  - Koji's gate (/mapping/gate_status): what it accepted and rejected over the recording.

  python3 gazebo_sim/mirror/tube_test.py vio_loss.jsonl
"""
import json
import sys
from collections import deque

STOPPED_S = 0.25      # no odometry message for this long: stopped
FROZEN_S = 0.25       # messages arriving, the pose unchanged for this long: frozen
JUMP_WINDOW_S = 3.0   # a jump this soon after tracking returns belongs to that loss
STRIP_S = 60.0        # the page's timeline
CODES = {"stopped": "S", "frozen": "F", "moving": "M"}
MEANS = {
    "stopped": "odometry stopped while lost, so PX4 dead-reckons: the case blind_carry was flown for in Gazebo",
    "frozen": "odometry kept arriving with a frozen pose: on the raw feed PX4 would fuse a pose that doesn't "
              "move; fed through Koji's gate (which passes nothing while vo_state is 2) it would dead-reckon",
    "moving": "odometry kept moving while vo_state said lost: check whether it followed the carry or drifted",
}


def odom_state(f):
    """'stopped', 'frozen' or 'moving' for one relay frame."""
    age, vio = f.get("age"), f.get("vio") or {}
    if age is None or age > STOPPED_S:
        return "stopped"
    still = vio.get("still")
    return "frozen" if still is not None and still > FROZEN_S else "moving"


class TubeTest:
    """Fed every relay frame in order; summary() for the page, report() for the terminal."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.losses, self.cur, self.prev_t, self.seen_jump, self.t0 = [], None, None, None, None
        self.gate_first = self.gate_last = self.now = None
        self.strip = deque()        # (t, code): T tracking; S, F, M lost with odometry stopped/frozen/moving

    def feed(self, f):
        t = f.get("t")
        if t is None:
            return
        dt = 0.0 if self.prev_t is None else min(max(t - self.prev_t, 0.0), 0.2)
        self.prev_t = t
        if self.t0 is None:
            self.t0 = t                              # the drone's clock: report times from here
        vio = f.get("vio") or {}
        state, lost = odom_state(f), f.get("vo_state") == 2
        self.now = {"vo_state": f.get("vo_state"), "odom": state}
        jump = vio.get("jump")
        if jump and jump != self.seen_jump:          # the relay repeats its last jump in every frame
            self.seen_jump = jump
            if self.cur is not None:
                target = self.cur
            elif self.losses and jump[0] - self.losses[-1]["end"] <= JUMP_WINDOW_S:
                target = self.losses[-1]
            else:
                target = None
            if target is not None:
                target["jump_m"] = max(target["jump_m"] or 0.0, jump[1])
        if vio.get("gate"):
            self.gate_first = self.gate_first or vio["gate"]
            self.gate_last = vio["gate"]
        if lost:
            if self.cur is None:
                self.cur = {"start": t, "end": t, "odom": {}, "jump_m": None}
            self.cur["end"] = t
            self.cur["odom"][state] = self.cur["odom"].get(state, 0.0) + dt
        elif self.cur is not None:
            self.losses.append(self.cur)
            self.cur = None
        self.strip.append((t, CODES[state] if lost else "T"))
        while self.strip[0][0] < t - STRIP_S:
            self.strip.popleft()

    def all_losses(self):
        return self.losses + ([self.cur] if self.cur is not None else [])

    def summary(self, cells=120):
        losses = self.all_losses()
        durs = [lo["end"] - lo["start"] for lo in losses]
        last, g0, g1 = (losses[-1] if losses else None), self.gate_first, self.gate_last
        jumps = [lo["jump_m"] for lo in losses if lo["jump_m"] is not None]
        since = None
        if g1 is not None and isinstance(g1.get("rejected"), int) and isinstance(g0.get("rejected"), int):
            since = g1["rejected"] - g0["rejected"]
        return {
            "now": self.now, "losses": len(losses), "lost_now": self.cur is not None,
            "longest_s": round(max(durs), 2) if durs else None, "total_s": round(sum(durs), 2),
            "last": None if last is None else {
                "dur_s": round(last["end"] - last["start"], 2),
                "odom": {k: round(v, 2) for k, v in last["odom"].items()},
                "jump_m": None if last["jump_m"] is None else round(last["jump_m"], 3)},
            "max_jump_m": round(max(jumps), 3) if jumps else None,
            "gate": None if g1 is None else {"accepted": g1.get("accepted"), "rejected": g1.get("rejected"),
                                             "rejected_since_start": since},
            "strip": self.strip_codes(cells),
        }

    def strip_codes(self, cells=120):
        """The last STRIP_S as `cells` codes, oldest first: '.' no frame, T tracking, S/F/M lost."""
        if not self.strip:
            return ""
        t0 = self.strip[-1][0] - STRIP_S
        out = ["."] * cells
        for t, c in self.strip:
            i = min(cells - 1, max(0, int((t - t0) / STRIP_S * cells)))
            if out[i] in (".", "T"):                 # a lost sample wins its cell
                out[i] = c
        return "".join(out)

    def report(self):
        losses, lines, seen = self.all_losses(), [], set()
        if not losses:
            lines.append("vo_state never went to 2: cuVSLAM kept tracking the whole recording.")
        for i, lo in enumerate(losses, 1):
            odom = sorted(lo["odom"].items(), key=lambda kv: -kv[1])
            text = ", ".join(f"{k} {v:.1f} s" for k, v in odom) or "no frames"
            jump = "no jump" if lo["jump_m"] is None else f"a {lo['jump_m'] * 100:.0f} cm jump"
            lines.append(f"loss {i}: {lo['end'] - lo['start']:.1f} s, starting {lo['start'] - self.t0:.1f} s in; "
                         f"odometry {text}; {jump}")
            if odom:
                seen.add(odom[0][0])
        lines += [f"  means: {MEANS[s]}" for s in ("stopped", "frozen", "moving") if s in seen]
        if any(lo["jump_m"] for lo in losses):
            lines.append("  means: the pose jumped around a loss; in Gazebo a 15 cm jump at the tube exit "
                         "left 2.4 cm at the window")
        g = self.summary()["gate"]
        if g is None:
            lines.append("Koji's gate: no /mapping/gate_status in the recording (not running)")
        else:
            extra = "" if g["rejected_since_start"] is None else \
                f", {g['rejected_since_start']} of them during the recording"
            lines.append(f"Koji's gate: accepted {g['accepted']}, rejected {g['rejected']}{extra}")
        return lines


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    test, n = TubeTest(), 0
    with open(argv[1]) as fh:
        for line in fh:
            try:
                f = json.loads(line)
            except ValueError:
                continue
            if isinstance(f, dict) and "img" not in f:
                test.feed(f)
                n += 1
    print(f"{n} frames from {argv[1]}")
    for line in test.report():
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
