"""Regenerate the three PNGs with Python, matplotlib and numpy. Run from any directory."""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

HERE = Path(__file__).resolve().parent
INK, BLUE, ORANGE, GREEN = "#1b3145", "#126d98", "#bc541d", "#197255"
plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.spines.left": False, "axes.spines.bottom": False, "text.color": INK,
                     "axes.labelcolor": INK, "xtick.color": INK, "ytick.color": INK})


def load(run):
    # A partial, NUL-containing final line in the yaw CSV is retained on disk.
    lines = (HERE / "logs" / f"{run}.csv").read_bytes().splitlines()
    rows = list(csv.DictReader(line.decode() for line in lines if b"\0" not in line))
    return {k: np.array([float(r[k]) if r[k] else np.nan for r in rows])
            if k in {"t", "armed", "offboard", "landed", "px4_z", "sp_z", "px4_yaw_deg",
                     "sp_yaw_deg", "px4_age_s", "odom_age_s", "sp_vz"}
            else np.array([r[k] for r in rows]) for k in rows[0]}


def canvas(run, title, n):
    fig, axs = plt.subplots(n, 1, figsize=(10, 2.05 * n + 1.35), squeeze=False)
    fig.subplots_adjust(left=.12, right=.965, top=.81 if n == 2 else .73, bottom=.14, hspace=.70)
    fig.text(.12, .95, title, fontsize=21, weight="bold", va="top")
    fig.text(.12, .875 if n == 2 else .83, f"Skybeta-19   /   {run} UTC   /   seconds from first log row",
             color="#526879", fontsize=10)
    for ax in axs.flat:
        ax.grid(axis="y", color="#dce4e7")
        ax.set_axisbelow(True)
        ax.tick_params(length=0, pad=6)
    return fig, list(axs.flat)


def save(fig, name):
    fig.savefig(HERE / "images" / f"{name}.png", dpi=160, facecolor="white")
    plt.close(fig)


def states(ax, t, data, keys):
    for i, (key, color) in enumerate(zip(keys, [BLUE, GREEN])):
        ax.fill_between(t, i-.22, i+.22, step="post", color="#edf1f3")
        ax.fill_between(t, i-.22, i+.22, where=data[key] == 1, step="post", color=color)
    ax.set(yticks=range(len(keys)), yticklabels=[s.capitalize() for s in keys],
           ylim=(len(keys)-.5, -.5), xlabel="Elapsed time (s)")


run = "20261108-020116_hover"
h = load(run)
t = h["t"] - h["t"][0]
fig, (ax, flags) = canvas(run, "Hover reached its hold and returned to the ground", 2)
ax.plot(t, h["px4_z"], color=BLUE, label="PX4 estimate", lw=2)
ax.plot(t, np.where(h["sp_kind"] == "pose", h["sp_z"], np.nan), "--",
        color=ORANGE, label="Position target", lw=2)
ax.set(xlim=(0, 50), ylabel="Local height (m)", xlabel="First 50 s")
ax.legend(loc="lower center", bbox_to_anchor=(.51,-.04), ncol=2, frameon=False)
ax.yaxis.set_major_locator(MaxNLocator(4))
states(flags, t, h, ["armed", "offboard"])
flags.set_xlim(0, t[-1])
save(fig, "hover")

run = "20261108-020558_yaw"
y = load(run)
t = y["t"] - y["t"][0]
fig, (ax, ages) = canvas(run, "Clockwise turn followed by a descent command", 2)
# Convert the measured and requested heading to a continuous clockwise displacement.
start = np.flatnonzero(y["step"] == "climb")[0]
initial = y["px4_yaw_deg"][start]
for col, color, style, label in [("px4_yaw_deg", BLUE, "-", "PX4 estimate"),
                                  ("sp_yaw_deg", ORANGE, "--", "Heading target")]:
    use = np.isfinite(y[col]) & (t >= t[start]) & (t <= 34.2)
    d = np.degrees(np.unwrap(np.radians(y[col][use]-initial)))
    ax.plot(t[use], -d, style, c=color, lw=2, label=label)
ax.set(xlim=(0, 35), ylabel="Clockwise turn (°)", yticks=[0, 90, 180], xlabel="First 35 s")
ax.legend(loc="upper left", ncol=2, frameon=False)
ages.plot(t, np.where(y["px4_age_s"] > 0, y["px4_age_s"], np.nan), c=BLUE, lw=1, label="PX4 position age")
ages.axhline(.5, ls="--", c=ORANGE, label="0.5 s freshness limit")
ages.set(xlim=(0,t[-1]), yscale="log", ylabel="Sample age (s)", xlabel="Full readable log (s)")
ages.legend(loc="upper left", ncol=2, frameon=False)
save(fig, "yaw")

run = "20261108-020501_forward"
f = load(run)
t = f["t"]-f["t"][0]
fig, (ax,) = canvas(run, "Forward test waited for Offboard throughout", 1)
states(ax, t, f, ["armed", "offboard"])
ax.set_xlim(0,t[-1])
ax.text(17, .05, "Armed for about 10 s", va="center")
ax.text(17, 1.05, "Offboard stayed off", va="center")
save(fig, "forward")
