"""Pass tof_driver --view frames through unchanged and log one CSV row per frame: unix time, left cm, right cm
(empty = nothing seen). usage: python3 -u tof_driver.py --view | python3 -u log_view.py OUT.csv"""
import os
import re
import signal
import sys
import time

signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C stops the driver; this ends when its output ends
CLEAR = "\x1b[2J\x1b[H"
VALUE = re.compile(r"(LEFT|RIGHT)\s+(--|\d+\.\d) cm")
log = open(sys.argv[1], "a", buffering=1)
log.write(f"# start {time.time():.2f}\n")
buf = ""
while True:
    data = os.read(0, 65536)
    if not data:
        break
    text = data.decode("utf-8", "replace")
    sys.stdout.write(text)
    sys.stdout.flush()
    buf += text
    *frames, buf = buf.split(CLEAR)
    now = time.time()
    for frame in frames:
        vals = dict(VALUE.findall(frame))
        if len(vals) == 2:
            row = ["" if vals[k] == "--" else vals[k] for k in ("LEFT", "RIGHT")]
            log.write(f"{now:.2f},{row[0]},{row[1]}\n")
