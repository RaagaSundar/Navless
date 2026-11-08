# Real Jetson flight package

`jetson/` is the complete team payload to copy to Skybeta 19. `mac/` contains the laptop dashboard. The dashboard renders on the Mac so the aircraft controller can continue independently when Wi-Fi or the browser disconnects.

| Directory | Entry point |
| --- | --- |
| [Jetson payload](jetson/README.md) | `jetson/run.sh` selects the passive input check, health adapter, or course controller in the existing physical stack. |
| [Mac dashboard](mac/dashboard.py) | `python3 mac/dashboard.py` opens a loopback listener in offline mode. Visit <http://127.0.0.1:8093/>. |
| [Local checks](tests/) | From this folder run `python3 -m pytest -q tests` with NumPy and pytest installed. |

### Aircraft and planner versions

The original hover/yaw controller at `20aaee7913cc608ae524eeecdecb53af9e8d7055` matched the archived installed controller byte for byte. Its pose and twist topics, ENU frame and RC handover define the compatibility target. The recordings have no embedded Git hash. The course executor here comes from the separate aircraft course revision `865db6b995f20c16164786b57ff7dcb9c66eeaa9`. Its height-offset correction, route following, arrival checks and descent logic remain in the original source. The new wrapper adds current state checks and latches loss of authority.

The window planner and packed course occupancy map are from the active split-simulation agent’s revision `7a7883250c6e783c357a14d56ed9626d7300ba58`. The staged hardware configuration uses its 97 precomputed course-frame waypoints. The active simulator also has an online stereo-cloud replanning path. This first physical profile uses a fixed route and preserves the aircraft controller’s heading policy. See [source identities](jetson/upstream.json) and the [hardware run instructions](jetson/README.md).

### Dashboard

The default offline mode makes zero SSH connections. After the team enables hardware monitoring, establish an authenticated OpenSSH session for the assigned host. A dedicated control socket avoids an SSH handshake on every update. From this Mac:

```sh
ssh -M -S ~/.ssh/skybeta19-dashboard.sock -N dronehacks@skybeta-19.local
```

In a second Mac terminal, run:

```sh
python3 mac/dashboard.py --live --control-socket ~/.ssh/skybeta19-dashboard.sock
```

The live reader checks the hostname on each read and consumes the organiser’s existing `/run/skybeta-status/ros_summary.json` at about 1 Hz. It verifies domain 19 and measures source age in the Jetson clock. Control state is unknown while the FC link is down. The map shows recent cuVSLAM and PX4 local positions in their reported frames, without registering them to the physical course. The live page serves read-only data on Mac loopback. Stop the console and the dedicated SSH session with Ctrl-C in their respective terminals.
