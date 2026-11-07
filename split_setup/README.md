# Mac ↔ Jetson split setup handoff

This additive follow-up to `local_course/` records the **installed split profile**
inspected on 2026-11-07 and adds a passive isolation doctor. It does not install
that runtime: its deployment scripts and NVIDIA dependencies are not bundled in
this repository. A fresh clone cannot run the split profile from these files.
Use [the self-contained Mac fixture](../local_course/README.md) for that path,
and [the Jetson-local runbook](../docs/JETSON_SIMULATION.md) for NVIDIA rendering.

| Profile | Rendering / perception | ROS / ABI | Evidence identity |
| --- | --- | --- | --- |
| Mac local course | Mac Docker Gazebo / CPU stereo VO | 220; Jazzy ARM64, Python 3.12 | Mac SIM |
| Mac ↔ Jetson split | Mac Gazebo + PX4 SITL / Jetson NVIDIA cuVSLAM | 219; Mac Jazzy; Jetson Humble ARM64, Python 3.10 | NVIDIA SIM, split renderer |
| Jetson stationary bench | Jetson NVIDIA EGL Gazebo + cuVSLAM | 219; Humble ARM64 | NVIDIA SIM, Jetson renderer |
| Physical aircraft | Real sensors / existing aircraft stack | 19 | Hardware; never classify as SIM |

Do not transfer binary ROS/Python/cuVSLAM dependencies between those ABIs. Domain
219 profiles are mutually exclusive. Domains 220/221/222 also have separate
owners; domain separation does not reserve CPU/GPU or authorize a lifecycle
change. Leave the physical stack and its UART/camera devices untouched.

## Passive discovery before startup

The doctor uses only Docker list/inspect on its **local host**. Copy this small
file to the Jetson through the reviewed operator workflow, then run there:

```bash
python3 doctor.py --expected drone19-split-sim
```

It exits 0 only when the expected container runs, no competing domain-219
profile runs, domain/localhost settings match, and the inspected container
reports no privileged or unexpected device access. Exit 1 means an isolation
check failed; exit 2 means inventory could not be read. This is a configuration
check, not a complete security audit: NVIDIA runtime can expose GPU devices
without listing them in Docker's `Devices` field. Docker inspection is a snapshot;
repeat immediately before a coordinated switch. Do not save or share raw
`docker inspect` output: it can contain credentials, environment and host paths.
The report deliberately emits only allowlisted fields.

For offline review, `--inspect-json FILE` accepts Docker's array of container
objects. Unit tests use synthetic records; they cannot establish live readiness.
Running this on the Mac cannot inspect Jetson containers.

## Reviewed startup sequence (installed deployment only)

1. Agree on domain-219 ownership and inspect both containers on the Jetson.
   Preserve logs/maps and confirm the stationary bench has been released. The
   doctor never stops a conflicting workload. After keyboard RC, land/release
   RC and use **Reset world** before any automated run.
2. In the operator-provided split deployment directory on the Mac, inspect
   `start.sh`, `compose.yaml`, `run_laptop.sh` and `run_jetson.sh` against the
   source identities below. `bash start.sh --split` is a **profile switch**: it
   stops `drone19-gazebo-bench`, starts Mac Compose and `drone19-split-sim`, and
   starts the renderer if absent. Plain `bash start.sh` preferentially starts
   the Jetson-local bench when installed; it is not equivalent.
3. Authenticate with the operator-provided password interactively. Reusable SSH
   ControlMaster sockets are authenticated local capabilities: verify host key
   and target, restrict access to the operator account, never commit/copy the
   socket or password, and do not disable host-key verification. The deployed
   launcher has a fixed socket location; inspect that actual script locally.
   If mDNS changes routes, have the operator verify the host identity/address
   before using an explicit target. Do not silently change network settings.
4. Verify tunnel listeners and actual content. The deployed launcher forwards
   reverse loopback TCP 17447 for the allowlisted camera/Clock transport; map
   18765→8765, stereo 18081 and CAD 18082 forward to laptop loopback. Mac
   noVNC is loopback 6081. An existing forward warning is not success: verify
   endpoints and advancing stamps. `/clock`, calibration/TF and images cross
   the transport; native DDS remains localhost-only on each host.
5. Repeat the Jetson doctor and collect at least 30 seconds of actual images,
   CameraInfo, Clock, tracking state and validated odometry. Confirm calibrated
   stereo dimensions/baseline/frame IDs, advancing source stamps, wall receipt
   freshness, tracking and simulation-time gate age. Topic/node names and a
   loading viewer do not establish those properties. Use installed passive
   collectors after reviewing their source; the Jetson-local collector in the
   other runbook has a different renderer/adapter and cannot qualify split IO.
6. Only after the sensor check, the installed Mac command
   `bash start_px4_from_mac.sh` attaches compiled SITL to the running world.
   Inspect its resolved Jetson address (`JETSON_IP` override), UDP 8889 reachability
   and current source first. Inspect fresh disarmed/landed PX4 reports, local
   position/estimator flags and actual EV data separately. This command starts
   SITL; it does not arm. A moving test still needs the team's explicit plan
   approval. Do not change real PX4 parameters or supply oracle odometry.
7. Preserve failures and per-run ULogs. Yield using the installed `stop.sh`
   only after reviewing its target containers/tunnel; do not remove state or
   stop another owner's workload. This PR did not execute these lifecycle steps.

The split runtime uses stereo-only cuVSLAM → unchanged odometry gate → PX4 EV
bridge and an XRCE UDP agent. It models a 640×400 mono8 pair at 10 Hz simulation
time, 0.075 m baseline and 80° horizontal FOV. These are simulated assumptions.
The separate downward-range model uses truth solely as sensor generation; that
oracle must remain outside portable navigation. CAD localization is an observer.

## Evidence and viewer integration

Keep Mac SIM and NVIDIA SIM archives in distinct run directories. Alongside each
archive record: profile, renderer host class, ROS domain/ABI, source commit/hash,
container image digest, calibration/TF, sensor assumptions/oracle use, UTC capture
window, wall duration, simulated duration, clock ratio, transport, raw/validated
counts, tracking losses, sample-age statistics, PX4 state and grader outcome.
Keep failed and incomplete runs. Avoid private paths, addresses, credentials,
session IDs and raw bags in Git. Use synthetic fixtures for shared tests.

The `local_course/viewer/` reads that fixture's archive schema; the installed
Jetson live map/depth/CAD viewers have different APIs. This PR does not convert
those schemas or claim they interoperate. Label the live viewer as NVIDIA SIM
and renderer type; use an explicit adapter and retain provenance before importing
an NVIDIA archive into the Mac catalog. Never infer a course pass from a live
map, stationary health check or process exit code.

## Inspected deployment source identities

SHA-256, recorded from the operator workspace on 2026-11-07; these establish
what was read, not a promise that the live deployment still matches:

| Script | SHA-256 |
| --- | --- |
| `start.sh` | `480733f5f8c3ab964461b22bf4d54caeceae312ae0a006c3d4cfbafe73a42564` |
| `compose.yaml` | `903fdb9750849432164e9b0215befae03745f9902c0f0297321a8fd2cec8ee6e` |
| `run_laptop.sh` | `b878a0633dcebcde64f06572309b245277da62736d1c789bdd51d48624c4e18e` |
| `run_jetson.sh` | `5051cb3c9f0e69acdc41e4e85ec5a04be8d47d9673fc101b68470cb62acdba25` |
| `start_px4_from_mac.sh` | `da192b2769742cc63b46b09c1405c2a5524676777324c0982a22df02f5f8022c` |

No clean installation, build, startup, sensor smoke, flight or hardware test was
performed for this follow-up. Prior stationary/disarmed results in the linked
runbook are observations of those earlier deployments, not qualification of
this checkout.

## Install the wrong-interface fix in an existing split deployment

A multi-interface Jetson can advertise an unreachable mDNS IPv4 while an
already authenticated SSH connection uses a working wired address. Using a new
mDNS lookup for XRCE then silently selects the wrong interface. The portable
`split_network.sh` helper reads the **server address (third field) of the
remote `SSH_CONNECTION`** through authenticated SSH. `JETSON_IP` remains the
highest-priority explicit XRCE IPv4 override; it does not require SSH lookup.
Malformed/missing data or SSH failure stops startup rather than guessing an IP.
Link-local IPv4 is accepted; multicast, wildcard, loopback and IPv6 are rejected.
An SSH-confirmed address does not prove that SITL's UDP can reach that address:
retain the passive XRCE/PX4 checks above.

On an existing reviewed deployment, install only the file patch:

```bash
python3 split_setup/install_network_fix.py /path/to/reviewed/split-deployment
```

The installer validates both known launcher blocks before writing, refuses an
unknown revision and can be repeated. It installs the helper and changes only
`start.sh` and `start_px4_from_mac.sh`; it does not run Docker/SSH, restart
processes, install packages, alter routes or require sudo. Keep the helper next
to both launchers. Reinspect changed sources; the historical launcher hashes
above describe the version **before** this patch.

Optional environment settings, inherited by either launcher:

| Setting | Meaning |
| --- | --- |
| `JETSON_IP` | Explicit literal IPv4 for XRCE; takes precedence over SSH server address |
| `JETSON_SSH_TARGET` | Explicit reviewed `user@host-or-IP`; takes precedence over default mDNS and disables fallback |
| `JETSON_SSH_FALLBACK` | Optional reviewed `user@IP` tried only if default mDNS SSH fails; no built-in address scan or guessed fallback |
| `JETSON_HOST_KEY_ALIAS` | Known-host identity, default `skybeta-19.local`; strict checking stays enabled for IP targets |
| `JETSON_SSH_SOCKET` | Operator-owned ControlMaster socket; defaults to the existing deployment's socket for compatibility |

For example, an operator who independently verified the wired address and its
known host key can set `JETSON_SSH_TARGET=dronehacks@<reviewed-wired-ip>` before
running the installed `start.sh --split`. Never paste an unverified IP into this
setting, disable host checking, or blindly accept a new key. The helper reuses an
existing authenticated socket if available; verify its owner and destination.
If a socket is stale or belongs to another target, have its owner close it and
choose an unused operator-owned socket. The helper does not delete sockets.
The SITL launcher uses BatchMode for address discovery; establish the reviewed
interactive SSH connection with `start.sh` first when needed.

Validation uses mocked SSH only: explicit override precedence, server-address
selection despite different advertised interface, malformed/failed discovery,
strict host alias options, reviewed fallback and installer refusal/idempotence.
It does not initiate an SSH session or change running simulations:

```bash
python3 -m unittest split_setup.test_doctor split_setup.test_network -v
bash -n split_setup/split_network.sh
```
