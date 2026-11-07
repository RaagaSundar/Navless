# Private laptop simulations on a shared Jetson (experimental)

Each operator runs Gazebo Harmonic + PX4 SITL on their own Apple Silicon laptop. A single NVIDIA Jetson runs **one** cuVSLAM + onboard-code session at a time. Other authenticated laptops queue FIFO; there is one outstanding ticket per assigned operator. The wait limit is 15 minutes and the active limit is 30 minutes. There is no simultaneous GPU-flight claim. This is a separate launcher; it neither starts nor stops the legacy split stack.

A laptop sends its simulated camera stream and bidirectional XRCE UDP datagrams through two reverse SSH forwards. Both the laptop and Jetson containers use Docker network mode `none`, so neither has an Ethernet interface or route to the aircraft. The helper on the Mac connects the inner SSH client's **private Unix sockets** to the laptop container using `docker exec -i`; there are no published Docker ports. The Jetson's inner sshd runs on stdin/stdout inside its container. It does not listen on the Jetson LAN. Gazebo truth is not a pose source for the onboard program. The stack uses the split camera-only cuVSLAM profile, existing odometry validity gate and PX4 odometry bridge. No keyboard pilot, GUI, inertial profile or Grader integration is provided.

Slot `n` (1–16) gets ROS domain `70+n`, Gazebo partition/container stem `navless-slot-NN`, camera reverse port `22001+10*n`, XRCE reverse port `22002+10*n`, and local UDP agent port `22003+10*n`. The ports bind to loopback in a private namespace. The laptop agent transport accepts the first local PX4 source and refuses other peers until disconnect. The sole user process is a nonroot `python3 main.py` in a new work directory on that slot's bounded volume. Applications can start their own children; only run a reviewed application with one commander. The Docker container, including all its children, is removed on exit. Port/domain separation is not relied upon as the access boundary.

## Administrator setup (Jetson; review and provision separately)

Requires a coordinated simulation window, an approved JetPack-compatible `skybeta:0.3.3` with the existing NVIDIA ROS stack, Docker's NVIDIA runtime, and the [portable split_sim package](../split_sim/README.md) in the same repository as this directory. Do not use the physical aircraft service account's keys or give participants Docker group membership. Never run the legacy split launchers during a session window: they do not participate in this scheduler. Keep `ENABLED` absent during hardware use. These commands are for the existing system administrator; the software does not enroll anybody.

1. In the repository root, build a local *simulation image* from the reviewed sources (the base and Ubuntu package downloads are installation-time prerequisites). Install the gateway under a root-owned path. Use a dedicated locked-password service identity, shown as `navless-gw`, authorized solely by root-managed forced commands. It needs Docker access; this is an administrator-trusted account, not a login or shell assigned to teammates. Do not share its password, SSH key, or Docker socket. For example, as administrator:

   ```bash
   test -f split_sim/sim_slam.launch.py
   docker build -f sim_sessions/Dockerfile -t navless-slot:reviewed .
   sudo useradd --system --create-home --shell /bin/sh navless-gw
   sudo usermod -aG docker navless-gw
   sudo install -d -m 0755 /opt/navless-sessions /opt/navless-sessions/slots
   sudo install -m 0755 sim_sessions/gateway.py /opt/navless-sessions/
   sudo install -m 0644 sim_sessions/layout.py /opt/navless-sessions/
   docker image inspect -f '{{.Id}}' navless-slot:reviewed | sudo tee /opt/navless-sessions/image-id >/dev/null
   sudo install -d -o navless-gw -m 0700 /var/lib/navless-sessions
   ```

2. Assign a **different** slot and public key per operator, verifying the fingerprint out of band. Example for slot 1, with the verified public key in `operator.pub`. It must be a plain public key (no leading options). Never copy a private client key to the Jetson. Slot host keys are distinct; send the public known_hosts line to that operator through an authenticated channel.

   ```bash
   slot=navless-slot-01
   sudo install -d -m 0755 "/opt/navless-sessions/slots/$slot"
   sudo install -m 0644 operator.pub "/opt/navless-sessions/slots/$slot/client.pub"
   sudo ssh-keygen -q -t ed25519 -N '' -f "/opt/navless-sessions/slots/$slot/host_key"
   sudo awk -v name="$slot" '{print name, $1, $2}' "/opt/navless-sessions/slots/$slot/host_key.pub"
   docker volume create --driver local --opt type=tmpfs --opt device=tmpfs \
     --opt o=size=128m,mode=0755 "${slot}-work"
   ```

   In the server's **root-managed** `AuthorizedKeysFile` for this gateway account, put one line per approved key, e.g. `restrict,command="/usr/bin/python3 -I /opt/navless-sessions/gateway.py 1" ssh-ed25519 PUBLIC_KEY_MATERIAL`. The forced command is fixed by the administrator and ignores everything supplied in `SSH_ORIGINAL_COMMAND`. Use a dedicated sshd `Match User navless-gw` policy: public-key-only authentication, `DisableForwarding yes`, `PermitTTY no`, `PermitUserRC no`, no accepted custom environment, no alternate authorized-keys commands, trusted CAs, password or keyboard-interactive path into this account. Verify effective config with `sshd -T -C user=navless-gw,host=localhost,addr=127.0.0.1` before enabling service. Verify a forced key cannot get a host shell or forward to any host port. Do not make application code, gateway files, slot config, authorized keys or image ID writable by participants or the gateway identity.

3. Inspect active workers, available memory and GPU use. Confirm other operators have released the physical/legacy compute window. Allow only the named slot containers. Once ready, `sudo touch /opt/navless-sessions/ENABLED`. Remove this file to prevent further admissions; it does not kill a running session. The gateway also refuses running legacy split/bench containers and any orphaned session. For the first session, use a passive telemetry reader; verify camera pairs, calibration/TF, clock, PX4 simulator identity, estimator checks and shutdown before planning a separate simulated flight. There is no arming approval implicit in installing or starting a session.

## Laptop install and run

Check out the same reviewed source, including `split_sim`, the course models and laptop image recipe. Use native macOS Python 3.9+ and OpenSSH, Docker Desktop with sufficient memory, and the prepared Jazzy/Harmonic image. Each operator prepares a **dedicated** state directory named exactly for their slot. Do not point it to the team's active simulator state. From the repository root, example for slot 1 (builds may take a while):

```bash
export SLOT_STATE="$HOME/navless-sims/navless-slot-01"
mkdir -p "$SLOT_STATE"
docker build -t dronehacks-local:jazzy-harmonic local_sim
docker run --rm --cpus=2 --memory=6g --entrypoint bash \
  -v "$SLOT_STATE:/root" -v "$PWD:/workspace/Navless:ro" \
  dronehacks-local:jazzy-harmonic /workspace/Navless/local_sim/bootstrap-px4.sh
```

Create a local SSH alias in your own SSH config; substitute the actual approved address and your own private key path. Verify the **outer** host key out of band in your normal known_hosts file. Save the administrator-provided **inner** slot key line in `~/.ssh/navless-slot-01.known_hosts`.

```sshconfig
Host my-sim-gateway
    HostName YOUR_APPROVED_JETSON_HOST
    User navless-gw
    IdentityFile ~/.ssh/navless_slot_01
    IdentitiesOnly yes
    ForwardAgent no
    StrictHostKeyChecking yes
```

Put your own Python source in a dedicated folder with `main.py` at its root, or pass one Python file directly. The client accepts regular files only, up to 128 files and 2 MiB total. Never include credentials or personal data. Start with a passive probe:

```python
# main.py: list topics for 30 s without publishing any command
import time
import rclpy
rclpy.init()
node = rclpy.create_node("session_probe")
for _ in range(30):
    print(sorted(name for name, _ in node.get_topic_names_and_types()), flush=True)
    time.sleep(1)
node.destroy_node()
rclpy.shutdown()
```

```bash
python3 sim_sessions/client.py --slot 1 --gateway my-sim-gateway \
  --key ~/.ssh/navless_slot_01 --known-hosts ~/.ssh/navless-slot-01.known_hosts \
  --repo "$PWD" --state "$SLOT_STATE" --program ./my-probe
```

Gazebo/PX4 start locally while waiting for a Jetson ticket. The program starts after SSH admission; it must wait for actual fresh estimator/vehicle/sensor messages itself. Stdout/stderr return to the laptop. The bounded onboard workspace is **ephemeral**; save desired output on the laptop, e.g. redirect stdout to a results file. PX4 and renderer logs remain in your dedicated laptop state. Each session starts fresh simulation processes; never use keyboard RC in this runner. No generic network upload, shell or pilot service runs on the Jetson.

## End and recover

For a later explicitly approved simulated mission, land/disarm before ending a normal session. End the foreground program or press Ctrl-C: SSH reverse listeners end with that connection and the owner removes only their laptop and UUID-named Jetson containers. The next waiting ticket is eligible after release. Network failure and stale SSH connections are bounded by keepalives, the queue timeout and the hard active runtime limit.

After a power cut or SIGKILL, do **not** restart physical services, remove unrelated containers, or delete queue locks manually. On a laptop inspect `docker inspect navless-slot-01-laptop` and its `navless.client` label, ensure no local runner owns it, then `docker rm -f navless-slot-01-laptop` if stale. On the Jetson close admissions, inspect `docker ps -a --filter label=navless.session`, confirm exact owner/UUID and that the session has ended, and remove *only* the confirmed orphan. Kernel flock handles abandoned queue tickets automatically. Reopen after inspecting compute/vehicle state. Revoke access by removing the operator's forced-key line, terminating any current session of that slot, and retiring its inner key.

## Verification and limits

On Apple Silicon macOS, Docker Engine 29.9.2, OpenSSH 10.5p1, and `ros:humble-ros-base-jammy` image ID `sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855`, run:

```bash
cd sim_sessions
TEST_DOCKER_SSH=1 python3 -m unittest -v test_sessions
```

Two live, independent laptop sandboxes (domains 84/85) used different pinned SSH keys, real reverse forwards, Unix sockets, network-none containers and framed two-way UDP payloads. Both returned distinct camera-channel payloads and bidirectional XRCE-channel datagrams; after killing one SSH client, its port and container were gone and the second returned a new payload. Scheduler tests exercised FIFO, duplicate admission rejection and dead queued/active processes. Source-bundle tests exercised multi-file transfer and traversal/symlink rejection. The first probe incorrectly expected `/sys/class/net` to contain only `lo`; Docker Desktop also lists **down** kernel tunnel devices. Verification now checks network mode and that **no non-loopback device is up**.

This proves transport/lifecycle behavior, not GPU capacity, actual Jetson sshd configuration, ROS delivery timing, simulated flight or hardware qualification. The nested Jetson image, real cuVSLAM startup and two physical laptops remain to be qualified on the installed JetPack and approved network. GPU code and timing can differ under memory pressure. Use one GPU slot only; these limits are experimental, not a guarantee that a GPU job will fit. Docker with a GPU is not a hostile-code sandbox: enroll only trusted teammates, not anonymous external users.
