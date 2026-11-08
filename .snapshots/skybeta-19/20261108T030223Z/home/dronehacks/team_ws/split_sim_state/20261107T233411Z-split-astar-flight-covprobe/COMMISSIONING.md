# Pilot-owned paired passive commissioning

Prepare only. Do not interrupt a moving or swept trial. The pilot chooses an already grounded/disarmed preflight interval; this probe never arms, changes a parameter, resets a source or publishes an estimate. No SSH or target process was started by the preparing agent.

`passive_pair_probe.py` reads actual raw Odometry, tracking status, ROS clock, PX4 ground state and existing gate diagnostics. It independently evaluates the pinned original `valid_odometry` function and a covariance-only canonicalized copy. Each genuine source packet retains exact pose/twist covariance, acquisition and receipt times, raw serialized CDR plus SHA256, original/candidate decisions, decision latency and byte-exact preservation checks for all non-covariance fields. Original input serialization must remain unchanged.

The original function is extracted from its SHA-verified source AST; its viewer, camera/GPU modules are never imported. The candidate imports only NumPy. BLAS/OpenMP threads are set to1 before import, Linux CPU affinity pins the entire new process/thread inheritance to one allowed CPU, and nice10 lowers priority. Only subscriptions are created by application code; rosout and parameter services are disabled. Ordinary ROS2 internal discovery/metadata is not a new sensor or control stream.

The probe requires domain219, fresh landed/disarmed PX4 state and actual tracking evidence. An armed/not-landed state ends it immediately. It exits successfully only after seeing both a real original/candidate-valid nonnegative source packet and a real roundoff-negative source packet rejected by the original but accepted by the candidate, with exact non-covariance preservation. If no genuine negative packet occurs, the result is incomplete(exit2), not a fabricated negative-source pass. Separate material-negative and0.528s-stale controls are explicitly in-process mutations and are never published. A failure in either hard-fail control prevents success.

## Source identity and command

Verify the staging archive's `sources.sha256` before running. The expected original local source hash is not a claim that the target was inspected: the probe checks the installed target file and stops if it differs. On mismatch, stop and review the installed function; do not automatically replace the expected hash.

In the existing simulation companion container, after the pilot has loaded its ordinary ROS environment and copied the staging directory without modifying the active viewer:

```bash
export ROS_DOMAIN_ID=219
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 /root/split_state/covariance_candidate/passive_pair_probe.py \
  --candidate /root/split_state/covariance_candidate/roundoff.py \
  --candidate-sha d219b93e3b3714603f0f38284a6e2dfce5c765e99248dbf904ef16cc6758e25d \
  --original /split/viewer/map_server.py \
  --original-sha d41055150089ed55ab0ac3ed496190c663a8661077d706266bf017395c589b61 \
  --output /root/split_state/REVIEWED-RUN-covariance-pair.jsonl \
  --wall-seconds 20
```

Estimated wall time: up to20s of passive acquisition plus roughly1–2s ROS/NumPy startup, potentially shorter if the real pair is observed. No GPU process, image processing or flight work is started. A pilot-imposed external25s timeout may bound initialization as well. Output is created exclusively; choose a new filename per attempt.

Read `probe_summary`: require `paired_actual_source_complete:true`, both actual categories, hard-fail controls true, unchanged input/non-covariance fields on every packet, source SHA equality and acceptable measured CPU latency. This is an interface commissioning check, not continuous-flight approval or proof of source accuracy. Root/pilot own any subsequent candidate deployment and separately approved continuous attempt once live health is known. Preserve original failed grades and all source/candidate evidence.
