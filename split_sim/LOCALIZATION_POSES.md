# Passive live and replay localization

The preview receives a separate, derived pose dataset for each run. It contains actual NVIDIA raw stereo VO, bridge-admitted VO, PX4 fused local position and an independently labelled truth reference. It does not rerun an estimator, alter a flight, change a grade or fit an estimate to truth.

## Install and test without a flight

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) on the Mac. The scripts carry PEP723 dependency metadata; `uv run --script` creates an isolated environment with pinned `pyulog==1.3.0`. Live mode does not import pyulog or parse an active ULog. Python3.11+ is required.

```bash
uv run --script split_sim/test_localization_poses.py
bash -n split_sim/run_split_course.sh split_sim/localization_worker.sh
python3 -m py_compile split_sim/build_localization_poses.py split_sim/split_record.py
```

Tests create temporary, clearly synthetic metadata and a mocked ULog reader. No recordings, GPU, ROS, SSH, flight authority, private host paths or real truth are needed. The runner's normal receive-only entrypoint is exercised by the tests:

```bash
bash split_sim/run_split_course.sh --localization-only /absolute/RUN_DIRECTORY --once
bash split_sim/run_split_course.sh --localization-only /absolute/RUN_DIRECTORY --watch
```

This branch executes before network/container/flight setup. Watch mode uses one run-local advisory lock, nice priority10,5Hz display snapshots and a10-second update interval. It stops after a terminal grade or signal. A duplicate watcher exits without starting a second worker. `localization-worker-state.json` records its lifecycle.

## Normal run integration

The existing runner starts the worker beside its passive mirror. Cleanup stops/reaps it; the closed finalizer builds the replay from final `flight/telemetry.jsonl` and the archived PX4 ULog. The recorder adds actual `local_position` and fusion observations, uncertainty/reset fields and wall/monotonic receipt timestamps. It publishes nothing.

Updates use incremental JSONL tails, cache immutable runtime inspection, defer file hashes until sealed audit and atomically replace output files. Final audit prefers `telemetry.jsonl` over a stale `live-telemetry.jsonl`. Missing sources remain explicitly unavailable; zero or absent camera covariance means unknown confidence.

## Frame and time contract

The run root contains `localization-live.json` while live, or `localization-poses.json` after a grade. A previous live filename becomes a sealed replay alias, with `mode: replay`; it must not appear as a current sensor stream. Every source preserves acquisition timestamp, native pose and frame, uncertainty, validity, provenance and sample age. Display snapshots hold only the latest source observation at or before the requested simulation tick. Samples older than0.5s are marked stale. Source wall receipt age is separate from simulation age and browser refresh time; unknown historical wall receipts are null.

Course overlays use only declared mission fixture coordinates and the preflight bearing/datum already frozen by the mission. The frozen source default launch height is accepted only with an archived invocation that uses it. Covariance remains explicitly in its native frame. Truth is a separately labelled independent reference and is never used for a transform or navigation.

A return leg is not registered from its requested pad center. Its origin is withheld until a frozen `return-manifest.json` supplies the measured `segment_registration.world_from_odom` matrix. Without the required metadata, native estimate axes remain unregistered. This producer is compatible with return-capable runners; it does not introduce a new flight leg or modify controller behavior.

Named derived artifacts carry a title, source flight IDs, baseline ID, What changed and limitations. They do not overwrite original flight comparison entries. Declared simulation fixture coordinates are not surveyed hardware registration, and this viewer does not establish hardware qualification.
