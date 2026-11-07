# Packaging verification (2026-11-07)

Base: upstream main `e4a26c8`. Isolated clean clone; no live workspace changes,
Docker builds, simulator restarts, flight commands or aircraft connections.
Only `local_course/` is added. Prior runtime observations are in README and were
not reproduced by this packaging.

Commands and results:

- Initial system `python3 -m pytest tests -q`: unavailable (`No module named pytest`).
  Created a disposable host venv with pytest/numpy/scipy/opencv-python.
- Venv `python -m pytest tests -q`: **11 passed, 4 skipped**. Existing optional
  integration/dependency skips; no grader or upstream tests changed.
- Initial package-qualified unittest invocation found legacy sibling-style imports
  in scorer/replay tests. Updated only test imports to `local_course.*` for the
  actual repository layout. Final `python -m unittest local_course.bridge_test
  local_course.score_test local_course.vo_test local_course.replay_test -q`:
  **32 tests passed**, covering fail-closed gates, clocks, frame/covariance math,
  image tracking loss/recovery, calibration and independent course rejection.
- All packaged Python files parsed with `ast.parse`; all shell scripts passed
  `bash -n`; viewer JavaScript passed `node --check`.
- `docker compose -f local_course/sim-compose.yaml config --quiet`: passed.
  This parses configuration only; no runtime lifecycle operation.
- Disposable loopback HTTP server: root/app/course/ideal/catalog returned 200;
  traversal, secret-like filenames and out-of-run symlinks returned 404.
  A synthetic passing grade without observed flight was not classified as a pass;
  adding an armed/airborne sample enabled the synthetic classification.
- Source/path review: repo mounted from `..`, build state from `state/home`,
  checked-in asset/range adapters. No sibling Jetson runtime, private path,
  session id, raw recording or credential dependency. Runtime Docker network is
  internal and only noVNC publishes on loopback. Raw output is ignored by Git.

Not run: fresh Docker build, clean-runtime sensor smoke, timing/rate qualification,
OBJ observer against this new checkout, flight or hardware validation. The draft
must retain those limits. Before readiness, repeat setup/start on an idle Mac and
capture image/source identity, actual timestamps/TF/calibration, stationary gate
results and shutdown evidence. Do not treat syntax/unit tests as moving-flight
qualification. Three previous airborne attempts yielded zero course passes.
