# Packaging verification

Checked2026-11-07 from a new clone of main`c967e27`, with no live simulation or
Jetson installation changes:

- `python3 split_sim/doctor.py --source-only`: every prerequisite present;32
  manifest-listed frozen runtime files match exact SHA-256.
- `python3 -m unittest split_sim.test_install -v`:2passed, including complete copy
  into a fresh space-containing folder and missing-prerequisite rejection.
- Host transport/admission/prepared-recorder tests:16passed.
- Frozen runtime tests:13passed,1skipped because the private failed-flight ULog
  is intentionally not distributed. The skip is not a synthetic pass.
-13shell syntax checks;58Python compilation checks; Docker Compose config parse
  passed. The command parsed config only, without starting containers.
- `bash split_sim/setup_jetson.sh --dry-run`: source checks and printed plan;
  no SSH/Docker action.
- `python3 split_sim/make_assets.py /tmp/navless-install-generated-assets`: public
  repository source generated the split camera/course assets successfully.
- Preview partner verified actual synthetic SQLite→onboard MP4/JPEG/manifest,
  OpenGL chase MP4 from generated public course assets, HTTP UI/catalog/JPEG,
  byte ranges206/416, POST501 and symlink containment; JS/Python syntax passed.

No heavy image/PX4 build, vendor-image provisioning, remote copy, CUDA test,
simulator lifecycle, arming or flight was performed for this packaging review.
No clean-install full-course success is claimed. The included213553 baseline
failed its height gate; corrected height/forward-window work must be referenced
separately before those variants are selected.
