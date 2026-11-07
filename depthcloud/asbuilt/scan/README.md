# How `scan_walls.json` was made

These scripts are kept so the wall positions can be traced back to the scan. You don't need them to build the as-built course: run `../make_asbuilt.py` for that.

**Input:** Shining's phone LiDAR scan of the real course, `dronehacks-course-scan-2026-11-06.ply`. It's a Scaniverse scan, 596k coloured points, 8.9 MB, and isn't in git; ask Shining for it. Run the scripts with Python 3, numpy, scipy and `cloud_localize.py` importable, from the drone's stack container or any laptop.

1. **`fit_walls.py scan.ply dronehacks-course-v3.obj`**
   - Levels the scan using the floor.
   - Aligns it (rotation plus east/north shift) using only the parts that match the plan: the net sides, Barriers 1–3, the window wall C2a and the tube. These agree within 0.3–6 cm.
   - Fits straight lines to the remaining wall-height points.
   - Writes `fit.json` and `fit.png`.
2. **`build_asbuilt.py scan.ply dronehacks-course-v3.obj out.obj`**
   - Picks which fitted lines make up M1, M2, M3, A1 and C1. Several walls appear 2–3 times, 10–30 cm apart, because the scan drifted as it was walked; these are combined into one line per wall, weighted by point count.
   - Joins each wall to the barrier or the net it touches.
   - Writes the per-wall result. That result became `../scan_walls.json`.
3. **`floor_view.py`** renders the floor in true colour, used to look for the taped guideline. It isn't visible in the scan: the floor is a uniform reddish mat with large unscanned gaps.

Shining confirmed the result against the real course on 2026-11-07. To correct a wall later, edit `../scan_walls.json`. Its `start_in` and `end_in` use `course_v3.json`'s frame: inches from the net's south-west corner, x east, y north. Then re-run `../make_asbuilt.py`.
