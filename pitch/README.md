# NAVLESS: ZERO GPS. PURE AUTONOMY.

A scroll-driven 3D pitch deck for **skybeta-19**, the GPS-denied indoor UAV we are flying at DroneHacks.
Next.js 16 (App Router) · React Three Fiber 9 · drei · `@react-three/postprocessing` · Framer Motion · Tailwind 4.

Three scenes on one timeline, plus a 2-minute auto-demo and a judge-interactive finale:

| # | Scene | What happens |
|---|-------|--------------|
| 01 | **Hero reveal** | The Sky Beta CAD builds itself out of the dusk in dark liquid chrome (`MeshPhysicalMaterial`: metalness 1, roughness 0.1, clearcoat 1) lit by a low amber sun, brutalist type slides in *behind* it with real depth, one 360° turn on scroll, a sparse cuVSLAM feature cloud. |
| 02 | **The environment** | The drone shrinks to the corner, a phone-LiDAR-style point cloud resolves into the as-built DroneHacks maze (glowing `EdgesGeometry` wireframe). skybeta-19's real 14.2 m route is drawn as a glowing gradient line. Three callouts: **A\* + EGO B-spline planning**, **OAK-D-S2 stereo odometry**, **IR wall centring** (tunnel cutaway with live L/R time-of-flight beams). |
| 03 | **Override mode** | Scroll locks, camera lifts to an isometric top-down. **`[ INJECT DYNAMIC OBSTACLE ]`** arms the grid; click to drop a blocker (it falls in and settles with one soft ring). A real A\* + EGO planner re-plans in the browser (a few ms) and the new B-spline is drawn immediately while the invalidated one fades out in vermilion. Planner output scrolls through the live terminal. |

The **live telemetry terminal** (on-board stack lines mixed with the real planner's output) and the live clock exist **only in override mode**, the interactive part. The hero and the tour stay clean: no FPS counter, no ticking clock, no streaming text.

## Look and feel: future Mars, golden hour

Calm and readable first. Warm light, cool shadows, and nothing that flashes.

| Role | Colour | Used for |
|------|--------|----------|
| **World** (warm) | sand `#f1dcc0`, copper-gold floor grid, terracotta boulders, gold for the three newest walls | the maze, the pad, the net, the ground |
| **Route** (warm) | rust `#e0592a` → gold `#f6b25e` | skybeta-19's flown route |
| **Perception and plan** (cool) | ice `#9cc6dd` | the stereo view cone, tracked features, the live A\* + EGO plan |
| **Danger** | vermilion `#ff5a3c` | obstacles and the invalidated plan only, so a blocker is the one warning in the frame |
| **Type and UI** | warm white `#f6e9d9`, gold accent (ember in override) | headline, HUD, buttons |

- **Lighting** (`components/three/StudioLights.tsx`): a low amber sun behind the airframe (gold edge-light), a warm bounce on its face, a cool dusty-blue fill for the shadows, and an environment of a horizon band, a sun strip, a pale ice strip and a rose ground bounce for the chrome to reflect. The backdrop (`three/Sky.tsx`) is a static dusk gradient with a low sun glow, and `three/Grade.tsx` adds a gentle warm-highlight / cool-shadow grade. Bloom is soft and only catches the things meant to glow.
- **Motion rules** (so it can never strobe): no film grain or noise pass, no flicker, no glitch or tearing, no screen flashes, no camera shake (a blocker landing is a tiny settle), no random per-frame effects. The only looping animations are a 3.6 s glow breath on the main button and a 1.6 s soft caret fade.
- **Change the palette** in two places that mirror each other: `src/lib/palette.ts` (everything WebGL) and the tokens at the top of `src/app/globals.css` (everything HTML).

## Run it

```bash
npm install
npm run dev          # http://localhost:3000
```

```bash
npm run build && npm start     # production
npm run typecheck              # tsc --noEmit
npm run test:planner           # planner sanity + 300 random drops on the real course
```

Needs a browser with WebGL and hardware acceleration on (current Chrome / Edge / Safari). Without WebGL the page shows a plain notice instead of the scene.

## Two ways through it

- **Scroll.** Wheel / trackpad / touch / keys. Lenis smooths it. At the end of the track the scroll **locks** and override mode takes over.
- **`[ SEQUENCE INITIATED: PLAY DEMO ]`** (or press `D`). The `AutoDirector` flies a 2-minute path: the camera rides `THREE.CatmullRomCurve3` rails, a monotone cubic (PCHIP) maps demo time onto the curve so it glides *through* each shot instead of braking at every one. Every HTML reveal is locked to the same clock.

### Keys

| Key | Does |
|-----|------|
| `D` | start the demo (tour mode) |
| `1` `2` `3` | jump to scene 1 / 2 / 3 |
| `Space` | pause / resume the demo |
| `Esc` | abort the demo → disarm the grid → leave override (one level per press) |
| `I` | arm / disarm obstacle injection (override) |
| click on the grid | drop a blocker (when armed). Hover first: the tag shows the re-plan result before you commit |
| `P` | drop a blocker on the active path |
| `Z` / `C` | undo last / clear all blockers |
| `Enter` | engage the drone on the current plan |
| `R` | reset the drone |
| scroll up hard | leave override |

### Deep links

| URL | Effect |
|-----|--------|
| `/?p=0.5` | open at scroll position 0.5 (0 to 1) |
| `/?demo=1` | start the auto-demo as soon as loading finishes |
| `/?q=low` · `medium` · `high` | lock the render quality tier (default is adaptive: a `PerformanceMonitor` walks DPR, MSAA and the chromatic-aberration pass down if the GPU can't hold frame rate) |

## The 2-minute sequence

| Time | Shot | On screen |
|------|------|-----------|
| 0:00 | macro of the Jetson Orin Nano board, chassis dissolving in | `JETSON ORIN NANO`, `OAK-D-S2`, `SKY BETA` captions |
| 0:10 | pull back to the liquid-chrome airframe, type slides in from behind it, one 360° turn | hero spec |
| 0:34 | lift off, LiDAR cloud resolves into the as-built maze | `SCAN → MAP` |
| 0:46 | top-down sweep of the whole course | **01 A\* + EGO B-spline planning** |
| 0:58 | chase cam on the real route | **02 OAK-D-S2 stereo odometry** |
| 1:16 | tunnel cutaway, IR beams to both walls | **03 IR wall centring** |
| 1:40 | isometric top-down, scroll locks | **OVERRIDE MODE** |
| 1:44 · 1:52 | scripted blockers drop on the active path, live re-plans | planner log spam |
| 1:58 | control released to you, grid armed | `[ INJECT DYNAMIC OBSTACLE ]` is yours |

The timeline is data: `src/lib/director.ts` (`P_KEYS`, `SHOTS`, `CAPTIONS`).

## How it is put together

```
src/
  app/                    layout, page, globals.css (design tokens), /api/snap (dev only)
  components/
    Deck.tsx              the page: Lenis scroll + lock, keyboard, mounts everything
    SceneManager.tsx      <Canvas>, Suspense, Preload, PerformanceMonitor, effects
    AutoDirector.tsx      the 2-minute demo: clock -> progress + camera rails + scripted override beats
    DroneModel.tsx        <primitive object={scene}/> for the CAD, liquid-chrome materials, props, anchors
    SceneEnvironment.tsx  lights, floor, route line, scan cloud, wireframe maze
    MazeEnvironment.tsx   as-built course: hologram walls, tunnel, boulders, pad, checkpoints
    InteractiveJudgeOverride.tsx   override scene: grid, blockers, plan lines, hover preview, flight
    InteractivePlanner.tsx         alias of the above (the module name in the original brief)
    Blocker.tsx           the obstacle a judge drops (steady, no glitch; its tag only shows in override)
    hud/                  HudFrame, DemoControls, OverridePanel, TelemetryHUD (override only), Callouts, HeroOverlay, Loader
    three/                GlowLine (MeshLine), RouteLine, HeroType (troika), Effects, StudioLights, CameraRig, ...
  lib/
    choreography.ts       the master timeline: scroll progress -> one mutable `view` object, no React renders per frame
    director.ts           demo clock, camera rails, captions
    planner.ts            occupancy grid -> ESDF -> clearance-aware A* -> LOS prune -> EGO-style B-spline
    telemetry.ts          the streaming terminal lines
    store.ts              zustand store + per-frame `flight` object
  data/                   course.json, route-*.json, provenance.json (extracted from git, see below)
  config/assets.ts        the asset slots
```

Everything per-frame runs in `useFrame` against mutable objects (`view`, `flight`) and Framer Motion `motionValue`s; React only re-renders on real state changes. HTML is `position: fixed` over a single full-screen canvas.

**Layout contract.** In the hero and the tour nothing is docked to the right, so the left and the centre are free. In override the telemetry panel is a side panel (27 rem, right) from 1024 px up and a 26vh strip along the bottom below that; the override banner, run bar and controls centre on the free area left of it (`calc((100% - 30.5rem) / 2)`), and the planner stats grid shows from 1280 px. If you change the panel's width in `TelemetryHUD.tsx`, change that `30.5rem` reserve in `OverridePanel.tsx` and `DemoControls.tsx` to match.

## Asset slots: use your own CAD or map

Everything is swapped in `src/config/assets.ts`; files live in `public/models`. Bump `V` there when you regenerate a model (they are served `immutable` in production builds).

- **`drone`** `.glb` / `.gltf`, rendered with `<primitive object={scene} />`.
  - `extras.role` on a node picks its material: `plate` `arm` `chrome` `motor-bell` `jetson` `port` `pcb` `pixhawk` `oak-body` `lens` `glass` `led` `dark` `battery` `battery-label` `heatsink`. No role falls back to chrome.
  - Empties named `prop_FL` `prop_FR` `prop_RL` `prop_RR` (with `extras.dir` = ±1 and `extras.radius` in metres) get spinning props. The Sky Beta CAD has none, so a tri-blade 6.1 in rotor is built procedurally (`three/rotor.ts`).
  - Empties named `anchor_camera` `anchor_jetson` `anchor_pixhawk` `anchor_battery` `anchor_esc` `anchor_motor_*` place the HUD callouts.
  - Frame: Y up, nose along −Z, metres, origin at the airframe centre.
- **`maze`** `.glb` / `.gltf` / `.obj`. Nodes are styled by `extras.kind` (GLB) or by a regex on the node name (OBJ): `wall` `tube` `boulder` `pad` `net` `floor`.
- **`scan`** optional LiDAR-style point cloud (`course-scan.bin`). Set it to `null` to skip the scan → map reveal.

### Regenerating the assets

You only need this to change the map, the route or the CAD: the generated files are committed (`src/data/*.json`, `public/models/*`), so a fresh clone runs with just `npm install && npm run dev`.

```bash
# course data (walls, tunnel, pad, 286-waypoint route) straight from git; read-only `git show`, nothing is checked out
npm run assets:data                       # reads drone19/flight; COURSE_REF=origin/drone19/flight if you only have the remote branch

# as-built course OBJ -> public/models/dronehacks-course.glb + course-scan.bin
npm run assets:course                     # or: node scripts/obj-to-glb.mjs --obj path/to/your.obj

# Sky Beta STEP -> public/models/navless-uav.glb (needs native OpenCascade: pip package `cadquery-ocp`)
uv venv --python 3.12 .venv-cad && uv pip install --python .venv-cad cadquery-ocp numpy
node scripts/step-to-glb.mjs --step ~/Downloads/sky-beta.step --dump scratch/skybeta --python .venv-cad/Scripts/python.exe   # Windows
node scripts/step-to-glb.mjs --step ~/Downloads/sky-beta.step --dump scratch/skybeta --python .venv-cad/bin/python           # macOS / Linux
```

The STEP is 78 MB. The WASM build of OpenCascade (`occt-import-js`) exhausts its 2 GB heap on it and silently returns faces with no triangles, which is why the extractor is native. The GLB comes out at about 75k triangles / 4 MB (per-part triangle budgets, flat normals on the decimated parts). Raw STEP/OBJ inputs and `scratch/` are git-ignored.

## What is measured and what is staged

Please read this before you put numbers from the deck in front of a judge.

- **Course geometry and the route are real.** They are extracted from git (`drone19/flight`, commit `6c123b9`; `src/data/provenance.json` lists the commit each file came from). The maze is the 2026-11-07 layout (MN / MD / MT replaced M1-M3). The route is the one skybeta-19 is flown on, not a drawing.
- **The planner is real, and it is a port.** `lib/planner.ts` runs in the browser and its log lines (`A* voxel expansion: …ms`, `EGO B-spline recalculating`, iteration costs, clearances) are measured on every re-plan. It is written for this deck, not the flight code: occupancy grid → ESDF → A\* guided by the real route → B-spline optimiser. Its keep-out is 0.25 m so the demo corridors stay passable; the flight stack's configured inflation is 0.28 m, which is the number the callout copy quotes. The blocker in the demo is a rectangular box, so a drop that narrows a corridor below the keep-out will correctly report **ROUTE SEALED**, and the planner falls back to the Gate B route when that is the only way through.
- **The telemetry stream is simulated** (and only appears in override). Rates and values are anchored to numbers from the repo's logs and READMEs (cuVSLAM 31.2 Hz, odom_gate 28.43 Hz / 59.89 ms median age, ToF 9.6 Hz, height offset 19 cm, …) but the stream itself is generated. The panel header says **SIM**. Planner lines mixed into the same stream are not simulated.
- **The point cloud is a resample, not the scan.** The raw Scaniverse scan (596k points) is not in git, only the walls fitted to it. `course-scan.bin` is a LiDAR-style resample of the as-built mesh with 6 mm noise, and the slide says so.
- **The drone is a modified CAD.** Decimated, re-materialled (the liquid chrome replaces the CAD colours), propellers added. The airframe dimensions in the caption are the CAD's.
- **One headline to decide on.** Callout 03 is titled *SUB-CENTIMETER IR WALL CENTERING* (the wording from the brief). The proof line on the same slide says what the numbers are: ±0.3 cm steady on the ToF reading, and a 10-12 cm off-centre start ending within 2 cm in closed loop (sim). So "sub-centimetre" describes the sensor reading; the closed-loop centring result on that slide is about 2 cm. If a judge is likely to press on it, change the headline in `components/hud/Callouts.tsx`.

## Development notes

- Dev-only QA hooks (not present in production builds): `window.__navless.go(0.5)` jumps the scroll, `seek(42)` starts the demo paused at 42 s, `play()` resumes it, `window.__snap('name')` writes the next post-processed canvas frame to `scratch/name.jpg`, and `window.__advance(n)` steps n frames of the 3D loop by hand (handy when the window is hidden and the browser has paused `requestAnimationFrame`).
- Quality and speed were checked in a throttled preview pane only. **Not profiled on the presentation machine**: do a run-through there first, and if it stutters, load with `?q=medium` or `?q=low`.
- A hidden or unfocused browser window pauses `requestAnimationFrame`: the deck and its demo clock freeze until the window is visible again.
- Reduced-motion visitors get the finished hero immediately and an instant scroll.
- `scripts/test-planner.mts` is the regression test for the planner (nominal plan, clearance against random blockers, sealed routes, replan time budget).
