# NAVLESS: ZERO GPS. PURE AUTONOMY.

A scroll-driven 3D pitch deck for **skybeta-19**, the GPS-denied indoor UAV we are flying at DroneHacks.
Next.js 16 (App Router) · React Three Fiber 9 · drei · `@react-three/postprocessing` · Framer Motion · Tailwind 4.

Three scenes on one timeline, plus a 2-minute auto-demo and a judge-interactive finale:

| # | Scene | What happens |
|---|-------|--------------|
| 01 | **Hero reveal** | The Sky Beta CAD builds itself out of the dusk in dark liquid chrome (`MeshPhysicalMaterial`: metalness 1, roughness 0.1, clearcoat 1) lit by a low amber sun, brutalist type slides in *behind* it with real depth, one 360° turn on scroll, a sparse cuVSLAM feature cloud. |
| 02 | **The environment** | The drone shrinks to the corner, a phone-LiDAR-style point cloud resolves into the as-built DroneHacks maze (glowing `EdgesGeometry` wireframe). skybeta-19's real 14.2 m route is drawn as a glowing gradient line. Three callouts: **A\* + EGO B-spline planning**, **OAK-D-S2 stereo odometry**, **blind carry through the tunnel** (tunnel cutaway: the camera cone goes dark and the head turns amber while it rides PX4's own estimate). |
| 03 | **Override mode** | Scroll locks, camera lifts to an isometric top-down. **`[ INJECT DYNAMIC OBSTACLE ]`** arms the grid; click to drop a blocker (it falls in and settles with one soft ring). A real A\* + EGO planner re-plans in the browser (a few ms) and the new B-spline is drawn immediately while the invalidated one fades out in vermilion. Planner output scrolls through the live terminal. |

The **live telemetry terminal** (on-board stack lines mixed with the real planner's output) and the live clock exist **only in override mode**, the interactive part. The hero and the tour stay clean: no FPS counter, no ticking clock, no streaming text.

## Pitch day: one laptop for the eye, the others for the detail

The judging pitch is 3:20 PM in Sequoia, 13 minutes plus questions (the judging page says 20 in total): the team talks, this laptop is for the eye, and the technical material comes up on other laptops.

- **This laptop (`/`)** is the cinematic deck above. It never sits still: 20 s untouched and the 2-minute sequence starts by itself; after it hands the grid over, 25 s untouched replays the four real flights below one after another, then the sequence starts again. A click or key stops the wait (`Esc` aborts the sequence). `/?attract=0` turns all of it off. The pointer nudges the camera a little (not in override), and Mars dust drifts through the course. Everything is slow: nothing flashes.
- **Replay the real flights** (override mode, four buttons under the controls: Run 1, 2, 4, 5). The demo drone flies the track skybeta-19's flight recorder says it *believed* (cuVSLAM odometry, 10 Hz, takeoff to the first impact), with a chase camera, an ice ribbon of the believed track, the gold route we planned, and the dashed setpoints it was sent. At the impact the camera eases back, a footprint ring marks the touch and the wall it hit lights up (Runs 1, 2, 4). This is the logged data, not a simulation; the panel says so. `src/data/evidence.json` is written by `npm run assets:evidence` straight from the CSVs on `origin/flight-logs/2026-11-08` (read-only `git show`). The impact is the first tick where the camera pose cannot be physical (a step over 0.25 m in one 0.1 s tick, or a heading change over 10° in one tick), walked back to where PX4's and the camera's headings first parted by more than 2°.
- **The other laptops: `/tech`.** The technical companion: the system diagram (what it senses, how it knows where it is, how it decides, how that becomes motor commands, where each piece runs and its loop rate), the evidence plot (click a run), what generalizes vs what is tuned, what breaks it, every number with its source and the likely questions, and the speaker script. Jump with the bar or `/tech#system`, `#evidence`, `#breaks`, `#numbers`, `#script`.
- **The script: `SCRIPT.md`** (`npm run script`), generated from `src/data/script.ts`, `facts.ts` and `today.ts`, so it can never disagree with `/tech`. About 10 minutes of speech in the judges' seven-part order (problem, one diagram, why this approach, generalizes vs tuned, what breaks it, evidence, hardest problem / next day / team).
- **Edit before 3:20 PM: `src/data/today.ts`.** The scored-run results (`scored`) and three flags (live re-planning, the localizer, the compass setting) change what the script and `/tech` say. The defaults are what was true at the 2026-11-08 midday handoff. Then `npm run script`, and rebuild (`npm run build`) if you serve the production build.
- **Slide deck (opt-in): `/?mode=talk`.** The same content as a presenter deck (`→` next, `N` notes and timer, `Q` numbers, `L` live demo). Not the default: the main screen is for the eye.

## Deploy (Vercel)

Live: **https://navless-pitch.vercel.app** (Vercel project `navless-pitch`, the personal scope of sundar.raaga@gmail.com). `/` is the cinematic deck, `/tech` the technical companion. The production domain is public; the unique per-deployment URLs sit behind Vercel login (Vercel Authentication).

It is deployed from a plain folder outside any git repo that is linked to the project: `C:/Users/sunda/Navless-vercel`. Do not link from inside this repo: the CLI can then connect the GitHub repository, and Vercel would build on every push to every branch. `vercel.json` pins the Next.js preset, because a project made with `vercel project add` starts on "Other" and then serves only `public/` (every page 404s).

To redeploy after editing `src/data/today.ts` or anything else (about 40 s), from this folder:

```bash
tar cf - --exclude=./node_modules --exclude=./.next --exclude=./scratch --exclude=./tsconfig.tsbuildinfo --exclude=./next-env.d.ts --exclude=./.vercel --exclude=./.env.local . | tar xf - -C /c/Users/sunda/Navless-vercel && cd /c/Users/sunda/Navless-vercel && vercel deploy --prod --yes --scope raag-sundars-projects
```

Never leave `.env.local` in the folder you deploy: `vercel link` writes a short-lived token there.

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
| Run 1 / 2 / 4 / 5 buttons | replay that real flight from the recorder's log (override) |
| scroll up hard | leave override |

### Deep links

| URL | Effect |
|-----|--------|
| `/?p=0.5` | open at scroll position 0.5 (0 to 1) |
| `/?demo=1` | start the auto-demo as soon as loading finishes |
| `/?attract=0` | no idle attract loop (the demo and the replays only start when you start them) |
| `/tech` | the technical companion for the other laptops |
| `/?mode=talk` | the slide deck (`#evidence.2` in the URL opens a slide and a build step) |
| `/?q=low` · `medium` · `high` | lock the render quality tier (default is adaptive: a `PerformanceMonitor` walks DPR, MSAA and the chromatic-aberration pass down if the GPU can't hold frame rate) |

## The 2-minute sequence

| Time | Shot | On screen |
|------|------|-----------|
| 0:00 | macro of the Jetson Orin Nano board, chassis dissolving in | `JETSON ORIN NANO`, `OAK-D-S2`, `SKY BETA` captions |
| 0:10 | pull back to the liquid-chrome airframe, type slides in from behind it, one 360° turn | hero spec |
| 0:34 | lift off, LiDAR cloud resolves into the as-built maze | `SCAN → MAP` |
| 0:46 | top-down sweep of the whole course | **01 A\* + EGO B-spline planning** |
| 0:58 | chase cam on the real route | **02 OAK-D-S2 stereo odometry** |
| 1:16 | tunnel cutaway, the camera cone goes dark and the head turns amber | **03 Blind carry through the dark** |
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
- **The flight replay is real data.** Positions, heights and headings are the flight recorder's (one point per 0.1 s tick) for the first four course runs, plus the route we planned for comparison. What is derived: the impact moment, found by the rule in `scripts/extract-evidence.mjs`. The README of the flight-logs branch (Shining's notes) is the source of what each run touched; Run 5's notes say "the front wall", so the deck does not light a specific wall for it.
- **The telemetry stream is simulated** (and only appears in override). Rates and values are anchored to numbers from the repo's logs and READMEs (cuVSLAM 31.2 Hz, odom_gate 28.43 Hz / 59.89 ms median age, height offset 19 cm, …) but the stream itself is generated. The panel header says **SIM**. Planner lines mixed into the same stream are not simulated.
- **The point cloud is a resample, not the scan.** The raw Scaniverse scan (596k points) is not in git, only the walls fitted to it. `course-scan.bin` is a LiDAR-style resample of the as-built mesh with 6 mm noise, and the slide says so.
- **The drone is a modified CAD.** Decimated, re-materialled (the liquid chrome replaces the CAD colours), propellers added. The airframe dimensions in the caption are the CAD's.
- **The tunnel beat is blind carry, not ToF.** The side time-of-flight sensors are no longer used (Raaga, 2026-11-08), so callout 03 says what the flight code does in the tunnel: it rides PX4's own estimate for at most 4 s. The ToF sensors appear only as built and bench-tested. The repo's launch files still had the ToF chain on by default at the last fetch; the deck claims nothing about that.

## Development notes

- Dev-only QA hooks (not present in production builds): `window.__navless.enter()` opens override, `window.__navless.replay.start(i)` replays a run, `window.__project(x,y,z)` gives a world point's screen position, `window.__navless.go(0.5)` jumps the scroll, `seek(42)` starts the demo paused at 42 s, `play()` resumes it, `window.__snap('name')` writes the next post-processed canvas frame to `scratch/name.jpg`, and `window.__advance(n)` steps n frames of the 3D loop by hand (handy when the window is hidden and the browser has paused `requestAnimationFrame`).
- Quality and speed were checked in a throttled preview pane only. **Not profiled on the presentation machine**: do a run-through there first, and if it stutters, load with `?q=medium` or `?q=low`.
- A hidden or unfocused browser window pauses `requestAnimationFrame`: the deck and its demo clock freeze until the window is visible again.
- Reduced-motion visitors get the finished hero immediately and an instant scroll.
- `scripts/test-planner.mts` is the regression test for the planner (nominal plan, clearance against random blockers, sealed routes, replan time budget).
