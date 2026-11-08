#!/usr/bin/env node
/**
 * extract-course-data.mjs
 *
 * Pulls the AS-BUILT DroneHacks course + skybeta-19's NE-first-leg window route straight out of
 * git (read-only `git show`, no checkout, no working-tree changes) and writes compact runtime JSON
 * for the pitch site:
 *
 *   src/data/course.json       walls / tunnel / boulders / pad / checkpoints, METRES, ENU, origin = floor centre
 *   src/data/route-window.json the 286-waypoint centreline route (the "ghost flight path")
 *   src/data/route-gate-b.json  the Gate B backup route (the planner's fallback guide)
 *   src/data/provenance.json   which commit every number came from
 *
 * Usage:   node scripts/extract-course-data.mjs            (reads drone19/flight)
 *          COURSE_REF=main node scripts/extract-course-data.mjs
 *
 * Source files (all on drone19/flight, identical on origin/drone19/flight 865db6b):
 *   gazebo_sim/course/course_asbuilt.json                    course_v3 + phone-LiDAR scan + 2026-11-07 maze change
 *   gazebo_sim/course/route_centreline_window_asbuilt.json   the route skybeta-19 flies
 *   depthcloud/asbuilt/maze_2026-11-07.json                  MN / MD / MT notes
 */
import { execFileSync } from 'node:child_process';
import { mkdirSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const pitch = path.resolve(here, '..');
const repo = path.resolve(pitch, '..');
const REF = process.env.COURSE_REF || 'drone19/flight';

const git = (...args) => execFileSync('git', ['-C', repo, ...args], { maxBuffer: 256 * 1024 * 1024 });
const showJson = (p) => JSON.parse(git('show', `${REF}:${p}`).toString('utf8'));
const lastCommit = (p) =>
  git('log', '-1', '--format=%h|%ad|%s', '--date=short', REF, '--', p).toString('utf8').trim();

const SRC = {
  course: 'gazebo_sim/course/course_asbuilt.json',
  route: 'gazebo_sim/course/route_centreline_window_asbuilt.json',
  routeB: 'gazebo_sim/course/route_centreline_gate_b_asbuilt.json',
  maze: 'depthcloud/asbuilt/maze_2026-11-07.json',
  obj: 'dronehacks-course-asbuilt.obj',
};

const IN = 0.0254;
const r4 = (v) => Math.round(v * 10000) / 10000;
/** inches from the net's south-west corner -> metres from the floor centre (x east, y north) */
const pos = ([x, y]) => [r4((x - 120) * IN), r4((y - 120) * IN)];
const len = (v) => r4(v * IN);

const course = showJson(SRC.course);
const route = showJson(SRC.route);
const routeB = showJson(SRC.routeB);

const walls = course.walls.map((w) => {
  const out = {
    name: w.name,
    kind: w.kind,
    a: pos(w.start_in),
    b: pos(w.end_in),
    thickness: len(w.thickness_in),
    height: len(w.height_in),
  };
  if (w.window) {
    out.window = {
      center: pos(w.window.center_in),
      width: len(w.window.width_in),
      height: len(w.window.height_in),
      bottom: len(w.window.bottom_above_floor_in),
      top: len(w.window.top_above_floor_in),
    };
  }
  return out;
});

const t = course.tunnel;
const tunnel = {
  xMin: pos([t.footprint_in.x_min, 0])[0],
  xMax: pos([t.footprint_in.x_max, 0])[0],
  yMin: pos([0, t.footprint_in.y_min])[1],
  yMax: pos([0, t.footprint_in.y_max])[1],
  ovalWidth: len(t.interior_oval_in.width),
  ovalHeight: len(t.interior_oval_in.height),
  length: len(t.length_in),
  axis: 'north-south',
  zBottom: len(t.interior_z_in[0]),
  zTop: len(t.interior_z_in[1]),
  note: t.note,
};
tunnel.centerX = r4((tunnel.xMin + tunnel.xMax) / 2);

const boulders = course.boulders_in.map((b) => ({
  center: pos(b.center),
  diameter: len(b.diameter),
  height: len(b.height),
}));

const pad = {
  center: pos(course.landing_pad_in.center),
  rOuter: len(course.landing_pad_in.outer_ring_radius),
  rings: course.landing_pad_in.inner_ring_radii.map(len),
  rApproach: len(course.landing_pad_in.approach_ring_radius),
};

const gates = Object.fromEntries(
  Object.entries(course.gates_in).map(([k, g]) => [
    k,
    { a: pos(g.between[0]), b: pos(g.between[1]), note: g.note },
  ]),
);

// Checkpoint markers: the five scored checkpoints (4 + 7 + 10 + 7 + 7 = 35 pts).
const window_ = walls.find((w) => w.window)?.window;
const gateAmid = [
  r4((gates.gate_A.a[0] + gates.gate_A.b[0]) / 2),
  r4((gates.gate_A.a[1] + gates.gate_A.b[1]) / 2),
];
const pts = Object.fromEntries(course.checkpoints.map((c) => [c.n, c]));
const launch = pos(course.launch_mark_in);
const checkpoints = [
  { n: 1, name: 'FIRST CORRIDOR', points: pts[1].points, at: [r4(launch[0] + 0.55), r4(launch[1] + 0.9)] },
  { n: 2, name: 'MAZE', points: pts[2].points, at: gateAmid },
  { n: 3, name: 'TUNNEL', points: pts[3].points, at: [tunnel.centerX, tunnel.yMin] },
  { n: 4, name: 'CHICANE', points: pts[4].points, at: window_.center },
  { n: 5, name: 'LANDING', points: pts[5].points, at: pad.center },
];

const out = {
  frame: 'ENU metres, origin = floor centre (x east, y north, z up). three.js: (x, z, -y).',
  notes:
    'AS-BUILT course. Walls MN/MD/MT replaced M1/M2/M3 on 2026-11-07. A1, C1 sit where the phone-LiDAR scan put them. ' +
    'The old floor tape is NOT included (it is the old maze and must not be followed).',
  net: { halfX: 3.048, halfY: 3.048, height: len(course.net_in.height) },
  walls,
  removedWalls: course.removed_walls.map((w) => w.name),
  tunnel,
  boulders,
  pad,
  launch,
  gates,
  checkpoints,
  drone: course.drone_m,
};

const routeBOut = {
  meta: {
    type: routeB.route_type,
    lengthM: routeB.length_m,
    minClearanceM: routeB.min_clearance_m_nose_north,
    notes: routeB.notes,
  },
  waypoints: routeB.waypoints_enu.map(([x, y, z]) => [r4(x), r4(y), r4(z)]),
};

const routeOut = {
  meta: {
    type: route.route_type,
    frame: route.frame_convention,
    launch: route.launch_origin_enu.slice(0, 2),
    spacingM: route.spacing_m,
    lengthM: route.length_m,
    tipRadiusM: route.tip_radius_m,
    minClearanceM: route.min_clearance_m_nose_north,
    minClearanceAt: route.min_clearance_at_nose_north,
    tightSpotsM: route.tight_spots_m_nose_north,
    notes: route.notes,
    maze: route.maze,
  },
  waypoints: route.waypoints_enu.map(([x, y, z]) => [r4(x), r4(y), r4(z)]),
};

const prov = {
  generatedBy: 'pitch/scripts/extract-course-data.mjs',
  repo: 'RaagaSundar/Navless',
  ref: REF,
  refCommit: git('rev-parse', REF).toString('utf8').trim(),
  files: Object.fromEntries(
    Object.entries(SRC).map(([k, p]) => {
      const [hash, date, subject] = lastCommit(p).split('|');
      return [k, { path: p, lastCommit: hash, date, subject }];
    }),
  ),
};

const dataDir = path.join(pitch, 'src', 'data');
mkdirSync(dataDir, { recursive: true });
writeFileSync(path.join(dataDir, 'course.json'), JSON.stringify(out, null, 1));
writeFileSync(path.join(dataDir, 'route-window.json'), JSON.stringify(routeOut));
writeFileSync(path.join(dataDir, 'route-gate-b.json'), JSON.stringify(routeBOut));
writeFileSync(path.join(dataDir, 'provenance.json'), JSON.stringify(prov, null, 2));

console.log(`course.json        ${walls.length} walls, ${boulders.length} boulders, tunnel, pad, ${checkpoints.length} checkpoints`);
console.log(`route-window.json  ${routeOut.waypoints.length} waypoints, ${routeOut.meta.lengthM} m`);
console.log(`route-gate-b.json  ${routeBOut.waypoints.length} waypoints, ${routeBOut.meta.lengthM} m`);
console.log(`provenance.json    ${REF} @ ${prov.refCommit.slice(0, 7)}`);
