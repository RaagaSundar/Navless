/**
 * test-planner.mts: sanity + stress tests for src/lib/planner.ts on the REAL as-built course.
 *   npm run test:planner
 */
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import type { CourseData, RouteData, Vec2 } from '../src/lib/course-types.ts';
import { DEFAULT_CONFIG, planMission, planClearanceAgainst, occupiedAt, type DynObstacle } from '../src/lib/planner.ts';
import { buildVia } from '../src/lib/via.ts';

const here = path.dirname(fileURLToPath(import.meta.url));
const load = <T,>(f: string) => JSON.parse(readFileSync(path.join(here, '..', 'src', 'data', f), 'utf8')) as T;
const course = load<CourseData>('course.json');
const route = load<RouteData>('route-window.json');
const routeB = load<{ waypoints: number[][] }>('route-gate-b.json');
const { via, anchors } = buildVia(course, route);
const guides = { primary: route.waypoints.map((w) => [w[0], w[1]] as Vec2), fallback: routeB.waypoints.map((w) => [w[0], w[1]] as Vec2) };
const START: Vec2 = course.launch;

let failures = 0;
const check = (name: string, ok: boolean, detail = '') => {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
  if (!ok) failures++;
};
const f = (v: number, d = 2) => v.toFixed(d);

console.log('via points:', via.map((v) => `${v.label}(${f(v.at[0])},${f(v.at[1])})`).join('  '));
console.log('route anchor indices:', JSON.stringify(anchors));

/* 1. nominal plan, no obstacles */
const nominal = planMission(course, via, [], START, DEFAULT_CONFIG, { kind: 'initial' }, guides);
check('nominal plan succeeds', nominal.ok, nominal.reason ?? '');
console.log(
  `   length ${f(nominal.length)} m (real route ${route.meta.lengthM} m)  T ${f(nominal.duration, 1)} s  minClr ${f(nominal.minClearance)} m  maxCurv ${f(nominal.maxCurvature)} 1/m  mode ${nominal.splineMode}  expanded ${nominal.expanded}  total ${f(nominal.timings.total, 1)} ms  fallback ${nominal.usedFallback}`,
);
check('nominal route length within 25% of the real route', Math.abs(nominal.length - route.meta.lengthM) / route.meta.lengthM < 0.25);
check('nominal min clearance >= keep-out - 3 cm', nominal.minClearance >= DEFAULT_CONFIG.inflation - 0.03, `(${f(nominal.minClearance)})`);
{
  // where is the sharpest turn?
  const sm = nominal.samples; let worst = 0, wi = 0;
  for (let k = 1; k < sm.length / 2 - 1; k++) {
    const a1 = Math.atan2(sm[2*k+1]-sm[2*k-1], sm[2*k]-sm[2*k-2]); const a2 = Math.atan2(sm[2*k+3]-sm[2*k+1], sm[2*k+2]-sm[2*k]);
    let da = Math.abs(a2 - a1); if (da > Math.PI) da = 2 * Math.PI - da; if (da > worst) { worst = da; wi = k; }
  }
  console.log(`   sharpest turn: ${(worst*180/Math.PI).toFixed(0)} deg between samples ${wi-1}..${wi+1} at (${sm[2*wi].toFixed(2)}, ${sm[2*wi+1].toFixed(2)})  [of ${sm.length/2} samples]`);
}
check('nominal has no hairpin (max curvature < 8 1/m)', nominal.maxCurvature < 8, `(${f(nominal.maxCurvature)})`);
check('nominal does not need Gate B', !nominal.usedFallback);

// distance of the real route to our plan (how close is A*+EGO to Raaga's centreline route?)
{
  let sum = 0;
  let max = 0;
  for (const w of route.waypoints) {
    let best = Infinity;
    for (let k = 0; k < nominal.samples.length / 2; k++) best = Math.min(best, Math.hypot(w[0] - nominal.samples[2 * k], w[1] - nominal.samples[2 * k + 1]));
    sum += best;
    max = Math.max(max, best);
  }
  console.log(`   real route -> plan deviation: mean ${f(sum / route.waypoints.length)} m  max ${f(max)} m`);
  check('plan hugs the real route (max deviation < 1.0 m)', max < 1.0, `(${f(max)})`);
}

/* 2. scripted obstacles */
const mk = (id: number, x: number, y: number, h = 0.2): DynObstacle => ({ id, x, y, hw: h, hd: h });
const at = (frac: number): Vec2 => {
  const i = Math.round(frac * (nominal.samples.length / 2 - 1));
  return [nominal.samples[2 * i], nominal.samples[2 * i + 1]];
};

const scenarios: [string, Vec2][] = [
  ['first corridor (12% along)', at(0.12)],
  ['mid corridor (25% along)', at(0.25)],
  ['gate A neighbourhood', via[0].at],
  ['tunnel entrance', via[1].at],
  ['inside the tunnel', [course.tunnel.centerX, (course.tunnel.yMin + course.tunnel.yMax) / 2]],
  ['window', via[4].at],
  ['pad approach (88% along)', at(0.88)],
];
for (const [name, p] of scenarios) {
  const o = mk(1, p[0], p[1]);
  const occ = occupiedAt(course, [], p[0], p[1]);
  const prior = planClearanceAgainst(course, nominal, [o]);
  const r = planMission(course, via, [o], START, DEFAULT_CONFIG, { kind: 'inject', obstacle: o, priorMinClearance: prior }, guides);
  console.log(`   [${name}] cell=${occ} ok=${r.ok} len=${f(r.length)} minClr=${f(r.minClearance)} fallback=${r.usedFallback} ${f(r.timings.total, 1)} ms ${r.ok ? '' : '(' + r.reason + ')'}`);
  if (r.ok) check(`clear of the obstacle: ${name}`, r.minClearance >= DEFAULT_CONFIG.inflation - 0.03);
}

/* 3. stress: 300 random obstacles anywhere on the floor */
{
  let ok = 0;
  let blocked = 0;
  let viol = 0;
  let fallbacks = 0;
  let maxMs = 0;
  let sumMs = 0;
  let rejected = 0;
  let seed = 12345;
  const rnd = () => ((seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296);
  const N = 300;
  for (let k = 0; k < N; k++) {
    const x = (rnd() * 2 - 1) * 2.9;
    const y = (rnd() * 2 - 1) * 2.9;
    if (occupiedAt(course, [], x, y) !== 'free') {
      rejected++;
      continue;
    }
    const o = mk(k, x, y);
    const r = planMission(course, via, [o], START, DEFAULT_CONFIG, { kind: 'inject', obstacle: o }, guides);
    sumMs += r.timings.total;
    maxMs = Math.max(maxMs, r.timings.total);
    if (r.ok) {
      ok++;
      if (r.usedFallback) fallbacks++;
      if (r.minClearance < DEFAULT_CONFIG.inflation - 0.03) viol++;
    } else blocked++;
  }
  console.log(`   random obstacles: ${N} drops, ${rejected} on walls (rejected), ${ok} planned, ${fallbacks} via Gate B, ${blocked} sealed routes, ${viol} clearance violations, mean ${f(sumMs / Math.max(1, ok + blocked), 1)} ms, max ${f(maxMs, 1)} ms`);
  check('no planned route violates the keep-out', viol === 0, `(${viol})`);
  check('planning stays fast (max < 60 ms)', maxMs < 60, `(${f(maxMs, 1)} ms)`);
}

/* 4. print one full log so the terminal copy can be reviewed */
{
  const p = at(0.3);
  const o = mk(7, p[0], p[1]);
  const prior = planClearanceAgainst(course, nominal, [o]);
  const r = planMission(course, via, [o], START, DEFAULT_CONFIG, { kind: 'inject', obstacle: o, priorMinClearance: prior }, guides);
  console.log('\n--- sample terminal log ---');
  for (const l of r.log) console.log(`[+${l.t.toFixed(1).padStart(6)}ms] ${l.tag.padEnd(5)} ${l.text}`);
}

/* 5. optional debug dump for plotting:  node scripts/test-planner.mts --dump=file.json */
const dumpArg = process.argv.find((a) => a.startsWith('--dump='));
if (dumpArg) {
  const heat: { x: number; y: number; code: string }[] = [];
  // sealed / ok / Gate-B map on a 0.2 m lattice for a 0.48 m blocker
  for (let y = -2.9; y <= 2.9; y += 0.2) {
    for (let x = -2.9; x <= 2.9; x += 0.2) {
      if (occupiedAt(course, [], x, y) !== 'free') {
        heat.push({ x, y, code: 'W' });
        continue;
      }
      const o = mk(1, x, y);
      const r = planMission(course, via, [o], START, DEFAULT_CONFIG, { kind: 'inject', obstacle: o }, guides);
      heat.push({ x, y, code: r.ok ? (r.usedFallback ? 'B' : 'O') : 'S' });
    }
  }
  const out = {
    course,
    route: route.waypoints,
    via: via.map((v) => ({ id: v.id, at: v.at })),
    nominal: { samples: Array.from(nominal.samples), polyline: nominal.polyline, legPolys: nominal.legPolys, ctrl: nominal.ctrl, mode: nominal.splineMode },
    heat,
  };
  const { writeFileSync } = await import('node:fs');
  writeFileSync(dumpArg.slice(7), JSON.stringify(out));
  console.log('dumped', dumpArg.slice(7));
}

console.log(failures ? `\n${failures} FAILED` : '\nall planner checks passed');
process.exit(failures ? 1 : 0);
