/**
 * planner.ts: the mini planner behind the OVERRIDE demo.
 *
 * It is the same idea Navless flies, shrunk to run in a browser tab in a few milliseconds:
 *
 *   occupancy grid (0.1 m)  ->  ESDF  ->  clearance-aware A*  ->  line-of-sight prune
 *     ->  clamped cubic B-spline whose control points are pushed off obstacles (EGO-Planner style)
 *
 * The map is the AS-BUILT DroneHacks course (walls, tunnel, window) read from git, plus whatever
 * obstacles the judges drop. Everything the terminal prints (expanded nodes, timings, clearance,
 * curvature) is measured from the run that just happened. No numbers are invented.
 *
 * Pure TypeScript with no DOM / three imports, so `npm run test:planner` can run it in Node.
 */
import type { CourseData, Vec2 } from './course-types.ts';

/* ───────────────────────────── config & types ───────────────────────────── */

export interface PlannerConfig {
  /** grid cell size, m */
  res: number;
  /** hard keep-out radius around anything occupied, m. skybeta-19 is 0.475 m wide with props (half = 0.2375 m);
   *  the real planner pads walls by 0.22 to 0.28 m (course_config.json / README), this demo uses 0.25. */
  inflation: number;
  /** A* prefers to stay at least this far from walls, m */
  prefClearance: number;
  /** strength of that preference */
  clearanceWeight: number;
  /** EGO soft safety distance for the B-spline, m */
  safe: number;
  /** B-spline control-point spacing, m */
  ctrlSpacing: number;
  /** cruise speed used for the T estimate, m/s (the real course flights run 0.3-0.5) */
  speed: number;
  /** minimum clearance a pruned shortcut may have, m */
  losClearance: number;
  /** a pruned shortcut may not stray further than this from the A* path it replaces, m */
  pruneDeviation: number;
  /** A* is pulled toward the real, Gazebo-checked flight route: extra cost at `guideRange` m away (multiplier) */
  guideWeight: number;
  guideRange: number;
}

export const DEFAULT_CONFIG: PlannerConfig = {
  res: 0.1,
  inflation: 0.25,
  prefClearance: 0.55,
  clearanceWeight: 1.2,
  safe: 0.4,
  ctrlSpacing: 0.3,
  speed: 0.5,
  losClearance: 0.36,
  pruneDeviation: 0.22,
  guideWeight: 1.8,
  guideRange: 0.7,
};

export interface DynObstacle {
  id: number;
  x: number;
  y: number;
  /** half extents of the square footprint, m */
  hw: number;
  hd: number;
}

export interface ViaPoint {
  id: string;
  label: string;
  at: Vec2;
  kind: 'gate' | 'tube' | 'window' | 'pad';
  /** optional groups can be skipped (Gate B fallback) when the planner cannot get through them */
  group?: 'window';
}

export type LogTag =
  | 'SYS' | 'EVENT' | 'MAP' | 'PLAN' | 'A*' | 'PRUNE' | 'EGO' | 'TRACK' | 'FSM' | 'WARN' | 'OK'
  | 'VIO' | 'GATE' | 'CAM' | 'EKF2' | 'HEAD' | 'PX4' | 'NAV' | 'MAG' | 'WALL' | 'TUBE' | 'BLIND';
export interface LogLine {
  /** ms since the replan started */
  t: number;
  tag: LogTag;
  text: string;
  level: 'info' | 'ok' | 'warn' | 'err' | 'dim';
}

export interface GridMap {
  n: number;
  res: number;
  origin: number;
  occ: Uint8Array;
  staticOcc: Uint8Array;
  esdf: Float32Array;
  esdfStatic: Float32Array;
}

export interface LegStat {
  from: string;
  to: string;
  ok: boolean;
  expanded: number;
  ms: number;
  length: number;
}

export interface PlanResult {
  ok: boolean;
  reason?: string;
  start: Vec2;
  goal: Vec2;
  /** via points that ended up on the route (markers) */
  via: ViaPoint[];
  usedFallback: boolean;
  legs: LegStat[];
  /** pruned A* waypoints */
  polyline: Vec2[];
  /** pruned A* waypoints per leg (for per-checkpoint colouring / debugging) */
  legPolys: Vec2[][];
  ctrl: Vec2[];
  /** x,y pairs, uniform arc-length spacing */
  samples: Float32Array;
  cum: Float32Array;
  length: number;
  minClearance: number;
  maxCurvature: number;
  duration: number;
  expanded: number;
  iterations: number;
  splineMode: 'ego' | 'rounded' | 'astar';
  timings: { grid: number; astar: number; prune: number; ego: number; total: number };
  blockedAt?: Vec2;
  log: LogLine[];
}

/* ───────────────────────────── grid ───────────────────────────── */

const SQRT2 = Math.SQRT2;
const GRID_N = 61;

const dist2seg = (px: number, py: number, a: Vec2, b: Vec2) => {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const l2 = dx * dx + dy * dy;
  const t = l2 === 0 ? 0 : Math.max(0, Math.min(1, ((px - a[0]) * dx + (py - a[1]) * dy) / l2));
  const cx = a[0] + t * dx;
  const cy = a[1] + t * dy;
  return Math.hypot(px - cx, py - cy);
};

function rasterSegment(occ: Uint8Array, n: number, res: number, origin: number, a: Vec2, b: Vec2, halfT: number) {
  const pad = halfT + res;
  const i0 = Math.max(0, Math.floor((Math.min(a[0], b[0]) - pad - origin) / res));
  const i1 = Math.min(n - 1, Math.ceil((Math.max(a[0], b[0]) + pad - origin) / res));
  const j0 = Math.max(0, Math.floor((Math.min(a[1], b[1]) - pad - origin) / res));
  const j1 = Math.min(n - 1, Math.ceil((Math.max(a[1], b[1]) + pad - origin) / res));
  for (let j = j0; j <= j1; j++) {
    for (let i = i0; i <= i1; i++) {
      const x = origin + (i + 0.5) * res;
      const y = origin + (j + 0.5) * res;
      // +0.045 so a 10 cm wall can never slip between cell centres
      if (dist2seg(x, y, a, b) <= halfT + 0.045) occ[j * n + i] = 1;
    }
  }
}

const staticCache = new WeakMap<CourseData, { occ: Uint8Array; esdf: Float32Array; key: string }>();

/** Occupancy of everything that is fixed: net, walls (window left open), tunnel sides. Flight-height plan view. */
function buildStatic(course: CourseData, cfg: PlannerConfig) {
  const key = `${cfg.res}`;
  const hit = staticCache.get(course);
  if (hit && hit.key === key) return hit;
  const n = GRID_N;
  const origin = (-n * cfg.res) / 2;
  const occ = new Uint8Array(n * n);

  // the net (20x20 ft room)
  const hx = course.net.halfX;
  const hy = course.net.halfY;
  const corners: Vec2[] = [
    [-hx, -hy],
    [hx, -hy],
    [hx, hy],
    [-hx, hy],
  ];
  for (let k = 0; k < 4; k++) rasterSegment(occ, n, cfg.res, origin, corners[k], corners[(k + 1) % 4], 0.02);

  // walls. The window is open at flight height (0.305 to 1.219 m), so the wall is split around it.
  for (const w of course.walls) {
    const halfT = w.thickness / 2;
    if (w.window) {
      const dx = w.b[0] - w.a[0];
      const dy = w.b[1] - w.a[1];
      const len = Math.hypot(dx, dy);
      const ux = dx / len;
      const uy = dy / len;
      const c = w.window.center;
      const hw = w.window.width / 2;
      rasterSegment(occ, n, cfg.res, origin, w.a, [c[0] - ux * hw, c[1] - uy * hw], halfT);
      rasterSegment(occ, n, cfg.res, origin, [c[0] + ux * hw, c[1] + uy * hw], w.b, halfT);
    } else {
      rasterSegment(occ, n, cfg.res, origin, w.a, w.b, halfT);
    }
  }

  // tunnel: two side walls, open at both mouths
  const t = course.tunnel;
  rasterSegment(occ, n, cfg.res, origin, [t.xMin + 0.02, t.yMin], [t.xMin + 0.02, t.yMax], 0.02);
  rasterSegment(occ, n, cfg.res, origin, [t.xMax - 0.02, t.yMin], [t.xMax - 0.02, t.yMax], 0.02);

  const esdf = computeEsdf(occ, n, cfg.res);
  const out = { occ, esdf, key };
  staticCache.set(course, out);
  return out;
}

/** Exact Euclidean distance transform (Felzenszwalb & Huttenlocher), metres to the nearest occupied cell surface. */
function computeEsdf(occ: Uint8Array, n: number, res: number): Float32Array {
  const INF = 1e12;
  const f = new Float64Array(n * n);
  for (let i = 0; i < n * n; i++) f[i] = occ[i] ? 0 : INF;

  const d = new Float64Array(n);
  const v = new Int32Array(n);
  const z = new Float64Array(n + 1);
  const line = new Float64Array(n);

  const pass = (get: (k: number) => number, set: (k: number, val: number) => void) => {
    for (let q = 0; q < n; q++) line[q] = get(q);
    let k = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = INF;
    for (let q = 1; q < n; q++) {
      let s = (line[q] + q * q - (line[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
      while (s <= z[k]) {
        k--;
        s = (line[q] + q * q - (line[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
      }
      k++;
      v[k] = q;
      z[k] = s;
      z[k + 1] = INF;
    }
    k = 0;
    for (let q = 0; q < n; q++) {
      while (z[k + 1] < q) k++;
      d[q] = (q - v[k]) * (q - v[k]) + line[v[k]];
    }
    for (let q = 0; q < n; q++) set(q, d[q]);
  };

  for (let x = 0; x < n; x++) pass((y) => f[y * n + x], (y, val) => (f[y * n + x] = val));
  for (let y = 0; y < n; y++) pass((x) => f[y * n + x], (x, val) => (f[y * n + x] = val));

  const out = new Float32Array(n * n);
  for (let i = 0; i < n * n; i++) {
    const cells = Math.sqrt(f[i]);
    out[i] = Math.min(99, Math.max(0, cells * res - res * 0.5));
  }
  return out;
}

export function buildGrid(course: CourseData, obstacles: DynObstacle[], cfg: PlannerConfig = DEFAULT_CONFIG): GridMap {
  const n = GRID_N;
  const origin = (-n * cfg.res) / 2;
  const st = buildStatic(course, cfg);
  let occ = st.occ;
  let esdf = st.esdf;
  if (obstacles.length) {
    occ = st.occ.slice();
    for (const o of obstacles) {
      const i0 = Math.max(0, Math.floor((o.x - o.hw - origin) / cfg.res));
      const i1 = Math.min(n - 1, Math.ceil((o.x + o.hw - origin) / cfg.res) - 1);
      const j0 = Math.max(0, Math.floor((o.y - o.hd - origin) / cfg.res));
      const j1 = Math.min(n - 1, Math.ceil((o.y + o.hd - origin) / cfg.res) - 1);
      for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) occ[j * n + i] = 1;
    }
    esdf = computeEsdf(occ, n, cfg.res);
  }
  return { n, res: cfg.res, origin, occ, staticOcc: st.occ, esdf, esdfStatic: st.esdf };
}

export const cellOf = (g: GridMap, x: number, y: number): [number, number] => [
  Math.max(0, Math.min(g.n - 1, Math.floor((x - g.origin) / g.res))),
  Math.max(0, Math.min(g.n - 1, Math.floor((y - g.origin) / g.res))),
];
export const centerOf = (g: GridMap, i: number, j: number): Vec2 => [g.origin + (i + 0.5) * g.res, g.origin + (j + 0.5) * g.res];

/** Bilinear ESDF lookup (metres). Outside the grid counts as the net: 0. */
export function sampleEsdf(g: GridMap, x: number, y: number): number {
  const fx = (x - g.origin) / g.res - 0.5;
  const fy = (y - g.origin) / g.res - 0.5;
  const i0 = Math.floor(fx);
  const j0 = Math.floor(fy);
  const tx = fx - i0;
  const ty = fy - j0;
  const at = (i: number, j: number) => (i < 0 || j < 0 || i >= g.n || j >= g.n ? 0 : g.esdf[j * g.n + i]);
  const a = at(i0, j0) * (1 - tx) + at(i0 + 1, j0) * tx;
  const b = at(i0, j0 + 1) * (1 - tx) + at(i0 + 1, j0 + 1) * tx;
  return a * (1 - ty) + b * ty;
}

function gradUnit(g: GridMap, x: number, y: number): Vec2 {
  const h = g.res * 0.5;
  const gx = sampleEsdf(g, x + h, y) - sampleEsdf(g, x - h, y);
  const gy = sampleEsdf(g, x, y + h) - sampleEsdf(g, x, y - h);
  const l = Math.hypot(gx, gy);
  return l < 1e-6 ? [0, 0] : [gx / l, gy / l];
}

/* ───────────────────────────── route prior ───────────────────────────── */

const guideCache = new WeakMap<Vec2[], { res: number; field: Float32Array }>();
/** Distance (m) from each cell centre to the nearest sample of a guide polyline. */
function guideField(g: GridMap, guide: Vec2[]): Float32Array {
  const hit = guideCache.get(guide);
  if (hit && hit.res === g.res) return hit.field;
  const field = new Float32Array(g.n * g.n);
  for (let j = 0; j < g.n; j++) {
    for (let i = 0; i < g.n; i++) {
      const x = g.origin + (i + 0.5) * g.res;
      const y = g.origin + (j + 0.5) * g.res;
      let best = Infinity;
      for (let k = 0; k < guide.length; k++) {
        const dx = guide[k][0] - x;
        const dy = guide[k][1] - y;
        const d = dx * dx + dy * dy;
        if (d < best) best = d;
      }
      field[j * g.n + i] = Math.sqrt(best);
    }
  }
  guideCache.set(guide, { res: g.res, field });
  return field;
}

/* ───────────────────────────── A* ───────────────────────────── */

class MinHeap {
  private ids: number[] = [];
  private keys: number[] = [];
  get size() {
    return this.ids.length;
  }
  push(id: number, key: number) {
    const ids = this.ids;
    const keys = this.keys;
    let i = ids.length;
    ids.push(id);
    keys.push(key);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (keys[p] <= key) break;
      ids[i] = ids[p];
      keys[i] = keys[p];
      i = p;
    }
    ids[i] = id;
    keys[i] = key;
  }
  pop(): number {
    const ids = this.ids;
    const keys = this.keys;
    const top = ids[0];
    const lastId = ids.pop()!;
    const lastKey = keys.pop()!;
    const n = ids.length;
    if (n > 0) {
      let i = 0;
      for (;;) {
        let c = 2 * i + 1;
        if (c >= n) break;
        if (c + 1 < n && keys[c + 1] < keys[c]) c++;
        if (keys[c] >= lastKey) break;
        ids[i] = ids[c];
        keys[i] = keys[c];
        i = c;
      }
      ids[i] = lastId;
      keys[i] = lastKey;
    }
    return top;
  }
}

const isFree = (g: GridMap, id: number, cfg: PlannerConfig) => g.esdf[id] >= cfg.inflation;

interface AStarResult {
  path: Vec2[] | null;
  expanded: number;
  ms: number;
  cost: number;
}

const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

function astar(g: GridMap, start: Vec2, goal: Vec2, cfg: PlannerConfig, gd?: Float32Array): AStarResult {
  const t0 = now();
  const n = g.n;
  const [si, sj] = cellOf(g, start[0], start[1]);
  const [gi, gj] = cellOf(g, goal[0], goal[1]);
  const sId = sj * n + si;
  const gId = gj * n + gi;
  if (!isFree(g, sId, cfg) || !isFree(g, gId, cfg)) return { path: null, expanded: 0, ms: now() - t0, cost: Infinity };

  const gScore = new Float32Array(n * n).fill(Infinity);
  const parent = new Int32Array(n * n).fill(-1);
  const closed = new Uint8Array(n * n);
  const heap = new MinHeap();
  const h = (i: number, j: number) => {
    const dx = Math.abs(i - gi);
    const dy = Math.abs(j - gj);
    return g.res * (dx + dy + (SQRT2 - 2) * Math.min(dx, dy));
  };
  gScore[sId] = 0;
  heap.push(sId, h(si, sj));
  let expanded = 0;
  const pref = cfg.prefClearance;
  const wc = cfg.clearanceWeight;

  while (heap.size) {
    const cur = heap.pop();
    if (closed[cur]) continue;
    closed[cur] = 1;
    expanded++;
    if (cur === gId) break;
    const ci = cur % n;
    const cj = (cur - ci) / n;
    for (let dj = -1; dj <= 1; dj++) {
      for (let di = -1; di <= 1; di++) {
        if (di === 0 && dj === 0) continue;
        const ni = ci + di;
        const nj = cj + dj;
        if (ni < 0 || nj < 0 || ni >= n || nj >= n) continue;
        const nid = nj * n + ni;
        if (closed[nid] || !isFree(g, nid, cfg)) continue;
        const diag = di !== 0 && dj !== 0;
        if (diag && (!isFree(g, cj * n + ni, cfg) || !isFree(g, nj * n + ci, cfg))) continue; // no corner cutting
        const d = g.esdf[nid];
        const pen = d >= pref ? 0 : ((pref - d) / pref) ** 2;
        const gp = gd ? cfg.guideWeight * Math.min(gd[nid] / cfg.guideRange, 1) : 0;
        const step = (diag ? SQRT2 : 1) * g.res * (1 + wc * pen + gp);
        const tentative = gScore[cur] + step;
        if (tentative < gScore[nid]) {
          gScore[nid] = tentative;
          parent[nid] = cur;
          heap.push(nid, tentative + h(ni, nj));
        }
      }
    }
  }
  if (!closed[gId]) return { path: null, expanded, ms: now() - t0, cost: Infinity };
  const cells: Vec2[] = [];
  for (let c = gId; c !== -1; c = parent[c]) {
    const i = c % n;
    cells.push(centerOf(g, i, (c - i) / n));
  }
  cells.reverse();
  cells[0] = [start[0], start[1]];
  cells[cells.length - 1] = [goal[0], goal[1]];
  return { path: cells, expanded, ms: now() - t0, cost: gScore[gId] };
}

/** Nearest free cell centre within `maxR` metres (ring search). */
function snapToFree(g: GridMap, p: Vec2, cfg: PlannerConfig, maxR = 0.8): Vec2 | null {
  const [ci, cj] = cellOf(g, p[0], p[1]);
  if (isFree(g, cj * g.n + ci, cfg)) return p;
  const R = Math.ceil(maxR / g.res);
  let best: Vec2 | null = null;
  let bestD = Infinity;
  for (let dj = -R; dj <= R; dj++) {
    for (let di = -R; di <= R; di++) {
      const i = ci + di;
      const j = cj + dj;
      if (i < 0 || j < 0 || i >= g.n || j >= g.n || !isFree(g, j * g.n + i, cfg)) continue;
      const dd = Math.hypot(di, dj) * g.res;
      if (dd < bestD && dd <= maxR) {
        bestD = dd;
        best = centerOf(g, i, j);
      }
    }
  }
  return best;
}

/* ───────────────────────────── prune, resample, B-spline ───────────────────────────── */

const polyLen = (p: Vec2[]) => {
  let s = 0;
  for (let i = 1; i < p.length; i++) s += Math.hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1]);
  return s;
};

function segmentClear(g: GridMap, a: Vec2, b: Vec2, minClr: number) {
  const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const steps = Math.max(1, Math.ceil(len / (g.res * 0.5)));
  for (let k = 0; k <= steps; k++) {
    const t = k / steps;
    if (sampleEsdf(g, a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t) < minClr) return false;
  }
  return true;
}

/** Greedy line-of-sight shortcut that never trades away clearance. */
function prune(g: GridMap, pts: Vec2[], cfg: PlannerConfig): Vec2[] {
  if (pts.length <= 2) return pts;
  const out: Vec2[] = [pts[0]];
  let i = 0;
  while (i < pts.length - 1) {
    let j = pts.length - 1;
    while (
      j > i + 1 &&
      !(
        maxDeviation(pts, i, j) <= cfg.pruneDeviation &&
        segmentClear(g, pts[i], pts[j], Math.min(cfg.losClearance, minClearanceAlong(g, pts, i, j)))
      )
    )
      j--;
    out.push(pts[j]);
    i = j;
  }
  return out;
}

/** Largest distance of the intermediate A* points from the chord i -> j. */
function maxDeviation(pts: Vec2[], i: number, j: number) {
  let m = 0;
  for (let k = i + 1; k < j; k++) m = Math.max(m, dist2seg(pts[k][0], pts[k][1], pts[i], pts[j]));
  return m;
}

/** The clearance the original A* path itself had between i and j: a shortcut may not be worse than the path it replaces by more than needed. */
function minClearanceAlong(g: GridMap, pts: Vec2[], i: number, j: number) {
  let m = Infinity;
  for (let k = i; k <= j; k++) m = Math.min(m, sampleEsdf(g, pts[k][0], pts[k][1]));
  return m;
}

/** Resample each polyline edge at ~`ds`, keeping every vertex (corners stay control points). */
function resampleEdges(poly: Vec2[], ds: number): Vec2[] {
  const out: Vec2[] = [poly[0]];
  for (let i = 1; i < poly.length; i++) {
    const a = poly[i - 1];
    const b = poly[i];
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const k = Math.max(1, Math.round(len / ds));
    for (let s = 1; s <= k; s++) out.push([a[0] + ((b[0] - a[0]) * s) / k, a[1] + ((b[1] - a[1]) * s) / k]);
  }
  return out;
}

const basis = (t: number): [number, number, number, number] => {
  const t2 = t * t;
  const t3 = t2 * t;
  return [(1 - 3 * t + 3 * t2 - t3) / 6, (4 - 6 * t2 + 3 * t3) / 6, (1 + 3 * t + 3 * t2 - 3 * t3) / 6, t3 / 6];
};

/** Triple the end points so the uniform cubic B-spline starts and ends exactly on them. */
const padCtrl = (c: Vec2[]): Vec2[] => [c[0], c[0], ...c, c[c.length - 1], c[c.length - 1]];

function evalSpline(ctrl: Vec2[], perSeg: number): Vec2[] {
  const P = padCtrl(ctrl);
  const segs = P.length - 3;
  const out: Vec2[] = [];
  for (let k = 0; k < segs; k++) {
    const last = k === segs - 1;
    for (let s = 0; s <= perSeg - (last ? 0 : 1); s++) {
      const b = basis(s / perSeg);
      out.push([
        b[0] * P[k][0] + b[1] * P[k + 1][0] + b[2] * P[k + 2][0] + b[3] * P[k + 3][0],
        b[0] * P[k][1] + b[1] * P[k + 1][1] + b[2] * P[k + 2][1] + b[3] * P[k + 3][1],
      ]);
    }
  }
  return out;
}

interface EgoStep {
  iter: number;
  js: number;
  jc: number;
  jd: number;
}

/**
 * EGO-style refinement. Gradient-free flavour of EGO-Planner's cost:
 *   J = J_smooth (curve acceleration) + J_collision (curve samples inside the soft safety radius) + J_fit (stay near the A* guide)
 * Collision pushes act on the curve samples and are distributed back to the four control points that
 * shape each sample (the "anchor point / repulsive direction" trick), with a least-squares share.
 */
function egoRefine(g: GridMap, ctrl0: Vec2[], pinned: boolean[], cfg: PlannerConfig, safe: number, maxIter: number) {
  const m = ctrl0.length;
  const Q: Vec2[] = ctrl0.map((p) => [p[0], p[1]]);
  const history: EgoStep[] = [];
  const NS = 8;
  let iterations = 0;
  for (let iter = 0; iter < maxIter; iter++) {
    iterations = iter + 1;
    const numX = new Float64Array(m);
    const numY = new Float64Array(m);
    const den = new Float64Array(m);
    let jc = 0;
    const P = padCtrl(Q);
    const segs = P.length - 3;
    for (let k = 0; k < segs; k++) {
      for (let s = 0; s < NS; s++) {
        const b = basis(s / NS);
        const x = b[0] * P[k][0] + b[1] * P[k + 1][0] + b[2] * P[k + 2][0] + b[3] * P[k + 3][0];
        const y = b[0] * P[k][1] + b[1] * P[k + 1][1] + b[2] * P[k + 2][1] + b[3] * P[k + 3][1];
        const d = sampleEsdf(g, x, y);
        if (d >= safe) continue;
        jc += (safe - d) * (safe - d);
        const gr = gradUnit(g, x, y);
        if (gr[0] === 0 && gr[1] === 0) continue;
        const push = safe - d;
        // min-norm way to move this curve sample by `push` along the gradient: control point j moves push * b_j / sum(b^2).
        // A control point averages the moves requested by every sample it shapes (weights b_j), which stays bounded.
        const b2 = b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + b[3] * b[3];
        for (let j = 0; j < 4; j++) {
          const ci = Math.max(0, Math.min(m - 1, k + j - 2));
          const req = (push * b[j]) / b2;
          numX[ci] += b[j] * req * gr[0];
          numY[ci] += b[j] * req * gr[1];
          den[ci] += b[j];
        }
      }
    }
    // smoothness: Laplacian relaxation. It is skipped for the last few iterations so the final pass is feasibility-only.
    const smoothing = iter < maxIter - 6;
    let maxMove = 0;
    const next: Vec2[] = Q.map((p) => [p[0], p[1]]);
    for (let i = 0; i < m; i++) {
      if (pinned[i]) continue;
      let dx = 0;
      let dy = 0;
      if (den[i] > 1e-9) {
        dx += (numX[i] / den[i]) * 0.7;
        dy += (numY[i] / den[i]) * 0.7;
      }
      if (smoothing && i > 0 && i < m - 1) {
        dx += (0.5 * (Q[i - 1][0] + Q[i + 1][0]) - Q[i][0]) * 0.22;
        dy += (0.5 * (Q[i - 1][1] + Q[i + 1][1]) - Q[i][1]) * 0.22;
      }
      const mag = Math.hypot(dx, dy);
      if (mag > 0.06) {
        dx *= 0.06 / mag;
        dy *= 0.06 / mag;
      }
      next[i] = [Q[i][0] + dx, Q[i][1] + dy];
      maxMove = Math.max(maxMove, Math.hypot(dx, dy));
    }
    for (let i = 0; i < m; i++) Q[i] = next[i];

    let js = 0;
    let jd = 0;
    for (let i = 1; i < m - 1; i++) {
      const ax = Q[i - 1][0] - 2 * Q[i][0] + Q[i + 1][0];
      const ay = Q[i - 1][1] - 2 * Q[i][1] + Q[i + 1][1];
      js += ax * ax + ay * ay;
    }
    for (let i = 0; i < m; i++) jd += (Q[i][0] - ctrl0[i][0]) ** 2 + (Q[i][1] - ctrl0[i][1]) ** 2;
    history.push({ iter: iter + 1, js, jc, jd });
    if (maxMove < 4e-4 && iter > 4) break;
  }
  return { ctrl: Q, iterations, history };
}

function chaikin(p: Vec2[], rounds: number): Vec2[] {
  let cur = p;
  for (let r = 0; r < rounds; r++) {
    const out: Vec2[] = [cur[0]];
    for (let i = 0; i < cur.length - 1; i++) {
      const a = cur[i];
      const b = cur[i + 1];
      out.push([a[0] * 0.75 + b[0] * 0.25, a[1] * 0.75 + b[1] * 0.25], [a[0] * 0.25 + b[0] * 0.75, a[1] * 0.25 + b[1] * 0.75]);
    }
    out.push(cur[cur.length - 1]);
    cur = out;
  }
  return cur;
}

/** Re-sample any polyline at uniform arc length. */
export function resampleUniform(poly: Vec2[], ds: number): { samples: Float32Array; cum: Float32Array; length: number } {
  const total = polyLen(poly);
  const n = Math.max(2, Math.ceil(total / ds) + 1);
  const samples = new Float32Array(n * 2);
  const cum = new Float32Array(n);
  let seg = 1;
  let acc = 0;
  let segStart = 0;
  samples[0] = poly[0][0];
  samples[1] = poly[0][1];
  for (let k = 1; k < n; k++) {
    const target = Math.min(total, (k * total) / (n - 1));
    while (seg < poly.length - 1 && acc + Math.hypot(poly[seg][0] - poly[seg - 1][0], poly[seg][1] - poly[seg - 1][1]) < target) {
      acc += Math.hypot(poly[seg][0] - poly[seg - 1][0], poly[seg][1] - poly[seg - 1][1]);
      seg++;
      segStart = acc;
    }
    const a = poly[seg - 1];
    const b = poly[seg];
    const sl = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1;
    const t = Math.min(1, (target - segStart) / sl);
    samples[2 * k] = a[0] + (b[0] - a[0]) * t;
    samples[2 * k + 1] = a[1] + (b[1] - a[1]) * t;
    cum[k] = target;
  }
  return { samples, cum, length: total };
}

function curveStats(g: GridMap, samples: Float32Array) {
  const n = samples.length / 2;
  let minClr = Infinity;
  let maxK = 0;
  let prevA = 0;
  for (let k = 0; k < n; k++) {
    minClr = Math.min(minClr, sampleEsdf(g, samples[2 * k], samples[2 * k + 1]));
    if (k > 0 && k < n - 1) {
      const ax = samples[2 * k] - samples[2 * k - 2];
      const ay = samples[2 * k + 1] - samples[2 * k - 1];
      const bx = samples[2 * k + 2] - samples[2 * k];
      const by = samples[2 * k + 3] - samples[2 * k + 1];
      const a1 = Math.atan2(ay, ax);
      const a2 = Math.atan2(by, bx);
      let da = a2 - a1;
      while (da > Math.PI) da -= 2 * Math.PI;
      while (da < -Math.PI) da += 2 * Math.PI;
      const ds = Math.hypot(bx, by) || 1e-6;
      maxK = Math.max(maxK, Math.abs(da) / ds);
      prevA = a2;
    }
  }
  void prevA;
  return { minClr, maxK };
}

/* ───────────────────────────── mission planner ───────────────────────────── */

interface LegPlan {
  from: string;
  to: string;
  poly: Vec2[];
  end: Vec2;
  stat: LegStat;
}

export function planMission(
  course: CourseData,
  via: ViaPoint[],
  obstacles: DynObstacle[],
  from: Vec2,
  cfg: PlannerConfig = DEFAULT_CONFIG,
  event?: { kind: 'initial' | 'inject' | 'replan' | 'clear'; obstacle?: DynObstacle; priorMinClearance?: number },
  guides?: { primary: Vec2[]; fallback?: Vec2[] },
): PlanResult {
  const T0 = now();
  const log: LogLine[] = [];
  const L = (tag: LogTag, text: string, level: LogLine['level'] = 'info') => log.push({ t: now() - T0, tag, text, level });
  const f2 = (v: number) => v.toFixed(2);

  const grid = buildGrid(course, obstacles, cfg);
  const tGrid = now() - T0;
  const dynCells = obstacles.reduce((s, o) => s + Math.round(((2 * o.hw) / cfg.res) * ((2 * o.hd) / cfg.res)), 0);

  if (event?.kind === 'inject' && event.obstacle) {
    const o = event.obstacle;
    L('EVENT', `dynamic obstacle #${o.id} injected  xy=(${f2(o.x)}, ${f2(o.y)})  ${f2(2 * o.hw)}x${f2(2 * o.hd)} m  h=1.73 m`, 'warn');
    L('MAP', `voxel update: ${Math.round(((2 * o.hw) / cfg.res) * ((2 * o.hd) / cfg.res))} cells -> OCCUPIED  (${dynCells} dynamic total)`);
    L('MAP', `ESDF rebuilt over ${grid.n}x${grid.n} cells in ${tGrid.toFixed(1)} ms  inflation=${f2(cfg.inflation)} m`);
    if (event.priorMinClearance !== undefined) {
      const hit = event.priorMinClearance < cfg.inflation;
      L(
        'PLAN',
        hit
          ? `active trajectory violates keep-out (clearance ${f2(Math.max(0, event.priorMinClearance))} m < ${f2(cfg.inflation)} m) -> REPLAN`
          : `active trajectory still clear (min ${f2(event.priorMinClearance)} m); re-optimising with the new map`,
        hit ? 'warn' : 'info',
      );
    }
  } else if (event?.kind === 'clear') {
    L('EVENT', 'obstacles cleared by operator', 'warn');
    L('MAP', `ESDF rebuilt in ${tGrid.toFixed(1)} ms  (static map only)`);
  } else if (event?.kind === 'replan') {
    L('PLAN', 'replan from live pose', 'info');
  } else {
    L('SYS', `as-built map loaded  ${grid.n}x${grid.n} @ ${cfg.res} m  walls=${course.walls.length}  tunnel=1  window=1`, 'dim');
    L('MAP', `ESDF built in ${tGrid.toFixed(1)} ms  inflation=${f2(cfg.inflation)} m`);
  }

  const goalVia = via[via.length - 1];
  const fail = (reason: string, blockedAt?: Vec2): PlanResult => {
    const total = now() - T0;
    L('WARN', reason, 'err');
    L('FSM', 'mission state -> HOLD  (no feasible trajectory)', 'err');
    return {
      ok: false,
      reason,
      start: from,
      goal: goalVia.at,
      via: [],
      usedFallback: false,
      legs: [],
      polyline: [],
      legPolys: [],
      ctrl: [],
      samples: new Float32Array(0),
      cum: new Float32Array(0),
      length: 0,
      minClearance: 0,
      maxCurvature: 0,
      duration: 0,
      expanded: 0,
      iterations: 0,
      splineMode: 'astar',
      timings: { grid: tGrid, astar: 0, prune: 0, ego: 0, total },
      blockedAt,
      log,
    };
  };

  const start = snapToFree(grid, from, cfg, 0.9);
  if (!start) return fail('start pose is inside a keep-out zone and no free cell is within 0.9 m', from);
  if (start !== from) L('MAP', `start inside keep-out; snapped ${f2(Math.hypot(start[0] - from[0], start[1] - from[1]))} m to nearest free cell`, 'warn');

  const used: ViaPoint[] = [];
  let legGuide = guides ? guideField(grid, guides.primary) : undefined;
  const legs: LegPlan[] = [];
  let cur: Vec2 = start;
  let curLabel = 'START';
  let expandedTotal = 0;
  let tAstar = 0;
  let usedFallback = false;

  const planLeg = (to: ViaPoint, idx: number, total: number): LegPlan | null => {
    const target = snapToFree(grid, to.at, cfg, 0.6);
    if (!target) {
      L('A*', `leg ${idx}/${total} ${curLabel} -> ${to.label}: target cell is occupied`, 'err');
      return null;
    }
    if (target !== to.at) L('MAP', `${to.label} waypoint inside keep-out; snapped ${f2(Math.hypot(target[0] - to.at[0], target[1] - to.at[1]))} m`, 'warn');
    L('A*', `leg ${idx}/${total}  ${curLabel} -> ${to.label}  open={start}  w_clr=${cfg.clearanceWeight}`, 'dim');
    const r = astar(grid, cur, target, cfg, legGuide);
    tAstar += r.ms;
    expandedTotal += r.expanded;
    if (!r.path) {
      L('A*', `leg ${idx}/${total}  OPEN SET EXHAUSTED after ${r.expanded.toLocaleString('en-US')} expansions (${r.ms.toFixed(1)} ms): no route`, 'err');
      return null;
    }
    const len = polyLen(r.path);
    L('A*', `A* voxel expansion: ${r.ms.toFixed(1)}ms | ${r.expanded.toLocaleString('en-US')} cells | leg ${idx}/${total} ${curLabel} -> ${to.label} | ${len.toFixed(2)} m`, 'ok');
    return { from: curLabel, to: to.label, poly: r.path, end: target, stat: { from: curLabel, to: to.label, ok: true, expanded: r.expanded, ms: r.ms, length: len } };
  };

  // number of legs we expect (for the n/m labels)
  const totalLegs = via.length;
  let idx = 0;
  let legNo = 0;
  let blockedAt: Vec2 | undefined;
  while (idx < via.length) {
    const v = via[idx];
    if (v.group) {
      let end = idx;
      while (end < via.length && via[end].group === v.group) end++;
      const block = via.slice(idx, end);
      const saved = { cur, curLabel, legs: legs.length, used: used.length, exp: expandedTotal };
      let ok = true;
      for (const bv of block) {
        legNo++;
        const lp = planLeg(bv, legNo, totalLegs);
        if (!lp) {
          ok = false;
          blockedAt = bv.at;
          break;
        }
        legs.push(lp);
        used.push(bv);
        cur = lp.end;
        curLabel = bv.label;
      }
      if (!ok) {
        legs.length = saved.legs;
        used.length = saved.used;
        cur = saved.cur;
        curLabel = saved.curLabel;
        usedFallback = true;
        if (guides?.fallback) legGuide = guideField(grid, guides.fallback);
        L('FSM', `checkpoint 4 (${block[0].label}) unreachable -> switching to GATE B backup route`, 'warn');
        blockedAt = undefined;
      }
      idx = end;
      continue;
    }
    legNo++;
    const lp = planLeg(v, legNo, totalLegs);
    if (!lp) return fail(`no feasible route to ${v.label}`, v.at);
    legs.push(lp);
    used.push(v);
    cur = lp.end;
    curLabel = v.label;
    idx++;
  }
  if (!legs.length) return fail('nothing to plan');

  // prune each leg, then stitch
  const tPrune0 = now();
  let rawLen = 0;
  const poly: Vec2[] = [];
  const legBreaks: number[] = []; // indices into `poly` that are via points
  const legPolys: Vec2[][] = [];
  let before = 0;
  for (const lg of legs) {
    before += lg.poly.length;
    const pr = prune(grid, lg.poly, cfg);
    rawLen += lg.poly.length;
    legPolys.push(pr.map((p) => [p[0], p[1]] as Vec2));
    if (poly.length) pr.shift();
    poly.push(...pr);
    legBreaks.push(poly.length - 1);
  }
  const tPrune = now() - tPrune0;
  L('PRUNE', `line-of-sight shortcut (clr >= ${f2(cfg.losClearance)} m): ${before} -> ${poly.length} waypoints  (${tPrune.toFixed(1)} ms)`, 'info');
  void rawLen;

  // control points: every polyline vertex is kept, edges resampled at ctrlSpacing
  const tEgo0 = now();
  const ctrl0: Vec2[] = [poly[0]];
  const pinned: boolean[] = [true];
  const viaIdx = new Set(legBreaks);
  for (let i = 1; i < poly.length; i++) {
    const a = poly[i - 1];
    const b = poly[i];
    const len = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const k = Math.max(1, Math.round(len / cfg.ctrlSpacing));
    for (let s = 1; s <= k; s++) {
      ctrl0.push([a[0] + ((b[0] - a[0]) * s) / k, a[1] + ((b[1] - a[1]) * s) / k]);
      pinned.push(s === k && viaIdx.has(i));
    }
  }
  pinned[ctrl0.length - 1] = true;
  // keep the last two control points fixed too so the end tangent is stable
  if (ctrl0.length > 3) pinned[ctrl0.length - 2] = true;
  if (ctrl0.length > 3) pinned[1] = true;
  L('EGO', `EGO B-spline recalculating  p=3  ctrl=${ctrl0.length}  knots=${ctrl0.length + 4}  ds=${f2(cfg.ctrlSpacing)} m  pinned=${pinned.filter(Boolean).length}`, 'info');

  let safe = cfg.safe;
  let refined = egoRefine(grid, ctrl0, pinned, cfg, safe, 48);
  let curve = evalSpline(refined.ctrl, 12);
  let uni = resampleUniform(curve, 0.05);
  let stats = curveStats(grid, uni.samples);
  let mode: PlanResult['splineMode'] = 'ego';
  const hardMin = cfg.inflation - 0.03;

  // log a thin slice of the optimiser's history
  const hist = refined.history;
  const show = new Set<number>([0, 1, 2, 3, Math.floor(hist.length / 2), hist.length - 1]);
  hist.forEach((h, k) => {
    if (show.has(k))
      L('EGO', `iter ${String(h.iter).padStart(2, '0')}  J_smooth=${h.js.toFixed(3)}  J_coll=${h.jc.toFixed(3)}  J_fit=${h.jd.toFixed(3)}`, k === hist.length - 1 ? 'ok' : 'dim');
  });

  if (stats.minClr < hardMin) {
    L('EGO', `curve dips to ${f2(stats.minClr)} m (< ${f2(hardMin)}): tightening, safe radius +0.08 m`, 'warn');
    safe += 0.08;
    refined = egoRefine(grid, refined.ctrl, pinned, cfg, safe, 40);
    curve = evalSpline(refined.ctrl, 12);
    uni = resampleUniform(curve, 0.05);
    stats = curveStats(grid, uni.samples);
  }
  if (stats.minClr < hardMin) {
    const rounded = chaikin(poly, 2);
    const ru = resampleUniform(rounded, 0.05);
    const rs = curveStats(grid, ru.samples);
    if (rs.minClr >= hardMin) {
      uni = ru;
      stats = rs;
      mode = 'rounded';
      L('EGO', 'fallback: corner-rounded polyline (spline could not clear the narrow gap)', 'warn');
    } else {
      const au = resampleUniform(poly, 0.05);
      uni = au;
      stats = curveStats(grid, au.samples);
      mode = 'astar';
      L('EGO', 'fallback: raw A* polyline (gap narrower than the soft radius)', 'warn');
    }
  }
  const tEgo = now() - tEgo0;

  const duration = uni.length / cfg.speed;
  const total = now() - T0;
  L(
    'EGO',
    mode === 'ego'
      ? `converged in ${refined.iterations} iterations  (${tEgo.toFixed(1)} ms)  min_clr=${f2(stats.minClr)} m  max_curv=${f2(stats.maxK)} 1/m`
      : `done (${mode})  min_clr=${f2(stats.minClr)} m  max_curv=${f2(stats.maxK)} 1/m`,
    'ok',
  );
  L('TRACK', `trajectory published: ${uni.length.toFixed(2)} m  T=${duration.toFixed(1)} s @ ${cfg.speed} m/s  ${refined.ctrl.length} ctrl pts`, 'ok');
  if (usedFallback) L('FSM', 'route = GATE B backup (window skipped)', 'warn');
  L('OK', `plan ready in ${total.toFixed(1)} ms  (A* ${tAstar.toFixed(1)} + prune ${tPrune.toFixed(1)} + EGO ${tEgo.toFixed(1)})  expanded=${expandedTotal.toLocaleString('en-US')}`, 'ok');

  return {
    ok: true,
    start,
    goal: goalVia.at,
    via: used,
    usedFallback,
    legs: legs.map((l) => l.stat),
    polyline: poly,
    legPolys,
    ctrl: refined.ctrl,
    samples: uni.samples,
    cum: uni.cum,
    length: uni.length,
    minClearance: stats.minClr,
    maxCurvature: stats.maxK,
    duration,
    expanded: expandedTotal,
    iterations: refined.iterations,
    splineMode: mode,
    timings: { grid: tGrid, astar: tAstar, prune: tPrune, ego: tEgo, total },
    blockedAt,
    log,
  };
}

/** Smallest ESDF value along an existing plan against a fresh map: used to tell "path blocked" from "path still clear". */
export function planClearanceAgainst(course: CourseData, plan: PlanResult, obstacles: DynObstacle[], cfg: PlannerConfig = DEFAULT_CONFIG): number {
  if (!plan.ok) return 0;
  const g = buildGrid(course, obstacles, cfg);
  let m = Infinity;
  for (let k = 0; k < plan.samples.length / 2; k++) m = Math.min(m, sampleEsdf(g, plan.samples[2 * k], plan.samples[2 * k + 1]));
  return m;
}

/** Position + heading at arc length `s` along a plan. */
export function pointAt(plan: PlanResult, s: number): { x: number; y: number; heading: number } {
  const n = plan.cum.length;
  if (n < 2) return { x: plan.samples[0] ?? 0, y: plan.samples[1] ?? 0, heading: 0 };
  const c = Math.max(0, Math.min(plan.length, s));
  let lo = 0;
  let hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (plan.cum[mid] <= c) lo = mid;
    else hi = mid;
  }
  const seg = plan.cum[hi] - plan.cum[lo] || 1;
  const t = (c - plan.cum[lo]) / seg;
  const x0 = plan.samples[2 * lo];
  const y0 = plan.samples[2 * lo + 1];
  const x1 = plan.samples[2 * hi];
  const y1 = plan.samples[2 * hi + 1];
  return { x: x0 + (x1 - x0) * t, y: y0 + (y1 - y0) * t, heading: Math.atan2(y1 - y0, x1 - x0) };
}

/** Is a world point inside static or dynamic occupancy (for "drop on a wall" rejection)? */
export function occupiedAt(course: CourseData, obstacles: DynObstacle[], x: number, y: number, cfg: PlannerConfig = DEFAULT_CONFIG): 'free' | 'static' | 'dynamic' {
  const g = buildGrid(course, [], cfg);
  const [i, j] = cellOf(g, x, y);
  if (g.staticOcc[j * g.n + i]) return 'static';
  for (const o of obstacles) if (Math.abs(x - o.x) <= o.hw && Math.abs(y - o.y) <= o.hd) return 'dynamic';
  return 'free';
}
