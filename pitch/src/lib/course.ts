/**
 * The as-built DroneHacks course + skybeta-19's real route, typed, plus the few derived values the
 * scenes need. Source of truth is git (see src/data/provenance.json); this file only reshapes it.
 */
import courseJson from '@/data/course.json';
import routeJson from '@/data/route-window.json';
import routeBJson from '@/data/route-gate-b.json';
import provenanceJson from '@/data/provenance.json';
import type { CourseData, RouteData, Vec2, Vec3 } from './course-types';
import { buildVia } from './via';

export const COURSE = courseJson as unknown as CourseData;
export const ROUTE = routeJson as unknown as RouteData;
export const ROUTE_B = routeBJson as unknown as { waypoints: Vec3[] };
export const PROVENANCE = provenanceJson as {
  ref: string;
  refCommit: string;
  files: Record<string, { path: string; lastCommit: string; date: string; subject: string }>;
};

export const { via: VIA, anchors: ROUTE_ANCHORS } = buildVia(COURSE, ROUTE);
export const GUIDES = {
  primary: ROUTE.waypoints.map((w) => [w[0], w[1]] as Vec2),
  fallback: ROUTE_B.waypoints.map((w) => [w[0], w[1]] as Vec2),
};

export const START: Vec2 = COURSE.launch;
export const PAD: Vec2 = COURSE.pad.center;
export const WALL_H = 1.7272;
export const NET_HALF = 3.048;
export const FLIGHT_Z = 0.85; // cruise height used by the real route (m)
export const DRONE_SPAN = 0.475; // overall width with props (m), from course_asbuilt.json

/** ENU (x east, y north, z up) -> three.js (x, y up, z = -north). */
export const enu = (x: number, y: number, z = 0): [number, number, number] => [x, z, -y];

/** cumulative arc length along the real route */
export const ROUTE_CUM: Float32Array = (() => {
  const w = ROUTE.waypoints;
  const c = new Float32Array(w.length);
  for (let i = 1; i < w.length; i++) c[i] = c[i - 1] + Math.hypot(w[i][0] - w[i - 1][0], w[i][1] - w[i - 1][1], w[i][2] - w[i - 1][2]);
  return c;
})();
export const ROUTE_LEN = ROUTE_CUM[ROUTE_CUM.length - 1];

/** fraction (0..1) of the real route's length at a waypoint index */
export const routeFrac = (i: number) => ROUTE_CUM[Math.max(0, Math.min(ROUTE_CUM.length - 1, i))] / ROUTE_LEN;

/** ENU position + heading on the real route at fraction u (0..1) */
export function routeAt(u: number): { x: number; y: number; z: number; heading: number } {
  const w = ROUTE.waypoints;
  const s = Math.max(0, Math.min(1, u)) * ROUTE_LEN;
  let lo = 0;
  let hi = w.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (ROUTE_CUM[mid] <= s) lo = mid;
    else hi = mid;
  }
  const seg = ROUTE_CUM[hi] - ROUTE_CUM[lo] || 1;
  const t = (s - ROUTE_CUM[lo]) / seg;
  const a = w[lo];
  const b = w[hi];
  return {
    x: a[0] + (b[0] - a[0]) * t,
    y: a[1] + (b[1] - a[1]) * t,
    z: a[2] + (b[2] - a[2]) * t,
    heading: Math.atan2(b[1] - a[1], b[0] - a[0]),
  };
}

/** Anchor fractions (0..1) of the key route events, used to pace the ghost flight against the scroll. */
export const ROUTE_F = {
  gateA: routeFrac(ROUTE_ANCHORS.gateA),
  tubeIn: routeFrac(ROUTE_ANCHORS.tubeIn),
  tubeOut: routeFrac(ROUTE_ANCHORS.tubeOut),
  window: routeFrac(ROUTE_ANCHORS.window),
};

/** checkpoint marker positions are the same five scored checkpoints the README lists (4 + 7 + 10 + 7 + 7 = 35 pts) */
export const CHECKPOINTS = COURSE.checkpoints;
export const TOTAL_POINTS = COURSE.checkpoints.reduce((s, c) => s + c.points, 0);
