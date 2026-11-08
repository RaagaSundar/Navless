/**
 * Mission via-points, derived from skybeta-19's real as-built route so every one of them is
 * guaranteed to be free space on the real course (the route is clearance-checked in Gazebo).
 *
 *   start -> GATE A -> TUBE N -> TUBE S -> WINDOW (in / centre / out)* -> PAD
 *                                           * optional: if the window is blocked the planner
 *                                             falls back to the Gate B backup route
 */
import type { CourseData, RouteData, Vec2 } from './course-types.ts';
import type { ViaPoint } from './planner.ts';

const d2 = (a: Vec2, b: Vec2) => Math.hypot(a[0] - b[0], a[1] - b[1]);

export interface RouteAnchors {
  /** route sample indices */
  gateA: number;
  tubeIn: number;
  tubeOut: number;
  window: number;
  pad: number;
}

export function buildVia(course: CourseData, route: RouteData): { via: ViaPoint[]; anchors: RouteAnchors } {
  const pts = route.waypoints;
  const xy = (i: number): Vec2 => [pts[i][0], pts[i][1]];
  const nearest = (p: Vec2) => {
    let best = 0;
    let bd = Infinity;
    for (let i = 0; i < pts.length; i++) {
      const d = d2(xy(i), p);
      if (d < bd) {
        bd = d;
        best = i;
      }
    }
    return best;
  };

  const gA = course.gates.gate_A;
  const gateA = nearest([(gA.a[0] + gA.b[0]) / 2, (gA.a[1] + gA.b[1]) / 2]);

  const t = course.tunnel;
  let first = -1;
  let last = -1;
  for (let i = 0; i < pts.length; i++) {
    const [x, y] = xy(i);
    if (x > t.xMin && x < t.xMax && y > t.yMin && y < t.yMax) {
      if (first < 0) first = i;
      last = i;
    }
  }
  if (first < 0) throw new Error('route never enters the tunnel; check route-window.json');
  const margin = Math.round(0.3 / route.meta.spacingM);
  const tubeIn = Math.max(0, first - margin);
  const tubeOut = Math.min(pts.length - 1, last + margin);

  const win = course.walls.find((w) => w.window)!.window!;
  const window = nearest(win.center);
  const wMargin = Math.round(0.7 / route.meta.spacingM);
  const winIn = Math.max(0, window - wMargin);
  const winOut = Math.min(pts.length - 1, window + wMargin);

  const via: ViaPoint[] = [
    { id: 'gateA', label: 'GATE A', at: xy(gateA), kind: 'gate' },
    { id: 'tubeN', label: 'TUBE N', at: xy(tubeIn), kind: 'tube' },
    { id: 'tubeS', label: 'TUBE S', at: xy(tubeOut), kind: 'tube' },
    { id: 'winIn', label: 'WINDOW IN', at: xy(winIn), kind: 'window', group: 'window' },
    { id: 'win', label: 'WINDOW', at: xy(window), kind: 'window', group: 'window' },
    { id: 'winOut', label: 'WINDOW OUT', at: xy(winOut), kind: 'window', group: 'window' },
    { id: 'pad', label: 'PAD', at: [course.pad.center[0], course.pad.center[1]], kind: 'pad' },
  ];
  return { via, anchors: { gateA, tubeIn, tubeOut, window, pad: pts.length - 1 } };
}
