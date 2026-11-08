/**
 * director.ts: the 2-minute auto-demo, as data.
 *
 *   DEMO CLOCK (s)  ->  scroll progress p   piecewise-linear timeline; every text window, the drone anchors, the maze
 *                                          reveal and the ghost flight already follow `progress`, so they sync for free
 *   DEMO CLOCK (s)  ->  camera              THREE.CatmullRomCurve3 for the camera position and one for the look-at target.
 *                                          Control points carry timestamps; a monotone cubic (PCHIP) maps time onto the curve
 *                                          parameter, so the camera glides THROUGH every shot instead of braking at each one.
 *
 *   0:00  macro of the Jetson Orin Nano board, chassis dissolving in
 *   0:10  pull back: the liquid-chrome airframe, type slides in from behind it, one 360 deg turn
 *   0:34  lift off the hero, LiDAR cloud resolves into the as-built maze
 *   0:46  top-down sweep of the whole course:  [01] A* + EGO B-spline
 *   0:58  chase cam on the real route:         [02] OAK-D-S2 stereo odometry
 *   1:16  tunnel cutaway:                      [03] blind carry through the tunnel
 *   1:40  isometric top-down, scroll locks, OVERRIDE MODE
 *   1:44  scripted blocker drops, planner logs spam, control released to the judge at 1:58
 */
import * as THREE from 'three';
import { motionValue } from 'framer-motion';
import { CAMERA_KEYS, ghostU } from './choreography';
import { enu, routeAt } from './course';
import { clamp, smoothstep } from './math';

export const DEMO_DURATION = 120;
export const OVERRIDE_AT = 100;
export const RELEASE_AT = 118;

/** seconds since the demo started; DOM captions subscribe to this */
export const demoTime = motionValue(0);

export const demo = {
  active: false,
  paused: false,
  t: 0,
  /** QA: jump the clock (and snap the camera) on the next frame */
  seek: null as number | null,
  flags: { override: false, drop1: false, drop2: false, release: false },
};
export const resetDemoFlags = () => {
  demo.flags = { override: false, drop1: false, drop2: false, release: false };
};

/* ───────────────────────────── time -> scroll progress ───────────────────────────── */

const P_KEYS: [number, number][] = [
  [0, 0.02], // macro: hero type + HUD wait
  [10, 0.02],
  [34, 0.34], // hero: type in, 360 turn, tags
  [46, 0.5], // scan -> map
  [58, 0.62], // callout 01 window = p 0.50 - 0.62
  [76, 0.76], // callout 02 window = p 0.64 - 0.76
  [92, 0.9], // callout 03 window = p 0.78 - 0.90
  [100, 0.985], // lift to the isometric view
  [101, 1],
  [DEMO_DURATION, 1],
];

export function pOfT(t: number): number {
  if (t <= P_KEYS[0][0]) return P_KEYS[0][1];
  for (let i = 0; i < P_KEYS.length - 1; i++) {
    const [t0, p0] = P_KEYS[i];
    const [t1, p1] = P_KEYS[i + 1];
    if (t <= t1) return p0 + ((p1 - p0) * (t - t0)) / (t1 - t0);
  }
  return 1;
}

/** the chassis dissolves in during the macro shot, on the demo clock rather than on scroll */
export const revealOfT = (t: number) => smoothstep(0.4, 7.5, t);

/* ───────────────────────────── monotone cubic (PCHIP) ───────────────────────────── */

function endSlope(h0: number, h1: number, d0: number, d1: number) {
  let m = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
  if (Math.sign(m) !== Math.sign(d0)) m = 0;
  else if (Math.sign(d0) !== Math.sign(d1) && Math.abs(m) > 3 * Math.abs(d0)) m = 3 * d0;
  return m;
}

/** Fritsch-Carlson shape-preserving interpolant: smooth (C1), never overshoots between knots. */
export function pchip(xs: number[], ys: number[]): (x: number) => number {
  const n = xs.length;
  const h: number[] = [];
  const delta: number[] = [];
  for (let i = 0; i < n - 1; i++) {
    h[i] = xs[i + 1] - xs[i];
    delta[i] = (ys[i + 1] - ys[i]) / h[i];
  }
  const d = new Array<number>(n).fill(0);
  if (n === 2) {
    d[0] = d[1] = delta[0];
  } else {
    for (let i = 1; i < n - 1; i++) {
      if (delta[i - 1] * delta[i] > 0) {
        const w1 = 2 * h[i] + h[i - 1];
        const w2 = h[i] + 2 * h[i - 1];
        d[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
      }
    }
    d[0] = endSlope(h[0], h[1], delta[0], delta[1]);
    d[n - 1] = endSlope(h[n - 2], h[n - 3], delta[n - 2], delta[n - 3]);
  }
  return (x: number) => {
    const c = clamp(x, xs[0], xs[n - 1]);
    let i = 0;
    while (i < n - 2 && c > xs[i + 1]) i++;
    const t = (c - xs[i]) / h[i];
    const t2 = t * t;
    const t3 = t2 * t;
    return (2 * t3 - 3 * t2 + 1) * ys[i] + (t3 - 2 * t2 + t) * h[i] * d[i] + (-2 * t3 + 3 * t2) * ys[i + 1] + (t3 - t2) * h[i] * d[i + 1];
  };
}

/* ───────────────────────────── the camera path ───────────────────────────── */

type V3 = [number, number, number];
interface Shot {
  t: number;
  pos: V3;
  look: V3;
  fov: number;
}

const key = (t: number, p: number): Shot => {
  const c = CAMERA_KEYS.find((k) => k.p === p)!;
  return { t, pos: c.pos, look: c.look, fov: c.fov };
};

/** behind-and-above the ghost drone at demo time t, so the chase cam is a pre-computed rail, not a live follow */
const chase = (t: number): Shot => {
  const g = routeAt(ghostU(pOfT(t)));
  const w = enu(g.x, g.y, g.z);
  return {
    t,
    pos: [w[0] - Math.cos(g.heading) * 3.3, w[1] + 3.9, w[2] + Math.sin(g.heading) * 3.3],
    look: [w[0], w[1] + 0.05, w[2]],
    fov: 34,
  };
};

// The hero drone sits at the origin at 7x scale, yaw fixed during the macro. Positions are three.js axes (x east, y up, z = -north).
const SHOTS: Shot[] = [
  // ── 0:00 macro. At Jetson height, looking into the gap between the two chrome plates, right on the Orin Nano board
  { t: 0, pos: [-1.0, -0.22, 1.3], look: [0.14, -0.23, 0.14], fov: 26 },
  { t: 5, pos: [-0.55, -0.2, 0.98], look: [0.14, -0.23, 0.12], fov: 23 },
  // ── 0:10 pull back to the full liquid-chrome airframe
  { t: 10, pos: [0.4, -0.1, 2.9], look: [0, -0.06, 0], fov: 28 },
  { t: 16, pos: [0.0, 0.55, 6.2], look: [0, 0, 0], fov: 30 },
  { t: 24, pos: [-1.0, 0.8, 7.9], look: [0.15, 0, 0], fov: 28 },
  key(34, 0.34),
  // ── 0:34 lift to the maze
  { t: 40, pos: [2.4, 5.2, 9.4], look: [0, 0.1, 0.1], fov: 31 },
  key(46, 0.46),
  // ── callout 01: sweep the whole plan
  key(52, 0.52),
  key(58, 0.6),
  // ── callout 02: chase the route
  chase(61),
  chase(64),
  chase(67),
  chase(70),
  chase(73),
  chase(76),
  // ── callout 03: tunnel cutaway
  key(80, 0.8),
  { t: 85, pos: [5.0, 4.5, 1.7], look: [2.3, 0.5, -0.25], fov: 33 },
  { t: 90, pos: [4.2, 4.0, -0.7], look: [2.3, 0.5, 0.05], fov: 33 },
  // ── lift to the isometric top-down and hold for OVERRIDE
  key(96, 0.9),
  key(100, 1.0),
  key(DEMO_DURATION, 1.0),
];

const posCurve = new THREE.CatmullRomCurve3(
  SHOTS.map((s) => new THREE.Vector3(...s.pos)),
  false,
  'centripetal',
  0.5,
);
const lookCurve = new THREE.CatmullRomCurve3(
  SHOTS.map((s) => new THREE.Vector3(...s.look)),
  false,
  'centripetal',
  0.5,
);
const uOfT = pchip(
  SHOTS.map((s) => s.t),
  SHOTS.map((_, i) => i / (SHOTS.length - 1)),
);
const fovOfT = pchip(
  SHOTS.map((s) => s.t),
  SHOTS.map((s) => s.fov),
);

/** camera pose at demo time t; writes into the given vectors and returns the FOV */
export function cameraAt(t: number, pos: THREE.Vector3, look: THREE.Vector3): number {
  const u = clamp(uOfT(t));
  posCurve.getPoint(u, pos);
  lookCurve.getPoint(u, look);
  return fovOfT(t);
}

/* ───────────────────────────── captions (HTML reveals synced to the clock) ───────────────────────────── */

export interface Caption {
  a: number;
  b: number;
  tag: string;
  lines: string[];
}
export const CAPTIONS: Caption[] = [
  { a: 0.9, b: 5.2, tag: 'JETSON ORIN NANO', lines: ['The onboard computer.', 'It runs vision, mapping and planning.'] },
  { a: 5.7, b: 9.8, tag: 'OAK-D-S2', lines: ['A pair of cameras, 75 mm apart.', 'They measure motion with no GPS.'] },
  { a: 10.5, b: 16.0, tag: 'SKY BETA', lines: ['The airframe: 4 motors, a 4S battery,', 'and a Pixhawk 6C flight controller.'] },
  // (no caption in OVERRIDE: the override panel's own hint line says the drops are scripted and when control returns)
];
