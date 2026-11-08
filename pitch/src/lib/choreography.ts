/**
 * choreography.ts: the master timeline.
 *
 * One scroll value (`progress`, 0..1) drives everything. Every frame `updateView()` turns it into a single
 * mutable `view` object (no allocations, no React renders) that every R3F component reads from:
 *
 *   SCENE 1  THE HERO REVEAL   p 0.00 - 0.36   drone materialises from black, 360 deg turn, HUD + cuVSLAM cloud fade in
 *   SCENE 2  THE ENVIRONMENT   p 0.36 - 0.92   drone shrinks to the corner, LiDAR cloud -> wireframe maze, ghost flight path
 *   SCENE 3  OVERRIDE MODE     p 0.92 - 1.00   camera lifts to the isometric view, scroll locks, judge takes over
 *
 * Camera keyframes live in three.js axes (x east, y up, z = -north), metres. Between keys the camera eases
 * with a half-linear / half-smooth curve so it always creeps but holds on each beat.
 */
import * as THREE from 'three';
import { clamp, damp, easeInOutCubic, lerp, remap, smoothstep } from './math';
import { overrideMix, progress, scrollVelocity } from './motion';
import { ROUTE_F, enu, routeAt } from './course';

export const SCROLL_VH = 820;

export const T = {
  hero: [0.0, 0.36] as const,
  toMap: [0.34, 0.46] as const,
  tour: [0.46, 0.92] as const,
  toOverride: [0.9, 0.985] as const,
  /** the three tech-stack beats of scene 2 */
  callouts: [
    [0.5, 0.62],
    [0.64, 0.76],
    [0.78, 0.9],
  ] as const,
  /** scroll position at which the page locks and OVERRIDE MODE engages */
  lock: 0.985,
};

type V3 = [number, number, number];
export interface Key {
  p: number;
  pos: V3;
  look: V3;
  fov: number;
}

export const CAMERA_KEYS: Key[] = [
  // ── scene 1: orbiting a drone that sits at the origin, 6x scale
  // look a touch low: the opening composition (type + drone) sits high enough to leave the bottom third to the HUD
  { p: 0.0, pos: [0.0, 0.35, 10.8], look: [0, -0.5, 0], fov: 30 },
  { p: 0.1, pos: [0.0, 0.5, 9.2], look: [0, 0, 0], fov: 30 },
  { p: 0.22, pos: [-1.0, 0.8, 7.9], look: [0.15, 0, 0], fov: 28 },
  { p: 0.34, pos: [0.7, 1.35, 7.2], look: [0, 0.05, 0], fov: 27 },
  // ── scene 2: the maze
  { p: 0.46, pos: [3.6, 9.6, 10.6], look: [0, 0, 0.2], fov: 34 },
  { p: 0.52, pos: [2.6, 9.0, 7.4], look: [-0.3, 0, -0.5], fov: 34 },
  { p: 0.6, pos: [2.2, 8.4, 5.0], look: [-0.6, 0, -1.2], fov: 34 },
  { p: 0.7, pos: [1.4, 7.0, 1.6], look: [0.8, 0.2, -1.8], fov: 34 },
  { p: 0.8, pos: [5.8, 4.9, 3.6], look: [2.3, 0.55, -0.5], fov: 34 },
  { p: 0.9, pos: [1.2, 9.6, 7.4], look: [-0.2, 0, -0.2], fov: 34 },
  // ── scene 3: isometric top-down. Narrow FOV + long lens = near-orthographic.
  { p: 1.0, pos: [4.8, 21.0, 9.6], look: [0, 0, -0.3], fov: 18 },
];

/** how far along the real route the ghost flight has got, per scroll position. Anchors come from the real route data. */
const GHOST_KEYS: [number, number][] = [
  [0.47, 0.0],
  [0.62, ROUTE_F.gateA],
  [0.76, ROUTE_F.tubeIn],
  [0.86, ROUTE_F.tubeOut],
  [0.95, 1.0],
];

export interface View {
  p: number;
  pRaw: number;
  o: number;
  vel: number;
  dt: number;
  time: number;
  /** drone anchor blend weights (sum to 1) */
  heroW: number;
  cornerW: number;
  startW: number;
  droneReveal: number;
  heroYaw: number;
  /** hero type + HUD */
  textIn: number;
  textOut: number;
  hudIn: number;
  /** environment */
  mazeIn: number;
  scanIn: number;
  scanBase: number;
  ghostU: number;
  ghostShow: number;
  callout: number;
  calloutMix: [number, number, number];
  followW: number;
  /** camera target (before damping) */
  camPos: THREE.Vector3;
  camLook: THREE.Vector3;
  camFov: number;
  /** impulse for camera shake (decays) */
  shake: number;
  /** QA / deep links: while > 0, smoothing is bypassed so a jump lands exactly (counts down in frames) */
  snap: number;
  /** AutoDirector owns the camera: `camPos / camLook / camFov` are written by the director, not sampled from scroll keys */
  directed: boolean;
  /** AutoDirector can pin the drone's dissolve-in to the demo clock instead of the scroll position */
  revealOverride: number | null;
  /** opening reveal (manual mode): set once the boot screen clears, then `introT` counts 0 -> INTRO_S seconds */
  introGo: boolean;
  introT: number;
}

/** seconds the hero takes to build itself when the page opens, before anyone has scrolled */
export const INTRO_S = 4.4;

export const view: View = {
  p: 0,
  pRaw: 0,
  o: 0,
  vel: 0,
  dt: 1 / 60,
  time: 0,
  heroW: 1,
  cornerW: 0,
  startW: 0,
  droneReveal: 0,
  heroYaw: 0,
  textIn: 0,
  textOut: 0,
  hudIn: 0,
  mazeIn: 0,
  scanIn: 0,
  scanBase: 0,
  ghostU: 0,
  ghostShow: 0,
  callout: -1,
  calloutMix: [0, 0, 0],
  followW: 0,
  camPos: new THREE.Vector3(0, 0.35, 10.8),
  camLook: new THREE.Vector3(),
  camFov: 30,
  shake: 0,
  snap: 0,
  directed: false,
  revealOverride: null,
  introGo: false,
  introT: 0,
};

/** Ask the camera for a soft settle (blocker impact). 0..1, capped low on purpose: it is a nudge, never a shake. */
export const kick = (amount: number) => {
  view.shake = Math.min(0.4, view.shake + amount);
};

const _a = new THREE.Vector3();
const _b = new THREE.Vector3();

function sampleKeys(p: number) {
  let i = 0;
  while (i < CAMERA_KEYS.length - 2 && p > CAMERA_KEYS[i + 1].p) i++;
  const a = CAMERA_KEYS[i];
  const b = CAMERA_KEYS[i + 1];
  const u = remap(p, a.p, b.p);
  const t = lerp(u, easeInOutCubic(u), 0.55);
  view.camPos.set(lerp(a.pos[0], b.pos[0], t), lerp(a.pos[1], b.pos[1], t), lerp(a.pos[2], b.pos[2], t));
  view.camLook.set(lerp(a.look[0], b.look[0], t), lerp(a.look[1], b.look[1], t), lerp(a.look[2], b.look[2], t));
  view.camFov = lerp(a.fov, b.fov, t);
}

export function ghostU(p: number) {
  if (p <= GHOST_KEYS[0][0]) return 0;
  for (let i = 0; i < GHOST_KEYS.length - 1; i++) {
    const [p0, u0] = GHOST_KEYS[i];
    const [p1, u1] = GHOST_KEYS[i + 1];
    if (p <= p1) return lerp(u0, u1, easeInOutCubic(remap(p, p0, p1)) * 0.35 + remap(p, p0, p1) * 0.65);
  }
  return 1;
}

const windowMix = (p: number, a: number, b: number) => smoothstep(a, a + 0.018, p) * (1 - smoothstep(b - 0.018, b, p));

/** Advance the shared view by one frame. Call once per frame, before anything that reads `view`. */
export function updateView(dt: number, time: number) {
  const v = view;
  v.dt = dt;
  v.time = time;
  v.pRaw = progress.get();
  v.p = v.snap > 0 || v.directed ? v.pRaw : damp(v.p, v.pRaw, 7, dt);
  if (Math.abs(v.p - v.pRaw) < 1e-4) v.p = v.pRaw;
  const p = v.p;
  v.o = overrideMix.get();
  v.vel = damp(v.vel, scrollVelocity.get(), 6, dt);
  v.shake = damp(v.shake, 0, 5, dt);

  // drone anchors
  const a = smoothstep(T.toMap[0], T.toMap[1], p);
  const b = smoothstep(T.toOverride[0], T.toOverride[1], p);
  v.heroW = 1 - a;
  v.cornerW = a * (1 - b);
  v.startW = a * b;
  // Opening reveal: in manual mode the hero builds itself on a clock once the boot screen clears, so the first screen is
  // never an empty void waiting for a scroll tick. Scrolling takes over from there (max of the two). The AutoDirector has
  // its own macro-shot dissolve on the demo clock, so while it flies the camera this contributes nothing.
  if (v.introGo && !v.directed && v.introT < INTRO_S) v.introT = Math.min(INTRO_S, v.introT + dt);
  const introOn = v.directed ? 0 : 1;
  v.droneReveal = v.revealOverride ?? Math.max(smoothstep(0.012, 0.2, p), introOn * smoothstep(0.1, INTRO_S, v.introT));
  v.heroYaw = Math.PI + 0.6 - Math.PI * 2 * easeInOutCubic(remap(p, 0.04, 0.34));

  // hero text + HUD
  v.textIn = Math.max(remap(p, 0.04, 0.2), introOn * remap(v.introT, 0.9, INTRO_S));
  v.textOut = smoothstep(0.3, 0.4, p);
  v.hudIn = smoothstep(0.14, 0.24, p) * (1 - smoothstep(0.31, 0.37, p));

  // environment
  v.mazeIn = smoothstep(0.36, 0.5, p);
  v.scanBase = 0.14;
  v.scanIn = smoothstep(0.33, 0.4, p) * (1 - smoothstep(0.5, 0.6, p) * (1 - v.scanBase)) * (1 - smoothstep(0.93, 0.985, p));
  v.ghostU = ghostU(p);
  v.ghostShow = smoothstep(0.47, 0.5, p) * (1 - smoothstep(0.93, 0.985, p));

  // callouts
  v.callout = -1;
  for (let i = 0; i < 3; i++) {
    const m = windowMix(p, T.callouts[i][0], T.callouts[i][1]);
    v.calloutMix[i] = m;
    if (m > 0.5) v.callout = i;
  }

  // camera (skipped while the AutoDirector is flying it)
  if (v.directed) {
    v.followW = 0;
    return;
  }
  sampleKeys(p);
  v.followW = smoothstep(0.63, 0.66, p) * (1 - smoothstep(0.74, 0.77, p));
  if (v.followW > 0.001) {
    const g = routeAt(v.ghostU);
    const gw = enu(g.x, g.y, g.z);
    // behind-and-above the ghost, relative to its heading (three z = -north)
    _a.set(gw[0] - Math.cos(g.heading) * 3.3, gw[1] + 3.9, gw[2] + Math.sin(g.heading) * 3.3);
    _b.set(gw[0], gw[1] + 0.05, gw[2]);
    view.camPos.lerp(_a, clamp(v.followW * 0.6));
    view.camLook.lerp(_b, clamp(v.followW * 0.85));
  }
}
