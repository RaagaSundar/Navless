export const TAU = Math.PI * 2;

export const clamp = (v: number, lo = 0, hi = 1) => (v < lo ? lo : v > hi ? hi : v);
export const lerp = (a: number, b: number, t: number) => a + (b - a) * t;
export const invLerp = (a: number, b: number, v: number) => (b === a ? 0 : (v - a) / (b - a));
/** 0..1 progress of `v` through [a, b] (clamped). */
export const remap = (v: number, a: number, b: number) => clamp(invLerp(a, b, v));
export const smoothstep = (a: number, b: number, v: number) => {
  const t = remap(v, a, b);
  return t * t * (3 - 2 * t);
};
export const smootherstep = (a: number, b: number, v: number) => {
  const t = remap(v, a, b);
  return t * t * t * (t * (t * 6 - 15) + 10);
};

export const easeInCubic = (t: number) => t * t * t;
export const easeOutCubic = (t: number) => 1 - Math.pow(1 - t, 3);
export const easeInOutCubic = (t: number) => (t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2);
export const easeOutExpo = (t: number) => (t >= 1 ? 1 : 1 - Math.pow(2, -10 * t));
export const easeOutBack = (t: number) => {
  const c1 = 1.70158;
  const c3 = c1 + 1;
  return 1 + c3 * Math.pow(t - 1, 3) + c1 * Math.pow(t - 1, 2);
};

/** Frame-rate independent exponential smoothing (a.k.a. damp). `lambda` ~ 3 (lazy) .. 20 (snappy). */
export const damp = (current: number, target: number, lambda: number, dt: number) =>
  lerp(current, target, 1 - Math.exp(-lambda * dt));

export const wrapPi = (a: number) => {
  a = (a + Math.PI) % TAU;
  if (a < 0) a += TAU;
  return a - Math.PI;
};
/** Damp an angle along the shortest arc. */
export const dampAngle = (current: number, target: number, lambda: number, dt: number) =>
  current + wrapPi(target - current) * (1 - Math.exp(-lambda * dt));

/** Small fast seeded PRNG: deterministic point clouds across reloads. */
export function mulberry32(seed: number) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export const fmt = (v: number, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : '—');
export const pad2 = (n: number) => String(Math.floor(n)).padStart(2, '0');
