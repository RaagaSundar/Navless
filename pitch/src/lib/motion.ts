/**
 * Shared motion values: the single source of truth that both the DOM (framer-motion `useTransform`) and the
 * WebGL scene (`useFrame` reads `.get()`) are driven from. Writing here never triggers a React render.
 */
import { motionValue } from 'framer-motion';

/** 0..1 across the whole scroll track (already smoothed by Lenis). Pinned to 1 while OVERRIDE is engaged. */
export const progress = motionValue(0);
/** 0..1 eased while OVERRIDE MODE engages / releases. */
export const overrideMix = motionValue(0);
/** |scroll velocity| in track-heights per second, softly clamped. Drives prop spin, chromatic aberration, HUD jitter. */
export const scrollVelocity = motionValue(0);
