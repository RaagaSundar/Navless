/**
 * palette.ts: FUTURE MARS, GOLDEN HOUR.
 *
 * One source of truth for every colour the WebGL side uses (the CSS tokens in app/globals.css mirror these: keep the two in step).
 *
 * The idea: a low amber sun over rust-red regolith. Warm light, cool shadows.
 *   WARM   = the world and the route: sand walls, gold + ember lines, rust ground, a dusk-glow sky
 *   COOL   = what the drone perceives and decides: its stereo view, tracked features, and the live plan (dusty ice-blue)
 *   DANGER = obstacles only (vermilion), so a blocker is the one thing on screen that reads as a warning
 */
export const PAL = {
  // ground and sky
  void: '#0d0807', // page / deepest shadow, warm near-black
  umber: '#1b0f0b', // panels
  regolith: '#4a2a1d', // dim structure: grid lines, ground
  duskTop: '#0e0809', // sky, top of frame
  duskMid: '#2a140e',
  duskGlow: '#8a4520', // horizon glow, bottom of frame
  // warm family
  dust: '#c9805f', // dusty terracotta
  rust: '#e0592a', // Mars red-orange: the brand colour
  ember: '#ff7d45', // bright orange: emphasis, the override accent
  gold: '#f6b25e', // golden-hour sun: the default accent
  sand: '#f1dcc0', // walls, wireframes, warm light neutrals
  ink: '#f6e9d9', // text
  // cool family
  ice: '#9cc6dd', // the live plan, the camera's view
  mist: '#e8f3f9', // hot core of the plan line
  // signals
  danger: '#ff5a3c',
  ok: '#9ccfb0',
  amber: '#f2b24a',
} as const;

/** HDR multiplier for things that should glow (bloom threshold is high now, so only these catch it) */
export const GLOW = 1.8;
