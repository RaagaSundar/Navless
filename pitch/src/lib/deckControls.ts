/**
 * deckControls.ts: imperative controls shared by the DOM (buttons, keyboard) and the scroll system.
 * Holds the Lenis instance, the scroll lock that engages OVERRIDE MODE, scene jumps, and the AutoDirector start / stop.
 */
import { animate } from 'framer-motion';
import type Lenis from 'lenis';
import { T, view } from './choreography';
import { demo, demoTime, resetDemoFlags } from './director';
import { clamp, easeInOutCubic } from './math';
import { overrideMix, progress } from './motion';
import { clearLogQueue, flight, log, useDeck } from './store';

export const controls: { lenis: Lenis | null } = { lenis: null };

/** scene index (0, 1, 2) for a scroll position */
export const sceneOf = (p: number) => (p < T.toMap[0] + 0.02 ? 0 : p < T.toOverride[0] ? 1 : 2);

export const SCENE_STARTS = [0.0, 0.46, 0.99] as const;
export const SCENE_NAMES = ['HERO REVEAL', 'THE ENVIRONMENT', 'OVERRIDE MODE'] as const;

export function enterOverride() {
  const { lenis } = controls;
  const st = useDeck.getState();
  if (st.override) return;
  // flip the flag FIRST: lenis.scrollTo() emits a scroll event synchronously, and that handler checks this flag
  st.setOverride(true);
  st.initPlan();
  if (lenis) {
    lenis.scrollTo(lenis.limit, { immediate: true, force: true });
    lenis.stop();
  }
  progress.set(1);
  animate(overrideMix, 1, { duration: 1.3, ease: [0.22, 1, 0.36, 1] });
}

export function exitOverride() {
  const { lenis } = controls;
  const st = useDeck.getState();
  if (!st.override) return;
  st.setArmed(false);
  st.setOverride(false);
  animate(overrideMix, 0, { duration: 0.9, ease: [0.22, 1, 0.36, 1] });
  if (lenis) {
    lenis.start();
    lenis.scrollTo(lenis.limit * 0.88, { duration: 1.8, easing: easeInOutCubic, force: true });
  }
}

/** jump to a scene (used by the HUD rail and 1 / 2 / 3 keys) */
export function jumpToScene(i: number) {
  const { lenis } = controls;
  if (!lenis || useDeck.getState().auto) return;
  if (i >= 2) {
    // glide to the end; the scroll lock takes over from there
    lenis.scrollTo(lenis.limit, { duration: 2.4, easing: easeInOutCubic, force: true });
    return;
  }
  if (useDeck.getState().override) exitOverride();
  lenis.start();
  lenis.scrollTo(SCENE_STARTS[i] * lenis.limit, { duration: 1.8, easing: easeInOutCubic, force: true });
}

export const toProgress = (scroll: number, limit: number) => clamp(limit > 0 ? scroll / limit : 0);

/* ───────────────────────────── the 2-minute sequence ───────────────────────────── */

export function startDemo() {
  const st = useDeck.getState();
  if (!st.ready || st.auto) return;
  // clean slate: leave override, drop blockers, park the drone
  if (st.override) {
    st.setOverride(false);
    overrideMix.set(0);
  }
  st.setArmed(false);
  if (st.obstacles.length) st.clear();
  if (flight.mode !== 'idle') st.resetFlight();
  clearLogQueue();
  st.clearLines();

  const { lenis } = controls;
  lenis?.scrollTo(0, { immediate: true, force: true });
  lenis?.stop();
  progress.set(0);
  resetDemoFlags();
  demo.t = 0;
  demo.seek = null;
  demo.paused = false;
  demo.active = true;
  demoTime.set(0);
  view.snap = 3;
  st.setAuto(true);
  log('SYS', 'SEQUENCE INITIATED: AutoDirector has the camera. esc = abort · space = pause', 'warn');
}

export function stopDemo() {
  const st = useDeck.getState();
  if (!st.auto) return;
  demo.active = false;
  demo.paused = false;
  st.setAuto(false);
  view.directed = false;
  view.revealOverride = null;
  const { lenis } = controls;
  if (!st.override && lenis) {
    lenis.scrollTo(progress.get() * lenis.limit, { immediate: true, force: true });
    lenis.start();
  }
  log('SYS', 'sequence released: manual control', 'dim');
}

export function toggleDemoPause() {
  const st = useDeck.getState();
  if (!st.auto) return;
  demo.paused = !demo.paused;
  st.setPaused(demo.paused);
}
