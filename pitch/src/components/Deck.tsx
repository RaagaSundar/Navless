'use client';

import Lenis from 'lenis';
import { memo, useEffect, useState } from 'react';
import { INTRO_S, SCROLL_VH, T, camOverride, view } from '@/lib/choreography';
import { controls, enterOverride, exitOverride, jumpToScene, startDemo, stopDemo, toProgress, toggleDemoPause } from '@/lib/deckControls';
import { demo } from '@/lib/director';
import { clamp } from '@/lib/math';
import { progress, scrollVelocity } from '@/lib/motion';
import { EVIDENCE } from '@/lib/evidence';
import { replay as replayState, startReplay, stopReplay } from '@/lib/replay';
import { flight, useDeck } from '@/lib/store';
import { getMode, useTalk } from '@/lib/talk';
import SceneManager from './SceneManager';
import Talk from './talk/Talk';
import Callouts from './hud/Callouts';
import DemoControls from './hud/DemoControls';
import FxLayers from './hud/FxLayers';
import HeroOverlay from './hud/HeroOverlay';
import HudFrame from './hud/HudFrame';
import Loader from './hud/Loader';
import OverridePanel from './hud/OverridePanel';
import TelemetryHUD from './hud/TelemetryHUD';

const Scene = memo(SceneManager);

/** the dusk gradient used behind the slides when the 3D backdrop is switched off (B) */
function Dusk() {
  return <div className="absolute inset-0" style={{ background: 'linear-gradient(180deg, #0e0809 0%, #2a140e 58%, #8a4520 100%)' }} />;
}

/**
 * Deck: the page. Two modes, one scene:
 *   TALK (default)  the presenter deck: seven sections in the judges' order, one build per click (components/talk, data/script.ts).
 *                   L opens the interactive part (OVERRIDE MODE: drop a blocker on the planner); esc returns to the slide.
 *   TOUR (?mode=tour, or T)  the cinematic scroll deck:
 *     MANUAL  a tall invisible track gives the scroll length; Lenis smooths the wheel; at the end of the track the scroll
 *             LOCKS and OVERRIDE MODE takes over until you release it (esc, the button, or a hard scroll back up).
 *     AUTO    [ SEQUENCE INITIATED: PLAY DEMO ] hands the camera to the AutoDirector for a 2-minute run (lib/director.ts).
 * Everything you see is `position: fixed`: one WebGL canvas, the HUD, the scene type, the telemetry stream.
 */
export default function Deck() {
  const override = useDeck((s) => s.override);
  const ready = useDeck((s) => s.ready);
  const talk = getMode() === 'talk';
  const bg = useTalk((s) => s.bg);
  const [webgl, setWebgl] = useState<boolean | null>(null);

  useEffect(() => {
    try {
      const c = document.createElement('canvas');
      setWebgl(!!(c.getContext('webgl2') || c.getContext('webgl')));
    } catch {
      setWebgl(false);
    }
  }, []);

  /* ───────── smooth scroll + the lock ───────── */
  useEffect(() => {
    const talk = getMode() === 'talk';
    if ('scrollRestoration' in history) history.scrollRestoration = 'manual';
    window.scrollTo(0, 0);
    const reduce = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    const lenis = new Lenis({ lerp: reduce ? 1 : 0.085, wheelMultiplier: 0.85, touchMultiplier: 1.5, smoothWheel: !reduce });
    controls.lenis = lenis;
    lenis.stop(); // frozen until the boot screen clears (and for good in talk mode: slides, not scroll)
    lenis.on('scroll', (l: Lenis) => {
      // while the AutoDirector runs the clock, the scroll position is not the source of truth; in talk mode the slides are
      if (demo.active || talk) return;
      const p = toProgress(l.scroll, l.limit);
      if (!useDeck.getState().override) progress.set(p);
      scrollVelocity.set(clamp(Math.abs(l.velocity) / 55));
      if (p >= T.lock && l.direction === 1 && !useDeck.getState().override) enterOverride();
    });
    let raf = 0;
    const loop = (t: number) => {
      lenis.raf(t);
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);

    // deep links: /?p=0.5 opens straight into the timeline, /?demo=1 plays the sequence once loaded, /?q=low|medium|high
    const q = new URLSearchParams(window.location.search);
    const start = Number(q.get('p'));
    const startTimer = window.setTimeout(() => {
      if (!talk && Number.isFinite(start) && start > 0) lenis.scrollTo(Math.min(1, start) * lenis.limit, { immediate: true, force: true });
    }, 50);
    const qual = q.get('q');
    if (qual === 'low' || qual === 'medium' || qual === 'high') {
      useDeck.getState().setQuality(qual);
      useDeck.setState({ qualityLocked: true });
    }
    if (process.env.NODE_ENV !== 'production') {
      // dev-only QA hooks: window.__navless.go(0.5) | seek(42) | play()
      (window as unknown as { __navless: unknown }).__navless = {
        go: (v: number) => {
          lenis.scrollTo(Math.min(1, Math.max(0, v)) * lenis.limit, { immediate: true, force: true });
          progress.set(Math.min(1, Math.max(0, v)));
          view.snap = 3;
        },
        seek: (t: number) => {
          if (!useDeck.getState().auto) startDemo();
          demo.paused = true;
          useDeck.getState().setPaused(true);
          demo.seek = t;
        },
        play: () => {
          demo.paused = false;
          useDeck.getState().setPaused(false);
        },
        store: useDeck,
        talk: useTalk,
        enter: enterOverride,
        replay: { start: startReplay, stop: stopReplay },
        rs: replayState,
        dbg: () => ({ flight: { ...flight }, cam: { pos: camOverride.pos.toArray(), look: camOverride.look.toArray(), on: camOverride.on }, view: { camPos: view.camPos.toArray(), camLook: view.camLook.toArray(), fov: view.camFov, startW: view.startW, o: view.o }, replay: { ...replayState } }),
      };
    }
    return () => {
      window.clearTimeout(startTimer);
      cancelAnimationFrame(raf);
      lenis.destroy();
      controls.lenis = null;
    };
  }, []);

  useEffect(() => {
    if (!ready) return;
    const talk = getMode() === 'talk';
    // the hero builds itself now (opening reveal); reduced-motion visitors get the finished hero straight away
    view.introGo = true;
    if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) view.introT = INTRO_S;
    if (!talk && !useDeck.getState().override && !useDeck.getState().auto) controls.lenis?.start();
    // warm the planner while nobody is looking, so the first replan is instant
    const idle = (window as unknown as { requestIdleCallback?: (cb: () => void) => number }).requestIdleCallback;
    const t = idle ? idle(() => useDeck.getState().initPlan()) : window.setTimeout(() => useDeck.getState().initPlan(), 400);
    const autoplay = new URLSearchParams(window.location.search).get('demo') === '1';
    const at = autoplay && !talk ? window.setTimeout(startDemo, 700) : 0;
    return () => {
      if (!idle) window.clearTimeout(t as number);
      if (at) window.clearTimeout(at);
    };
  }, [ready]);

  /* ───────── attract loop: this laptop is for the eye, so it never sits still ─────────
     20 s without a touch on the hero starts the 2-minute sequence by itself. After it hands the grid to the judge, 25 s of nobody
     touching anything replays the four real flights from the logs one after another, and then the sequence starts again.
     A click or key stops the wait (esc aborts the sequence). ?attract=0 turns all of this off. */
  useEffect(() => {
    if (talk || !ready) return;
    if (new URLSearchParams(window.location.search).get('attract') === '0') return;
    const HERO_IDLE_MS = 20_000;
    const OVERRIDE_IDLE_MS = 25_000;
    const HOLD_AFTER_TOUCH_S = 6;
    let last = performance.now();
    let cycling = false; // we started the replays ourselves
    const touch = (e?: Event) => {
      last = performance.now();
      if (cycling && e && e.type !== 'pointermove') {
        cycling = false;
        stopReplay();
      }
    };
    const evs = ['pointerdown', 'pointermove', 'keydown', 'wheel', 'touchstart'] as const;
    evs.forEach((e) => window.addEventListener(e, touch, { passive: true }));
    const timer = window.setInterval(() => {
      const st = useDeck.getState();
      if (st.auto) {
        cycling = false;
        touch(); // the sequence is running: nothing is idle
        return;
      }
      if (st.override) {
        if (cycling) {
          const run = EVIDENCE.runs[replayState.run];
          if (!replayState.active || !run) {
            cycling = false;
          } else if (replayState.t > run.tContact + HOLD_AFTER_TOUCH_S) {
            const next = replayState.run + 1;
            if (next < EVIDENCE.runs.length) startReplay(next);
            else {
              cycling = false;
              touch();
              startDemo();
            }
          }
          return;
        }
        if (performance.now() - last > OVERRIDE_IDLE_MS && !replayState.active) {
          cycling = true;
          startReplay(0);
        }
        return;
      }
      if (performance.now() - last > HERO_IDLE_MS) {
        touch();
        startDemo();
      }
    }, 1000);
    return () => {
      window.clearInterval(timer);
      evs.forEach((e) => window.removeEventListener(e, touch));
    };
  }, [ready, talk]);

  /* ───────── keyboard + hard scroll-up release ───────── */
  useEffect(() => {
    const talk = getMode() === 'talk';
    const onKey = (e: KeyboardEvent) => {
      if (e.metaKey || e.ctrlKey || e.altKey) return;
      const st = useDeck.getState();
      const k = e.key.toLowerCase();
      if (k === 'escape') {
        if (st.auto) stopDemo();
        else if (st.armed) st.setArmed(false);
        else if (st.override) exitOverride();
        return;
      }
      if (st.auto) {
        if (k === ' ') {
          e.preventDefault();
          toggleDemoPause();
        }
        return;
      }
      if (!st.override) {
        if (talk) return; // components/talk/Talk.tsx owns the keys until the live demo opens
        if (k === '1' || k === '2' || k === '3') jumpToScene(Number(k) - 1);
        else if (k === 'd' && st.ready) startDemo();
        return;
      }
      if (k === 'z') st.undo();
      else if (k === 'c') st.clear();
      else if (k === 'i') st.setArmed(!st.armed);
      else if (k === 'p') st.dropOnPath();
      else if (k === 'r') st.resetFlight();
      else if (k === 'enter' || k === ' ') {
        e.preventDefault();
        if (st.flightMode === 'idle' || st.flightMode === 'landed') st.engage();
      }
    };
    let acc = 0;
    let t0 = 0;
    const onWheel = (e: WheelEvent) => {
      const st = useDeck.getState();
      if (!st.override || st.auto || (e.target as HTMLElement | null)?.closest?.('.term-scroll')) return;
      const now = performance.now();
      if (now - t0 > 700) acc = 0;
      t0 = now;
      acc += e.deltaY;
      if (acc < -520) {
        acc = 0;
        exitOverride();
      }
    };
    window.addEventListener('keydown', onKey);
    window.addEventListener('wheel', onWheel, { passive: true });
    return () => {
      window.removeEventListener('keydown', onKey);
      window.removeEventListener('wheel', onWheel);
    };
  }, []);

  return (
    <main data-mode={override ? 'override' : 'tour'} className="relative bg-void text-ink">
      {webgl === false ? (
        <div className="fixed inset-0 z-50 flex items-center justify-center p-8 text-center">
          <div>
            <div className="font-display text-5xl">NAVLESS</div>
            <p className="hud-label mt-6">ZERO GPS. PURE AUTONOMY.</p>
            <p className="hud-value mx-auto mt-6 max-w-md text-[12px] leading-relaxed text-ink/70">
              WebGL is off or unavailable in this browser. Use current Chrome, Edge or Safari with hardware acceleration on.
            </p>
          </div>
        </div>
      ) : (
        <>
          <div className="fixed inset-0 z-0 bg-void">{webgl && (!talk || bg || override) ? <Scene /> : <Dusk />}</div>
          <FxLayers />
          {talk ? (
            // the slides; while the live demo is open the usual HUD (live clock, esc to release) takes their place
            override ? <HudFrame /> : <Talk />
          ) : (
            <>
              <HudFrame />
              <HeroOverlay />
              <Callouts />
            </>
          )}
          <TelemetryHUD />
          <OverridePanel />
          {talk ? null : <DemoControls />}
          <Loader />
        </>
      )}
      {/* the scroll track (tour only: talk mode has no scrolling) */}
      {talk ? null : <div aria-hidden style={{ height: `${SCROLL_VH}vh` }} />}
    </main>
  );
}
