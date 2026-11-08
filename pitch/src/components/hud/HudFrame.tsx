'use client';

import { AnimatePresence, motion, useMotionValueEvent } from 'framer-motion';
import { useEffect, useRef, useState } from 'react';
import { BRAND } from '@/config/assets';
import { SCENE_NAMES, jumpToScene, sceneOf, startDemo } from '@/lib/deckControls';
import { progress } from '@/lib/motion';
import { pad2 } from '@/lib/math';
import { useDeck } from '@/lib/store';

/**
 * The permanent HUD furniture, kept to the minimum: wordmark, scene name, and a thin progress rail with the scene buttons.
 * No FPS counter, no ticking clock, no crosshair or corner brackets: those are for the interactive part only (the live
 * clock appears in OVERRIDE, next to the telemetry). It re-themes through a CSS variable, with no React render.
 */
export default function HudFrame() {
  const [scene, setScene] = useState(0);
  const [past, setPast] = useState(false);
  const override = useDeck((s) => s.override);
  const auto = useDeck((s) => s.auto);
  const ready = useDeck((s) => s.ready);
  useMotionValueEvent(progress, 'change', (p) => {
    const s = sceneOf(p);
    setScene((prev) => (prev === s ? prev : s));
    const pa = p > 0.07;
    setPast((prev) => (prev === pa ? prev : pa));
  });

  return (
    <div className="pointer-events-none fixed inset-0 z-30 select-none">
      {/* top bar */}
      <div className="absolute inset-x-5 top-5 flex items-start justify-between sm:inset-x-8 sm:top-8">
        <div data-hud="top-left" className="font-display text-[15px] tracking-[-0.03em]">
          {BRAND.name}
        </div>

        <AnimatePresence mode="wait">
          <motion.div
            key={scene}
            data-hud="top-center"
            className="hud-label text-center"
            initial={{ opacity: 0, y: -4 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: 4 }}
            transition={{ duration: 0.4 }}
          >
            <span className="hud-accent">[ {String(scene + 1).padStart(2, '0')} ]</span> {SCENE_NAMES[scene]}
          </motion.div>
        </AnimatePresence>

        <div data-hud="top-right" className="hud-label text-right">
          {override ? <LiveClock /> : <span className="hidden sm:inline">{BRAND.drone}</span>}
        </div>
      </div>

      {/* bottom rail */}
      <div className="absolute inset-x-5 bottom-5 sm:inset-x-8 sm:bottom-8">
        <div className="relative">
          <div className="h-px w-full bg-white/15" />
          <motion.div className="absolute left-0 top-0 h-px w-full origin-left bg-[var(--accent)]" style={{ scaleX: progress }} />
          <div className="mt-2.5 flex items-center justify-between">
            <div data-hud="rail-left" className="pointer-events-auto flex items-center gap-1 sm:gap-2">
              {SCENE_NAMES.map((name, i) => (
                <button
                  key={name}
                  type="button"
                  onClick={() => jumpToScene(i)}
                  data-active={scene === i}
                  className="hud-label cursor-pointer px-1.5 py-1 transition-colors hover:text-ink data-[active=true]:text-ink"
                  aria-label={`Jump to scene ${i + 1}: ${name}`}
                >
                  <span className={scene === i ? 'hud-accent' : ''}>{pad2(i + 1)}</span>
                  <span className="hidden sm:inline"> {name}</span>
                </button>
              ))}
            </div>
            <div data-hud="rail-right" className="hud-label pointer-events-auto">
              {override ? (
                '[ ESC ] RELEASE'
              ) : auto ? (
                '[ ESC ] EXIT DEMO'
              ) : past && ready ? (
                <button type="button" onClick={startDemo} className="hud-label cursor-pointer transition-colors hover:text-ink">
                  [ D ] PLAY DEMO
                </button>
              ) : (
                'SCROLL ↓'
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

/** the live clock: mounted only in override, so there is nothing ticking anywhere else */
function LiveClock() {
  const clock = useRef<HTMLSpanElement>(null);
  useEffect(() => {
    let raf = 0;
    const t0 = performance.now();
    let last = 0;
    const tick = (now: number) => {
      if (now - last > 250 && clock.current) {
        last = now;
        const s = (now - t0) / 1000;
        clock.current.textContent = `${pad2(s / 60)}:${pad2(s % 60)}`;
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);
  return (
    <span>
      LIVE <span ref={clock} className="hud-value text-ink">00:00</span>
    </span>
  );
}
