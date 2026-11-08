'use client';

import { AnimatePresence, motion, useMotionValueEvent, useTransform } from 'framer-motion';
import { useRef } from 'react';
import { startDemo, stopDemo, toggleDemoPause } from '@/lib/deckControls';
import { CAPTIONS, DEMO_DURATION, demoTime, type Caption } from '@/lib/director';
import { pad2, smoothstep } from '@/lib/math';
import { progress } from '@/lib/motion';
import { useDeck } from '@/lib/store';

/**
 * The two ways through the deck: scroll it yourself, or press [ SEQUENCE INITIATED: PLAY DEMO ] and let the AutoDirector
 * fly the 2-minute path. While it plays this shows a thin progress bar with pause / exit, and the captions pinned to the
 * opening shots (the scene type syncs through the scroll progress the director writes). Once you have scrolled away from
 * the hero, "play demo" lives in the bottom rail (HudFrame) instead of a second button here.
 */

const clockText = (t: number) => `${Math.floor(t / 60)}:${pad2(t % 60)}`;

// While it plays the run bar sits under the top bar; in OVERRIDE it steps down below the banner and centres on the free area
// left of the telemetry panel (100% - 30.5rem, the same reserve the override controls use), nudged right of the planner
// stats from xl up. So the banner, the run bar and the stats never share a row.
const RUN_TOP = 'left-1/2 top-[3.2rem] w-[min(22rem,88vw)] sm:top-[4rem]';
const RUN_OVERRIDE = 'left-1/2 top-[11.8rem] w-[min(22rem,88vw)] sm:top-[9.2rem] lg:left-[calc((100%-30.5rem)/2)] xl:left-[calc((100%-30.5rem)/2+9rem)]';

function CaptionBlock({ c }: { c: Caption }) {
  const o = useTransform(demoTime, (t) => smoothstep(c.a, c.a + 0.5, t) * (1 - smoothstep(c.b - 0.5, c.b, t)));
  const x = useTransform(o, [0, 1], [-18, 0]);
  return (
    <motion.div data-hud="caption" className="absolute left-5 top-[24vh] w-[min(30rem,86vw)] sm:left-[5.5vw]" style={{ opacity: o, x }}>
      <div className="mb-2 flex items-center gap-3">
        <span className="hud-label hud-accent">[ {c.tag} ]</span>
        <i className="block h-px w-16 bg-[var(--accent)]" />
      </div>
      {c.lines.map((l) => (
        <div key={l} className="hud-value text-[clamp(0.85rem,1.2vw,1.05rem)] leading-[1.65] text-ink">
          {l}
        </div>
      ))}
    </motion.div>
  );
}

export default function DemoControls() {
  const auto = useDeck((s) => s.auto);
  const paused = useDeck((s) => s.paused);
  const ready = useDeck((s) => s.ready);
  const override = useDeck((s) => s.override);

  const clock = useRef<HTMLSpanElement>(null);
  useMotionValueEvent(demoTime, 'change', (t) => {
    if (clock.current) clock.current.textContent = clockText(t);
  });
  const fill = useTransform(demoTime, (t) => t / DEMO_DURATION);

  // the big call-to-action at the very start; it fades away as soon as you scroll
  const big = useTransform(progress, (p) => 1 - smoothstep(0.02, 0.07, p));

  const canPlay = ready && !auto;

  return (
    <div className="pointer-events-none fixed inset-0 z-30">
      {/* hero CTA */}
      <AnimatePresence>
        {canPlay && !override && (
          // The scroll-driven `opacity: big` owns this element's opacity. The mount fade lives one level down: putting
          // `initial={{ opacity: 0 }}` here too fights the style motion value and leaves the button invisible on first paint.
          <motion.div key="cta" data-hud="cta" className="absolute bottom-[6.6rem] left-1/2 -translate-x-1/2 text-center sm:bottom-[7.4rem]" style={{ opacity: big }} initial={{ y: 16 }} animate={{ y: 0 }} exit={{ opacity: 0, transition: { duration: 0.25 } }}>
            <motion.div initial={{ opacity: 0 }} animate={{ opacity: 1 }} transition={{ duration: 0.7, delay: 0.25 }}>
              <button
                type="button"
                onClick={startDemo}
                className="inject-btn pointer-events-auto whitespace-nowrap px-[clamp(0.9rem,2.2vw,2.2rem)] py-[clamp(0.65rem,1.1vw,1rem)] font-mono text-[clamp(0.7rem,calc(0.95vw+0.1rem),1rem)] font-bold uppercase tracking-[0.1em]"
              >
                [ SEQUENCE INITIATED: PLAY DEMO ]
              </button>
              <div className="hud-label mt-4 tracking-[0.14em]">or just scroll ↓</div>
            </motion.div>
          </motion.div>
        )}
      </AnimatePresence>

      {/* while it plays */}
      <AnimatePresence>
        {auto && (
          <motion.div key="run" data-hud="runbar" className={`absolute -translate-x-1/2 transition-[top,left,width] duration-700 ease-out ${override ? RUN_OVERRIDE : RUN_TOP}`} initial={{ opacity: 0, y: -8 }} animate={{ opacity: 1, y: 0 }} exit={{ opacity: 0, transition: { duration: 0.3 } }}>
            <div className="mb-1.5 flex items-center justify-between gap-3">
              <span className="hud-label text-ink">
                {paused ? 'PAUSED' : 'DEMO'} <span ref={clock}>0:00</span> / {clockText(DEMO_DURATION)}
              </span>
              <span className="pointer-events-auto flex gap-1.5">
                <button type="button" onClick={toggleDemoPause} className="cursor-pointer border border-white/25 bg-void/60 px-2 py-1 font-mono text-[9px] uppercase tracking-[0.18em] text-ink/80 hover:border-[var(--accent)] hover:text-[var(--accent)]">
                  {paused ? 'resume' : 'pause'}
                </button>
                <button type="button" onClick={stopDemo} className="cursor-pointer border border-white/25 bg-void/60 px-2 py-1 font-mono text-[9px] uppercase tracking-[0.18em] text-ink/80 hover:border-ember hover:text-ember">
                  exit
                </button>
              </span>
            </div>
            <div className="h-px w-full bg-white/15">
              <motion.div className="h-px origin-left bg-[var(--accent)]" style={{ scaleX: fill }} />
            </div>
          </motion.div>
        )}
      </AnimatePresence>

      {/* HTML captions pinned to the opening shots */}
      {auto && CAPTIONS.map((c) => <CaptionBlock key={c.tag + c.a} c={c} />)}
    </div>
  );
}
