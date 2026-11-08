'use client';

import { AnimatePresence, motion } from 'framer-motion';
import { memo, useEffect, useLayoutEffect, useRef } from 'react';
import { progress } from '@/lib/motion';
import { createTelemetry, ctxOf, stepTelemetry } from '@/lib/telemetry';
import { drainLogs, enqueueLogs, flight, queuedLogs, useDeck, type TermLine } from '@/lib/store';

/**
 * TelemetryHUD: the live terminal. It exists ONLY in OVERRIDE MODE, the interactive part of the deck: nothing streams across
 * the hero or the tour, so those stay clean. When the judge takes over, the panel slides in on the right and the REAL
 * planner's output lands in it (A* expansion counts, the EGO iterations, clearances), mixed with the simulated on-board stack
 * lines (cuVSLAM, odom_gate, PX4 height offset, blind carry).
 *
 * Lines arrive one at a time, quickly enough to read as a live stream and slowly enough to actually read: a 5 ms re-plan that
 * logs 40 lines plays out over about a second and a half, not as a blur.
 *
 * Simulated lines (lib/telemetry.ts) are marked SIM in the header. Planner lines (lib/planner.ts) are measured.
 */

interface Gauge {
  k: string;
  /** a label that follows state (the fourth gauge reads OBST on the ground and ALT in flight) */
  label?: () => string;
  unit?: string;
  get: (t: number) => string;
}
const G_PLAN: Gauge[] = [
  { k: 'PATH', unit: 'm', get: () => (useDeck.getState().plan?.ok ? useDeck.getState().plan!.length.toFixed(2) : '—') },
  { k: 'MIN CLR', unit: 'm', get: () => (useDeck.getState().plan?.ok ? useDeck.getState().plan!.minClearance.toFixed(2) : '—') },
  { k: 'REPLAN', unit: 'ms', get: () => (useDeck.getState().plan ? useDeck.getState().plan!.timings.total.toFixed(1) : '—') },
  {
    k: 'OBST',
    label: () => (flight.mode === 'flying' ? 'ALT' : 'OBST'),
    get: () => (flight.mode === 'flying' ? `${flight.z.toFixed(2)} m` : String(useDeck.getState().obstacles.length)),
  },
];

function Gauges() {
  const els = useRef<(HTMLSpanElement | null)[]>([]);
  const labels = useRef<(HTMLDivElement | null)[]>([]);
  useEffect(() => {
    let raf = 0;
    let last = 0;
    const tick = (now: number) => {
      if (now - last > 200) {
        last = now;
        for (let i = 0; i < G_PLAN.length; i++) {
          const el = els.current[i];
          if (el) el.textContent = G_PLAN[i].get(now / 1000);
          const lb = labels.current[i];
          const want = G_PLAN[i].label?.();
          if (lb && want && lb.textContent !== want) lb.textContent = want;
        }
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);
  return (
    <div className="grid grid-cols-4 gap-px border-b border-white/12 bg-white/10">
      {G_PLAN.map((g, i) => (
        <div key={g.k} className="bg-void/80 px-2 py-1.5">
          <div
            ref={(el) => {
              labels.current[i] = el;
            }}
            className="hud-label text-[8px] tracking-[0.16em]"
          >
            {g.k}
          </div>
          <div className="hud-value text-[12px] text-ink">
            <span
              ref={(el) => {
                els.current[i] = el;
              }}
            >
              —
            </span>
            {g.unit ? <span className="ml-0.5 text-[9px] text-white/45">{g.unit}</span> : null}
          </div>
        </div>
      ))}
    </div>
  );
}

const Row = memo(function Row({ l }: { l: TermLine }) {
  return (
    <div className="term-row" data-lvl={l.level}>
      <span className="term-stamp">{l.stamp}</span>
      <span className="term-tag" data-tag={l.tag}>
        {l.tag}
      </span>
      <span className="term-text">{l.text}</span>
    </div>
  );
});

// The side panel from lg up; below that a 26vh strip along the bottom, with the override controls stacked directly above it
// (OverridePanel.tsx uses the same 5.3rem + 26vh).
const PANEL =
  'pointer-events-auto fixed inset-x-3 bottom-[4.7rem] z-30 flex h-[26vh] flex-col border border-[var(--accent)]/45 bg-void/75 backdrop-blur-md sm:inset-x-6 sm:bottom-[5.3rem] lg:inset-x-auto lg:bottom-[5.4rem] lg:right-8 lg:top-[4.6rem] lg:h-auto lg:w-[27rem]';

function Panel() {
  const lines = useDeck((s) => s.lines);
  const body = useRef<HTMLDivElement>(null);
  const pinned = useRef(true);
  const q = useRef<HTMLSpanElement>(null);

  useLayoutEffect(() => {
    const el = body.current;
    if (el && pinned.current) el.scrollTop = el.scrollHeight;
  }, [lines]);

  useEffect(() => {
    let raf = 0;
    let last = 0;
    const tick = (now: number) => {
      if (now - last > 200 && q.current) {
        last = now;
        q.current.textContent = String(queuedLogs()).padStart(2, '0');
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);

  return (
    <motion.aside
      className={PANEL}
      data-hud="telemetry"
      initial={{ x: 40, opacity: 0 }}
      animate={{ x: 0, opacity: 1 }}
      exit={{ opacity: 0, transition: { duration: 0.22 } }}
      transition={{ duration: 0.6, delay: 0.6, ease: [0.22, 1, 0.36, 1] }}
    >
      <div className="brackets pointer-events-none absolute inset-0 [--bl:12px]" aria-hidden />
      <header className="flex items-center justify-between border-b border-white/12 px-3 py-1.5">
        <span className="hud-label text-ink">
          [ TELEMETRY <span className="hud-accent">//</span> skybeta-19 <span className="text-white/45">· SIM</span> ]
        </span>
        <span className="hud-label">
          queue <span ref={q}>00</span>
        </span>
      </header>
      <Gauges />
      <div
        ref={body}
        onScroll={(e) => {
          const el = e.currentTarget;
          pinned.current = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
        }}
        className="term-scroll min-h-0 flex-1 overflow-y-auto px-3 py-1.5"
        role="log"
        aria-live="off"
      >
        {lines.map((l) => (
          <Row key={l.id} l={l} />
        ))}
      </div>
      <footer className="flex items-center gap-2 border-t border-white/12 px-3 py-1.5">
        <span className="hud-label whitespace-nowrap normal-case tracking-[0.04em] text-ink/60">navless@skybeta-19:~$</span>
        <span className="caret" />
      </footer>
    </motion.aside>
  );
}

export default function TelemetryHUD() {
  const override = useDeck((s) => s.override);
  const appendLines = useDeck((s) => s.appendLines);

  // the generator and the pump run only while the panel is on screen
  useEffect(() => {
    if (!override) return;
    const gen = createTelemetry();
    let raf = 0;
    let lastLine = 0;
    const tick = (now: number) => {
      const st = useDeck.getState();
      if (st.ready) {
        const fresh = stepTelemetry(gen, now, ctxOf(progress.get(), true));
        if (fresh.length) enqueueLogs(fresh);
      }
      const n = queuedLogs();
      // one line at a time: faster when there is a backlog (a re-plan), never faster than ~28 lines a second
      const gap = n > 30 ? 36 : n > 10 ? 60 : 120;
      if (n && now - lastLine >= gap) {
        lastLine = now;
        appendLines(drainLogs(1));
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [override, appendLines]);

  return <AnimatePresence>{override ? <Panel key="panel" /> : null}</AnimatePresence>;
}
