'use client';

import { AnimatePresence, motion } from 'framer-motion';
import { useEffect, useRef } from 'react';
import { exitOverride } from '@/lib/deckControls';
import { flight, useDeck } from '@/lib/store';

/**
 * Scene 3 chrome: the OVERRIDE banner, the planner stats, and the controls. The massive pulsing
 * [ INJECT DYNAMIC OBSTACLE ] button arms the grid; every click on the grid then drops a blocker.
 */

const STATUS_LABEL = { nominal: 'NOMINAL', replanned: 'REPLANNED', sealed: 'ROUTE SEALED' } as const;
const FLIGHT_LABEL = { idle: 'HOVER · ON MARK', flying: 'TRACKING', hold: 'HOLD · NO ROUTE', landed: 'LANDED' } as const;

type Tone = 'ok' | 'warn' | 'err';
const toneClass = (tone?: Tone) => (tone === 'err' ? 'text-danger' : tone === 'warn' ? 'text-amber' : tone === 'ok' ? 'text-ok' : 'text-ink');

/** one readout in the two-column planner grid: label over value (a long column of rows does not fit a 720 px screen) */
function Cell({ k, v, tone, valueRef }: { k: string; v: string; tone?: Tone; valueRef?: React.Ref<HTMLSpanElement> }) {
  return (
    <div className="min-w-0">
      <div className="hud-label text-[9px] leading-none">{k}</div>
      <div className={`hud-value mt-[3px] whitespace-nowrap text-[12.5px] leading-tight ${toneClass(tone)}`}>
        <span ref={valueRef}>{v}</span>
      </div>
    </div>
  );
}

function FlightLine() {
  const alt = useRef<HTMLSpanElement>(null);
  const spd = useRef<HTMLSpanElement>(null);
  const cp = useRef<HTMLSpanElement>(null);
  useEffect(() => {
    let raf = 0;
    let last = 0;
    const tick = (now: number) => {
      if (now - last > 90) {
        last = now;
        if (alt.current) alt.current.textContent = `${flight.z.toFixed(2)} m`;
        if (spd.current) spd.current.textContent = `${flight.speed.toFixed(2)} m/s`;
        if (cp.current) cp.current.textContent = `${flight.checkpoints} / 5`;
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);
  return (
    <>
      <Cell k="ALT" v="0.45 m" valueRef={alt} />
      <Cell k="SPEED" v="0.00 m/s" valueRef={spd} />
      <Cell k="CHECKPOINTS" v="0 / 5" valueRef={cp} />
    </>
  );
}

function Ctl({ children, onClick, disabled, hot }: { children: React.ReactNode; onClick: () => void; disabled?: boolean; hot?: boolean }) {
  return (
    <button
      type="button"
      onClick={onClick}
      disabled={disabled}
      className={`pointer-events-auto cursor-pointer border px-3 py-2 font-mono text-[10.5px] uppercase tracking-[0.18em] backdrop-blur-sm transition-colors duration-100 disabled:cursor-not-allowed disabled:opacity-30 ${
        hot ? 'border-[var(--accent)] bg-[var(--accent)] text-black hover:bg-ink' : 'border-white/25 bg-black/55 text-ink/85 hover:border-[var(--accent)] hover:bg-[var(--accent)] hover:text-black'
      }`}
    >
      {children}
    </button>
  );
}

export default function OverridePanel() {
  const override = useDeck((s) => s.override);
  const armed = useDeck((s) => s.armed);
  const setArmed = useDeck((s) => s.setArmed);
  const plan = useDeck((s) => s.plan);
  const status = useDeck((s) => s.planStatus);
  const nObs = useDeck((s) => s.obstacles.length);
  const flightMode = useDeck((s) => s.flightMode);
  const dropOnPath = useDeck((s) => s.dropOnPath);
  const undo = useDeck((s) => s.undo);
  const clear = useDeck((s) => s.clear);
  const engage = useDeck((s) => s.engage);
  const resetFlight = useDeck((s) => s.resetFlight);
  const auto = useDeck((s) => s.auto);

  const base = useRef<number | null>(null);
  if (status === 'nominal' && plan?.ok) base.current = plan.length;
  const delta = plan?.ok && base.current ? plan.length - base.current : 0;
  const sealed = status === 'sealed';

  const hint = auto
    ? 'auto demo: two scripted drops, then it is yours'
    : sealed
      ? 'route sealed: the corridor is narrower than the keep-out · [z] undo'
      : armed
        ? 'click the grid to drop · esc to disarm · hover to preview the re-plan'
        : nObs === 0
          ? 'arm it, then click anywhere on the grid. the blue line is the live plan.'
          : 'keep dropping. try the window or the tunnel.';

  return (
    <AnimatePresence>
      {override && (
        <motion.div key="override" className="pointer-events-none fixed inset-0 z-30" initial={{ opacity: 0 }} animate={{ opacity: 1 }} exit={{ opacity: 0, transition: { duration: 0.4 } }} transition={{ duration: 0.6, delay: 0.5 }}>
          {/* banner */}
          <motion.div data-hud="ov-banner" className="absolute left-1/2 top-[4.6rem] -translate-x-1/2 text-center lg:left-[calc((100%-30.5rem)/2)]" initial={{ y: -16, opacity: 0 }} animate={{ y: 0, opacity: 1 }} transition={{ delay: 0.7, duration: 0.5 }}>
            <div className="font-display text-[clamp(1.7rem,3.6vw,3.4rem)] text-ember">OVERRIDE MODE</div>
            <div className="hud-label mt-2">YOUR TURN · DROP A BLOCKER · WATCH IT RE-PLAN · SIMULATION</div>
          </motion.div>

          {/* stats */}
          <motion.div data-hud="ov-stats" className="absolute left-5 top-[9.2rem] hidden w-[17.5rem] sm:left-8 xl:block" initial={{ x: -24, opacity: 0 }} animate={{ x: 0, opacity: 1 }} transition={{ delay: 0.9, duration: 0.5 }}>
            <div className="hud-label mb-2.5 flex items-center gap-2">
              <i className="inline-block size-1.5 bg-[var(--accent)]" />
              PLANNER · LIVE
            </div>
            <div className="grid grid-cols-2 gap-x-4 gap-y-2.5 border-l border-white/20 pl-3">
              <Cell k="STATE" v={STATUS_LABEL[status]} tone={sealed ? 'err' : status === 'replanned' ? 'warn' : 'ok'} />
              <Cell k="OBSTACLES" v={String(nObs)} />
              <Cell k="PATH" v={plan?.ok ? `${plan.length.toFixed(2)} m` : '—'} />
              <Cell k="Δ vs NOMINAL" v={plan?.ok ? `${delta >= 0 ? '+' : ''}${delta.toFixed(2)} m` : '—'} tone={delta > 0.05 ? 'warn' : undefined} />
              <Cell k="MIN CLEARANCE" v={plan?.ok ? `${plan.minClearance.toFixed(2)} m` : '—'} />
              <Cell k="T @ 0.5 m/s" v={plan?.ok ? `${plan.duration.toFixed(1)} s` : '—'} />
              <Cell k="A* EXPANDED" v={plan?.ok ? plan.expanded.toLocaleString('en-US') : '—'} />
              <Cell k="REPLAN" v={plan ? `${plan.timings.total.toFixed(1)} ms` : '—'} tone="ok" />
              <Cell k="ROUTE" v={plan?.usedFallback ? 'GATE B BACKUP' : 'WINDOW'} tone={plan?.usedFallback ? 'warn' : undefined} />
              <Cell k="DRONE" v={FLIGHT_LABEL[flightMode]} tone={flightMode === 'hold' ? 'err' : flightMode === 'landed' ? 'ok' : undefined} />
              <FlightLine />
            </div>
          </motion.div>

          {/* controls */}
          <motion.div data-hud="ov-controls" className="absolute inset-x-0 bottom-[calc(5.3rem+26vh+0.75rem)] flex flex-col items-center gap-3 px-4 lg:bottom-[5.4rem] lg:right-[30.5rem]" initial={{ y: 30, opacity: 0 }} animate={{ y: 0, opacity: 1 }} transition={{ delay: 0.8, duration: 0.55 }}>
            <AnimatePresence mode="wait">
              <motion.div key={hint} className={`hud-label max-w-[44rem] text-center ${sealed ? 'text-danger' : ''}`} initial={{ opacity: 0, y: 6 }} animate={{ opacity: 1, y: 0 }} exit={{ opacity: 0, y: -6 }} transition={{ duration: 0.2 }}>
                {hint}
              </motion.div>
            </AnimatePresence>

            <motion.button
              type="button"
              onClick={() => setArmed(!armed)}
              disabled={auto}
              data-armed={armed}
              className="inject-btn pointer-events-auto font-mono text-[clamp(1rem,2.35vw,2.3rem)] font-bold uppercase tracking-[0.06em] px-[clamp(1.2rem,3vw,3rem)] py-[clamp(0.9rem,1.6vw,1.5rem)]"
              whileTap={{ scale: 0.98 }}
              aria-pressed={armed}
            >
              [ {armed ? 'ARMED · CLICK THE GRID' : 'INJECT DYNAMIC OBSTACLE'} ]
            </motion.button>

            <div className="flex flex-wrap items-center justify-center gap-2">
              <Ctl onClick={dropOnPath} disabled={auto}>[ Drop on path ]</Ctl>
              {flightMode === 'idle' || flightMode === 'landed' ? (
                <Ctl onClick={engage} disabled={sealed || auto} hot>
                  [ {flightMode === 'landed' ? 'Fly again' : 'Engage flight'} ]
                </Ctl>
              ) : (
                <Ctl onClick={resetFlight} disabled={auto}>[ Abort · reset ]</Ctl>
              )}
              <Ctl onClick={undo} disabled={nObs === 0 || auto}>[ Undo · Z ]</Ctl>
              <Ctl onClick={clear} disabled={nObs === 0 || auto}>[ Clear · C ]</Ctl>
              <Ctl onClick={exitOverride}>[ Esc · Exit ]</Ctl>
            </div>
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>
  );
}
