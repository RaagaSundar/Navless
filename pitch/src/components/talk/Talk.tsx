'use client';

import { useEffect, useLayoutEffect, useState } from 'react';
import { SLIDES, totalSeconds } from '@/data/script';
import { enterOverride } from '@/lib/deckControls';
import { useDeck } from '@/lib/store';
import { settleBackdrop, useTalk } from '@/lib/talk';
import { NumbersAndQA } from './Numbers';
import { VISUALS } from './slides';

/**
 * Talk: the presenter deck. A 1920x1080 stage, scaled to fit, over the same WebGL scene the tour uses. The slide order, the build
 * steps and what to say are in data/script.ts. Keys: → / space next, ← back, 1-7 sections, 0 title, N notes, Q numbers,
 * L live demo, B backdrop on/off, F fullscreen, T cinematic tour, ? help.
 */
const fmt = (s: number) => {
  const a = Math.abs(Math.round(s));
  return `${s < 0 ? '-' : ''}${Math.floor(a / 60)}:${String(a % 60).padStart(2, '0')}`;
};

export default function Talk() {
  const ready = useDeck((s) => s.ready);
  const { slide, step, notes, sheet, help, go, next, prev } = useTalk();
  const sl = SLIDES[slide];
  const Visual = VISUALS[sl.id];
  const [size, setSize] = useState({ w: 1920, h: 1080 });

  useLayoutEffect(() => {
    const f = () => setSize({ w: window.innerWidth, h: window.innerHeight });
    f();
    window.addEventListener('resize', f);
    return () => window.removeEventListener('resize', f);
  }, []);

  // back to where you were if the page is reloaded mid-talk (the slide is in the URL hash)
  useEffect(() => {
    useTalk.getState().restoreFromHash();
  }, []);

  // steer the 3D scene behind the slide
  useEffect(() => {
    if (ready) settleBackdrop();
  }, [slide, ready]);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.metaKey || e.ctrlKey || e.altKey) return;
      const st = useTalk.getState();
      const k = e.key;
      const lower = k.toLowerCase();
      if (k === 'Escape') {
        st.closeOverlays();
        return;
      }
      if (lower === 'n') st.toggle('notes');
      else if (lower === 'q') st.toggle('sheet');
      else if (k === '?' || k === '/') st.toggle('help');
      else if (lower === 'b') st.toggle('bg');
      else if (lower === 'f') {
        if (document.fullscreenElement) void document.exitFullscreen();
        else void document.documentElement.requestFullscreen?.();
      } else if (lower === 't') window.location.assign(window.location.pathname);
      else if (st.sheet) return; // the numbers sheet scrolls; the rest of the keys wait for it to close
      else if (lower === 'l') {
        st.closeOverlays();
        enterOverride();
      } else if (k === 'ArrowRight' || k === 'PageDown' || k === ' ' || k === 'Enter') {
        e.preventDefault();
        st.next();
      } else if (k === 'ArrowLeft' || k === 'PageUp' || k === 'Backspace') {
        e.preventDefault();
        st.prev();
      } else if (k >= '1' && k <= '7') {
        const i = SLIDES.findIndex((s) => s.chapter === Number(k));
        if (i >= 0) st.go(i, 0);
      } else if (k === '0' || k === 'Home') st.go(0, 0);
      else if (k === 'End') st.go(SLIDES.length - 1, SLIDES[SLIDES.length - 1].steps - 1);
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);

  if (!ready) return null;

  const notesH = notes ? Math.round(size.h * 0.32) : 0;
  const scale = Math.min(size.w / 1920, (size.h - notesH) / 1080);

  return (
    <>
      {/* dim the scene so the slide reads */}
      <div className="pointer-events-none fixed inset-0 z-10 bg-void" style={{ opacity: sl.backdrop.dim, transition: 'opacity 1.2s ease' }} />

      <div className="pointer-events-none fixed inset-0 z-20 overflow-hidden select-none">
        <div style={{ position: 'absolute', left: '50%', top: (size.h - notesH) / 2, width: 1920, height: 1080, transform: `translate(-50%, -50%) scale(${scale})` }}>
          {/* chrome */}
          <div className="absolute inset-x-[96px] top-[48px] flex items-start justify-between">
            <div className="font-display text-[30px]">NAVLESS</div>
            {sl.chapter > 0 ? (
              <div className="t-label text-center">
                <span className="text-gold">{String(sl.chapter).padStart(2, '0')}</span> / 07 · {sl.label}
              </div>
            ) : (
              <div />
            )}
            <div className="t-label text-right">skybeta-19</div>
          </div>

          {sl.chapter > 0 ? <div className="t-claim absolute left-[96px] top-[112px] max-w-[1728px]">{sl.claim}</div> : null}

          <div key={`${slide}`} className="talk-slide absolute inset-0">
            <Visual step={step} setStep={(n) => go(slide, n)} />
          </div>

          {/* section rail */}
          <div className="absolute inset-x-[96px] bottom-[40px] flex gap-[10px]">
            {SLIDES.filter((s) => s.chapter > 0).map((s) => {
              const idx = SLIDES.indexOf(s);
              const fill = idx < slide ? 1 : idx === slide ? (step + 1) / s.steps : 0;
              return (
                <button key={s.id} type="button" onClick={() => go(idx, 0)} className="pointer-events-auto flex-1 text-left" aria-label={`Go to section ${s.chapter}: ${s.label}`}>
                  <div className="relative h-[4px] bg-white/15">
                    <div className="absolute inset-y-0 left-0 bg-gold" style={{ width: `${fill * 100}%`, opacity: idx === slide ? 1 : 0.55, transition: 'width 0.6s ease' }} />
                  </div>
                  <div className={`t-label mt-2 !text-[15px] ${idx === slide ? '!text-ink' : ''}`}>
                    {String(s.chapter).padStart(2, '0')}
                    {idx === slide ? ` ${s.label}` : ''}
                  </div>
                </button>
              );
            })}
          </div>
          <button type="button" onClick={prev} aria-label="Previous" className="pointer-events-auto absolute bottom-[34px] left-[26px] h-[52px] w-[52px] text-[34px] text-ink/30 transition-colors hover:text-ink">
            ‹
          </button>
          <button type="button" onClick={next} aria-label="Next" className="pointer-events-auto absolute bottom-[34px] right-[26px] h-[52px] w-[52px] text-[34px] text-ink/30 transition-colors hover:text-ink">
            ›
          </button>
        </div>
      </div>

      {notes ? <Notes /> : null}
      {sheet ? <Sheet /> : null}
      {help ? <Help /> : null}
    </>
  );
}

/* ───────────────────────────── presenter notes (N) ───────────────────────────── */
function useElapsed(startedAt: number | null) {
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const t = window.setInterval(() => setNow(Date.now()), 500);
    return () => window.clearInterval(t);
  }, []);
  return startedAt ? (now - startedAt) / 1000 : 0;
}

function Notes() {
  const { slide, step, startedAt, resetTimer } = useTalk();
  const elapsed = useElapsed(startedAt);
  const sl = SLIDES[slide];
  const beatIdx = Math.max(0, sl.beats.findIndex((b) => b.step === step));
  const beat = sl.beats[beatIdx];
  const nextBeat = sl.beats[beatIdx + 1] ?? SLIDES[slide + 1]?.beats[0];
  const target = SLIDES.slice(0, slide + 1).reduce((n, s) => n + s.seconds, 0);
  const diff = elapsed - target;
  return (
    <div className="fixed inset-x-0 bottom-0 z-40 border-t border-white/15 bg-black/85 px-8 py-4 backdrop-blur" style={{ height: '32vh' }}>
      <div className="flex items-baseline justify-between font-mono text-[13px] uppercase tracking-[0.18em] text-ink/55">
        <span>
          {sl.label} · step {step + 1}/{sl.steps}
        </span>
        <span>
          {startedAt ? (
            <>
              {fmt(elapsed)} of {fmt(totalSeconds())} · {diff > 8 ? <span className="text-amber">{fmt(diff)} over</span> : diff < -8 ? <span className="text-ok">{fmt(-diff)} spare</span> : 'on pace'}
            </>
          ) : (
            'timer starts when you leave the title'
          )}
          <button type="button" onClick={resetTimer} className="ml-4 text-ink/40 hover:text-ink">
            reset
          </button>
        </span>
      </div>
      {beat.cue ? <div className="mt-2 font-mono text-[14px] text-ice">[{beat.cue}]</div> : null}
      <div className="mt-2 max-h-[55%] overflow-y-auto text-[clamp(18px,2.1vw,30px)] leading-[1.35] text-ink">{beat.say}</div>
      {beat.todo ? <div className="mt-2 font-mono text-[13px] text-amber">TODO: {beat.todo}</div> : null}
      {nextBeat ? <div className="mt-2 line-clamp-1 text-[14px] text-ink/40">Next: {nextBeat.say}</div> : null}
    </div>
  );
}

/* ───────────────────────────── numbers and sources (Q) ───────────────────────────── */
function Sheet() {
  return (
    <div className="fixed inset-0 z-50 overflow-y-auto bg-void/95 px-[6vw] py-10 backdrop-blur">
      <div className="mx-auto max-w-[1280px]">
        <div className="flex items-baseline justify-between">
          <div className="font-display text-[40px]">Numbers, and where they come from</div>
          <button type="button" onClick={() => useTalk.getState().toggle('sheet')} className="t-label hover:text-ink">
            Esc to close
          </button>
        </div>
        <NumbersAndQA />
      </div>
    </div>
  );
}

/* ───────────────────────────── keys (?) ───────────────────────────── */
function Help() {
  const rows: [string, string][] = [
    ['→  space  enter', 'next build / slide'],
    ['←  backspace', 'back'],
    ['1 – 7', 'jump to a section'],
    ['0  home', 'title'],
    ['N', 'notes and timer (presenter)'],
    ['Q', 'numbers and sources, likely questions'],
    ['L', 'live demo: drop a blocker on the planner (esc to leave)'],
    ['B', 'backdrop on / off (off for a weak projector)'],
    ['F', 'fullscreen'],
    ['T', 'cinematic scroll tour'],
  ];
  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-void/80 backdrop-blur" onClick={() => useTalk.getState().toggle('help')}>
      <div className="t-panel w-[min(560px,90vw)] p-8">
        <div className="t-label text-gold">Keys</div>
        <div className="mt-4 space-y-2">
          {rows.map(([k, v]) => (
            <div key={k} className="grid grid-cols-[170px_1fr] gap-4 text-[15px]">
              <span className="font-mono text-gold">{k}</span>
              <span className="text-ink/80">{v}</span>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
