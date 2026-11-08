/**
 * talk.ts: state + controls for TALK MODE, the slide-by-slide version of the deck (opt-in: `?mode=talk`).
 *
 * The cinematic scroll tour is the default: that is what this laptop shows. Talk mode replaces scrolling with slides and build
 * steps. The technical content (diagram, evidence, numbers, script) now lives on /tech for the other laptops. Slide order, builds and what to say live in data/script.ts.
 * The WebGL scene behind the slides is steered by the same master `progress` value the tour uses; a slide just names where.
 */
import { animate } from 'framer-motion';
import { create } from 'zustand';
import { SLIDES } from '@/data/script';
import { progress } from './motion';

export type Mode = 'talk' | 'tour';

let cached: Mode | null = null;
/** the cinematic scroll deck is the default (this laptop is for the eye); `?mode=talk` opens the slide deck. Decided once per page load. */
export function getMode(): Mode {
  if (cached) return cached;
  if (typeof window === 'undefined') return 'tour';
  cached = new URLSearchParams(window.location.search).get('mode') === 'talk' ? 'talk' : 'tour';
  return cached;
}

const clampInt = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, Math.round(v)));

/** `#evidence.2` -> { slide: 6, step: 2 } */
function parseHash(): { slide: number; step: number } | null {
  if (typeof window === 'undefined') return null;
  const m = /^#([a-z]+)(?:\.(\d+))?$/.exec(window.location.hash);
  if (!m) return null;
  const slide = SLIDES.findIndex((s) => s.id === m[1]);
  if (slide < 0) return null;
  return { slide, step: clampInt(Number(m[2] ?? 0), 0, SLIDES[slide].steps - 1) };
}

function writeHash(slide: number, step: number) {
  try {
    window.history.replaceState(null, '', `#${SLIDES[slide].id}${step ? `.${step}` : ''}`);
  } catch {
    /* sandboxed frames: the hash is a convenience only */
  }
}

interface TalkState {
  slide: number;
  step: number;
  /** presenter notes panel (N) */
  notes: boolean;
  /** numbers and sources sheet (Q) */
  sheet: boolean;
  help: boolean;
  /** show the 3D backdrop (B toggles it off for a weak projector laptop) */
  bg: boolean;
  /** ms epoch of the first move off the title slide: the presenter's timer, shown only in the notes panel */
  startedAt: number | null;

  go: (slide: number, step?: number) => void;
  next: () => void;
  prev: () => void;
  toggle: (k: 'notes' | 'sheet' | 'help' | 'bg') => void;
  closeOverlays: () => void;
  restoreFromHash: () => void;
  resetTimer: () => void;
}

export const useTalk = create<TalkState>((set, get) => ({
  slide: 0,
  step: 0,
  notes: false,
  sheet: false,
  help: false,
  bg: true,
  startedAt: null,

  go: (slide, step = 0) => {
    const s = clampInt(slide, 0, SLIDES.length - 1);
    const t = clampInt(step, 0, SLIDES[s].steps - 1);
    const { startedAt } = get();
    set({ slide: s, step: t, startedAt: startedAt ?? (s > 0 ? Date.now() : null) });
    writeHash(s, t);
  },
  next: () => {
    const { slide, step, go } = get();
    if (step < SLIDES[slide].steps - 1) go(slide, step + 1);
    else if (slide < SLIDES.length - 1) go(slide + 1, 0);
  },
  prev: () => {
    const { slide, step, go } = get();
    if (step > 0) go(slide, step - 1);
    else if (slide > 0) go(slide - 1, SLIDES[slide - 1].steps - 1);
  },
  toggle: (k) => set((st) => ({ [k]: !st[k] }) as Pick<TalkState, typeof k>),
  closeOverlays: () => set({ notes: false, sheet: false, help: false }),
  restoreFromHash: () => {
    const h = parseHash();
    if (h) set({ slide: h.slide, step: h.step, startedAt: h.slide > 0 ? Date.now() : null });
  },
  resetTimer: () => set({ startedAt: null }),
}));

/** glide the 3D scene to the current slide's backdrop (used on every slide change and when the live demo hands control back) */
export function settleBackdrop(duration = 1.6) {
  const { slide } = useTalk.getState();
  animate(progress, SLIDES[slide].backdrop.p, { duration, ease: [0.22, 1, 0.36, 1] });
}
