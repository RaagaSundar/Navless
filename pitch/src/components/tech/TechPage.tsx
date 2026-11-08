'use client';

import { useEffect, useLayoutEffect, useRef, useState, type ReactNode } from 'react';
import { SLIDES, wordsOf, type Slide } from '@/data/script';
import { NumbersAndQA } from '../talk/Numbers';
import { VISUALS } from '../talk/slides';

/**
 * TechPage: the technical companion, for the OTHER laptops (the main one is for the eye).
 * The system diagram, the evidence plot (click a run), what generalizes, what breaks it, every number with its source, the likely
 * questions, and the speaker script. Each section is the same 1920x1080 sheet the slide deck uses, fully built, scaled to fit.
 * Open it on a second laptop at /tech, jump with the bar or with #system, #evidence, #breaks, #numbers, #script.
 */
const NAV: [string, string][] = [
  ['problem', 'Problem'],
  ['system', 'System diagram'],
  ['approach', 'Why this approach'],
  ['general', 'Generalizes vs tuned'],
  ['breaks', 'What breaks it'],
  ['evidence', 'Evidence'],
  ['close', 'Next day · team'],
  ['numbers', 'Numbers · Q&A'],
  ['script', 'Script'],
];

function FitSheet({ children }: { children: ReactNode }) {
  const ref = useRef<HTMLDivElement>(null);
  const [scale, setScale] = useState(0.5);
  useLayoutEffect(() => {
    const el = ref.current;
    if (!el) return;
    const f = () => setScale(el.clientWidth / 1920);
    f();
    const ro = new ResizeObserver(f);
    ro.observe(el);
    return () => ro.disconnect();
  }, []);
  return (
    <div ref={ref} className="relative w-full overflow-hidden border border-white/10 bg-[rgb(16_9_7/0.82)]" style={{ aspectRatio: '16 / 9' }}>
      <div className="absolute left-0 top-0" style={{ width: 1920, height: 1080, transform: `scale(${scale})`, transformOrigin: 'top left' }}>
        {children}
      </div>
    </div>
  );
}

function SlideSheet({ slide }: { slide: Slide }) {
  const Visual = VISUALS[slide.id];
  const [step, setStep] = useState(slide.steps - 1);
  return (
    <section id={slide.id} className="scroll-mt-20">
      <div className="mb-2 flex items-baseline justify-between">
        <div className="t-label text-gold">
          {String(slide.chapter).padStart(2, '0')} · {slide.label}
        </div>
        {slide.steps > 1 ? (
          <div className="flex items-center gap-2 text-[12px] text-ink/50">
            <span className="uppercase tracking-[0.18em]">build</span>
            {Array.from({ length: slide.steps }, (_, i) => (
              <button key={i} type="button" onClick={() => setStep(i)} className={`h-6 w-6 border font-mono text-[11px] ${step === i ? 'border-gold bg-gold/15 text-ink' : 'border-white/20 hover:text-ink'}`}>
                {i + 1}
              </button>
            ))}
          </div>
        ) : null}
      </div>
      <FitSheet>
        <div className="t-claim absolute left-[96px] top-[70px] max-w-[1728px]">{slide.claim}</div>
        <div className="absolute inset-0 -top-[0px]" style={{ transform: 'translateY(-44px)' }}>
          <Visual step={step} setStep={setStep} />
        </div>
      </FitSheet>
    </section>
  );
}

function Script() {
  return (
    <section id="script" className="scroll-mt-20">
      <div className="t-label text-gold">Speaker script</div>
      <p className="mt-2 max-w-[820px] text-[15px] text-ink/60">
        Written to be said, not read. Cues in blue are not spoken. Also in SCRIPT.md. About {Math.round(SLIDES.reduce((n, s) => n + wordsOf(s), 0) / 150)} minutes of speech.
      </p>
      <div className="mt-6 grid gap-x-12 gap-y-9 lg:grid-cols-2">
        {SLIDES.map((s) => (
          <div key={s.id}>
            <div className="font-display text-[26px] leading-none">
              {s.chapter ? `${s.chapter} · ` : ''}
              {s.label}
            </div>
            <div className="mt-4 space-y-4">
              {s.beats.map((b, i) => (
                <div key={i}>
                  {b.cue ? <div className="font-mono text-[12px] text-ice">[{b.cue}]</div> : null}
                  <div className="mt-1 text-[19px] leading-[1.45] text-ink">{b.say}</div>
                  {b.todo ? <div className="mt-1 font-mono text-[12px] text-amber">TODO: {b.todo}</div> : null}
                </div>
              ))}
            </div>
          </div>
        ))}
      </div>
    </section>
  );
}

export default function TechPage() {
  const [active, setActive] = useState('');
  useEffect(() => {
    const ids = NAV.map(([id]) => id);
    const onScroll = () => {
      let cur = '';
      for (const id of ids) {
        const el = document.getElementById(id);
        if (el && el.getBoundingClientRect().top < 140) cur = id;
      }
      setActive(cur);
    };
    onScroll();
    window.addEventListener('scroll', onScroll, { passive: true });
    // jump to a #section once the sheets have laid out
    const t = window.setTimeout(() => {
      const id = window.location.hash.slice(1);
      if (id) document.getElementById(id)?.scrollIntoView();
    }, 250);
    return () => {
      window.removeEventListener('scroll', onScroll);
      window.clearTimeout(t);
    };
  }, []);

  return (
    <main className="min-h-screen text-ink" style={{ background: 'linear-gradient(180deg, #0d0807 0%, #1b0f0b 45%, #2a140e 100%)' }}>
      <header className="sticky top-0 z-30 border-b border-white/10 bg-void/90 backdrop-blur">
        <div className="mx-auto flex max-w-[1680px] flex-wrap items-center gap-x-6 gap-y-2 px-5 py-3">
          <a href="/" className="font-display text-[20px]" title="the visual, for the main laptop">
            NAVLESS
          </a>
          <span className="t-label !text-[12px]">technical companion</span>
          <nav className="ml-auto flex flex-wrap gap-1">
            {NAV.map(([id, label]) => (
              <a key={id} href={`#${id}`} className={`px-2.5 py-1 font-mono text-[12px] uppercase tracking-[0.12em] transition-colors ${active === id ? 'bg-gold/15 text-ink' : 'text-ink/55 hover:text-ink'}`}>
                {label}
              </a>
            ))}
          </nav>
        </div>
      </header>

      <div className="mx-auto max-w-[1680px] space-y-14 px-5 py-8">
        {SLIDES.filter((s) => s.chapter > 0).map((s) => (
          <SlideSheet key={s.id} slide={s} />
        ))}
        <section id="numbers" className="scroll-mt-20">
          <div className="font-display text-[34px]">Numbers, and where they come from</div>
          <NumbersAndQA />
        </section>
        <Script />
      </div>
    </main>
  );
}
