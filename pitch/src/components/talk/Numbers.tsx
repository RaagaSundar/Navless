'use client';

import { NUMBERS } from '@/data/facts';
import { QA_LIST, SLIDES } from '@/data/script';

/** Every number on the slides with what it is and where it came from, then the likely questions. Shared by the Q overlay and /tech. */
export function NumbersAndQA() {
  const groups = SLIDES.filter((s) => s.chapter > 0).map((s) => ({ s, rows: NUMBERS.filter((n) => n.section === s.chapter) }));
  return (
    <>
      <p className="mt-3 max-w-[900px] text-[15px] leading-[1.5] text-ink/65">
        Every number we show, what it is, and the file, log or person it came from. The flight-recorder logs are on branch flight-logs/2026-11-08.
      </p>
      {groups.map(({ s, rows }) => (
        <div key={s.id} className="mt-8">
          <div className="t-label text-gold">
            {String(s.chapter).padStart(2, '0')} · {s.label}
          </div>
          <div className="mt-2 divide-y divide-white/10 border-y border-white/10">
            {rows.map((n) => (
              <div key={n.id} className="grid grid-cols-[minmax(120px,200px)_1fr_1fr] gap-x-6 py-2 text-[15px] leading-[1.4]">
                <div className="font-mono font-bold text-gold">{n.value}</div>
                <div className="text-ink/90">{n.what}</div>
                <div className="text-ink/50">{n.source}</div>
              </div>
            ))}
          </div>
        </div>
      ))}
      <div className="mt-10">
        <div className="t-label text-gold">If they ask</div>
        <div className="mt-2 divide-y divide-white/10 border-y border-white/10">
          {QA_LIST.map((q) => (
            <div key={q.q} className="grid grid-cols-[minmax(160px,360px)_1fr] gap-x-6 py-3 text-[15px] leading-[1.45]">
              <div className="text-ink">{q.q}</div>
              <div className="text-ink/75">{q.a}</div>
            </div>
          ))}
        </div>
      </div>
    </>
  );
}
