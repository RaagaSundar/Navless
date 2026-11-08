'use client';

import type { CSSProperties, ReactNode } from 'react';

/** small section label, the HUD voice: mono, wide tracking, dim */
export function Label({ children, className = '', tone }: { children: ReactNode; className?: string; tone?: 'gold' | 'ice' | 'amber' }) {
  const color = tone === 'gold' ? 'text-gold' : tone === 'ice' ? 'text-ice' : tone === 'amber' ? 'text-amber' : '';
  return <div className={`t-label ${color} ${className}`}>{children}</div>;
}

/** a build step: fades and lifts in when `step` reaches `at`. Space is always reserved, so nothing shifts as it appears. */
export function Reveal({ at, step, children, className = '', style }: { at: number; step: number; children: ReactNode; className?: string; style?: CSSProperties }) {
  const on = step >= at;
  return (
    <div className={`t-reveal ${className}`} style={{ opacity: on ? 1 : 0, transform: on ? 'none' : 'translateY(14px)', ...style }} aria-hidden={!on}>
      {children}
    </div>
  );
}

/** a small outlined tag, e.g. "RUNS ON  Jetson" */
export function Chip({ k, children, tone = 'sand' }: { k?: string; children: ReactNode; tone?: 'sand' | 'gold' | 'ice' }) {
  const border = tone === 'gold' ? 'border-gold/50' : tone === 'ice' ? 'border-ice/50' : 'border-sand/30';
  const text = tone === 'gold' ? 'text-gold' : tone === 'ice' ? 'text-ice' : 'text-sand';
  return (
    <span className={`inline-flex items-baseline gap-2 border ${border} px-3 py-[5px] font-mono text-[19px] leading-none ${text}`}>
      {k ? <span className="text-[14px] uppercase tracking-[0.18em] text-ink/50">{k}</span> : null}
      {children}
    </span>
  );
}
