'use client';

import { useProgress } from '@react-three/drei';
import { AnimatePresence, motion, useSpring } from 'framer-motion';
import { useEffect, useRef, useState } from 'react';
import { useDeck } from '@/lib/store';

/**
 * Boot screen as a raw terminal. It rides on drei's useProgress (three's LoadingManager), which also counts the scan cloud
 * because ScanCloud registers its fetch with the manager. Each asset prints a line as it lands; the bar is plain block
 * characters. It clears itself a beat after everything is in.
 */
const BAR = 34;
const base = (u: string) => u.split('/').pop() ?? u;

export default function Loader() {
  const { progress, active, item, loaded, total } = useProgress();
  const ready = useDeck((s) => s.ready);
  const setReady = useDeck((s) => s.setReady);
  const spring = useSpring(0, { stiffness: 90, damping: 22, mass: 0.6 });
  const bar = useRef<HTMLPreElement>(null);
  const [done, setDone] = useState<string[]>([]);

  useEffect(() => {
    spring.set(progress);
  }, [progress, spring]);
  useEffect(() => {
    return spring.on('change', (v) => {
      const n = Math.max(0, Math.min(BAR, Math.round((v / 100) * BAR)));
      if (bar.current) bar.current.textContent = `[${'█'.repeat(n)}${'░'.repeat(BAR - n)}] ${String(Math.round(v)).padStart(3, '0')}%`;
    });
  }, [spring]);
  useEffect(() => {
    if (item) setDone((d) => (d.includes(item) ? d : [...d, item].slice(-7)));
  }, [item, loaded]);

  useEffect(() => {
    if (!active && progress >= 100) {
      const t = window.setTimeout(setReady, 650);
      return () => window.clearTimeout(t);
    }
  }, [active, progress, setReady]);
  // never strand someone on the boot screen
  useEffect(() => {
    const t = window.setTimeout(setReady, 20000);
    return () => window.clearTimeout(t);
  }, [setReady]);

  return (
    <AnimatePresence>
      {!ready && (
        <motion.div
          key="loader"
          className="fixed inset-0 z-[60] flex flex-col justify-between bg-void p-6 sm:p-10"
          initial={{ opacity: 1 }}
          exit={{ opacity: 0, transition: { duration: 0.9, ease: [0.22, 1, 0.36, 1] } }}
        >
          <div className="flex items-start justify-between">
            <div className="hud-label">[ NAVLESS // BOOT ]</div>
            <div className="hud-label">skybeta-19 · DroneHacks indoor course</div>
          </div>

          <div className="max-w-[46rem]">
            <div className="font-display mb-8 text-[clamp(3rem,10vw,9rem)] text-transparent [-webkit-text-stroke:1.5px_rgba(244,244,239,0.9)]">NAVLESS</div>
            <div className="hud-value space-y-1 text-[clamp(10px,1.05vw,13px)] leading-[1.7] text-ink/80">
              <div className="text-ink">
                navless@skybeta-19:~$ <span className="text-ink/90">./boot --deck</span>
              </div>
              <div>
                <span className="text-ok">[ ok ]</span> three r187 · webgl2 · powerPreference=high-performance
              </div>
              {done.map((d) => (
                <div key={d}>
                  <span className="text-ok">[ ok ]</span> {base(d)}
                </div>
              ))}
              <div className="text-ink/55">
                <span className="text-amber">[ .. ]</span> {total ? `${loaded}/${total} assets` : 'waiting for assets'}
                <span className="caret ml-2" />
              </div>
            </div>
            <pre ref={bar} className="hud-value mt-6 text-[clamp(10px,1.2vw,14px)] text-gold">
              {`[${'░'.repeat(BAR)}] 000%`}
            </pre>
          </div>

          <div className="hud-label flex justify-between">
            <span>sky-beta.step → 75k tris · as-built course drone19/flight · LiDAR-style resample</span>
            <span>ZERO GPS · PURE AUTONOMY</span>
          </div>
        </motion.div>
      )}
    </AnimatePresence>
  );
}
