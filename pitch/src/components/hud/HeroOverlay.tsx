'use client';

import { motion, useTransform } from 'framer-motion';
import { useWindow } from './hooks';

/** Scene 1 furniture: three plain lines under the airframe, once it has turned to face you. Nothing else. */
const SPEC: [string, string][] = [
  ['GPS', 'none'],
  ['EYES', 'two stereo cameras, 31 readings a second'],
  ['BRAIN', 'a Jetson Orin Nano, on board'],
];

export default function HeroOverlay() {
  const spec = useWindow(0.2, 0.37, 0.025);
  const specX = useTransform(spec, [0, 1], [-30, 0]);

  return (
    <div className="pointer-events-none fixed inset-0 z-30">
      <motion.div data-hud="hero-spec" className="absolute bottom-24 left-5 w-[min(25rem,70vw)] sm:bottom-28 sm:left-8" style={{ opacity: spec, x: specX }}>
        <div className="border-l border-white/20 pl-3">
          {SPEC.map(([k, v]) => (
            <div key={k} className="flex gap-3 py-[4px]">
              <span className="hud-label w-[3.6rem] flex-none">{k}</span>
              <span className="hud-value text-[12.5px] leading-[1.5] text-ink/90">{v}</span>
            </div>
          ))}
        </div>
      </motion.div>
    </div>
  );
}
