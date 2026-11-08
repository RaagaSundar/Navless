'use client';

import { motion, useTransform, type MotionValue } from 'framer-motion';
import { T } from '@/lib/choreography';
import { useWindow } from './hooks';

/**
 * Scene 2 type. Each beat is a window of the scroll track: a headline that unmasks upward like a curtain, one plain sentence
 * about what it means, and one line of specs for anyone who wants the numbers (from hardware_slam/README.md, navless_nav and
 * planner/). No chips, no stacked labels: a judge should be able to read each beat in five seconds.
 */
interface Beat {
  n: string;
  tag: string;
  lines: [string, string];
  plain: string;
  spec: string;
  range: readonly [number, number];
}

const BEATS: Beat[] = [
  {
    n: '01',
    tag: 'PLANNING',
    lines: ['A* + EGO', 'B-SPLINE PLANNING'],
    plain: 'It finds the shortest safe path through the maze, then smooths it into a curve the drone can actually fly.',
    spec: 'stays 28 cm clear of every wall · re-plans about once a second',
    range: T.callouts[0],
  },
  {
    n: '02',
    tag: 'LOCALISATION',
    lines: ['OAK-D-S2', 'STEREO ODOMETRY'],
    plain: 'Two cameras track the walls to work out how the drone is moving. No GPS, no beacons.',
    spec: 'cuVSLAM on a Jetson Orin Nano · 31 updates a second',
    range: T.callouts[1],
  },
  {
    n: '03',
    tag: 'THE DARK TUNNEL',
    lines: ['BLIND CARRY', 'THROUGH THE DARK'],
    plain: 'The tunnel is dark: the cameras see nothing for about three seconds. The drone keeps flying the route on the autopilot’s own estimate.',
    spec: 'rides PX4’s own estimate for at most 4 s · the tunnel is about 3 s dark',
    range: T.callouts[2],
  },
];

function Mask({ children, mix, delay = 0 }: { children: string; mix: MotionValue<number>; delay?: number }) {
  const y = useTransform(mix, [0, 1], ['112%', '0%']);
  const o = useTransform(mix, [0, 0.25, 1], [0, 1, 1]);
  return (
    <span className="block overflow-hidden pb-[0.06em]" style={{ transitionDelay: `${delay}ms` }}>
      <motion.span className="block" style={{ y, opacity: o }}>
        {children}
      </motion.span>
    </span>
  );
}

function Callout({ b }: { b: Beat }) {
  const mix = useWindow(b.range[0], b.range[1]);
  const mix2 = useTransform(mix, (v) => Math.max(0, (v - 0.15) / 0.85));
  const x = useTransform(mix, [0, 1], [-24, 0]);
  const rule = useTransform(mix, [0, 1], [0, 1]);
  return (
    <motion.div data-hud="callout" className="pointer-events-none absolute bottom-[12vh] left-5 w-[min(76rem,92vw)] sm:left-[5.5vw]" style={{ opacity: mix }}>
      <div className="mb-4 flex items-center gap-4">
        <span className="hud-value text-[13px] hud-accent">[ {b.n} ]</span>
        <motion.i className="block h-px w-20 origin-left bg-[var(--accent)]" style={{ scaleX: rule }} />
        <span className="hud-label">{b.tag}</span>
      </div>
      <h2 className="font-display text-[clamp(2.1rem,5vw,6.2rem)] text-ink drop-shadow-[0_2px_30px_rgba(13,8,7,0.9)]">
        <Mask mix={mix}>{b.lines[0]}</Mask>
        <Mask mix={mix} delay={80}>
          {b.lines[1]}
        </Mask>
      </h2>
      <motion.p className="hud-value mt-6 max-w-[34rem] text-[clamp(13px,1.05vw,15px)] leading-[1.65] text-ink/90" style={{ opacity: mix2, x }}>
        {b.plain}
      </motion.p>
      <motion.p className="hud-label mt-3 max-w-[40rem] normal-case tracking-[0.06em] text-ink/55" style={{ opacity: mix2, x }}>
        {b.spec}
      </motion.p>
    </motion.div>
  );
}

/** the bridge between scenes 1 and 2: scan -> map */
function Bridge() {
  const mix = useWindow(0.375, 0.5, 0.02);
  const y = useTransform(mix, [0, 1], [16, 0]);
  return (
    <motion.div data-hud="bridge" className="pointer-events-none absolute left-5 top-[19vh] w-[min(31rem,86vw)] sm:left-[5.5vw]" style={{ opacity: mix, y }}>
      <div className="hud-label hud-accent mb-3">[ SCAN → MAP ]</div>
      <div className="font-display text-[clamp(1.6rem,3.4vw,3.6rem)]">
        PHONE LIDAR.
        <br />
        THE REAL COURSE.
      </div>
      <p className="hud-value mt-4 max-w-[27rem] text-[clamp(13px,1.05vw,15px)] leading-[1.65] text-ink/90">We scanned the course with a phone, built a map from it, and the planner works on that map.</p>
      <p className="hud-label mt-3 normal-case tracking-[0.06em] text-ink/45">The points shown are a resample of that map; the raw scan is not in the repo.</p>
    </motion.div>
  );
}

export default function Callouts() {
  return (
    <div className="pointer-events-none fixed inset-0 z-30">
      <Bridge />
      {BEATS.map((b) => (
        <Callout key={b.n} b={b} />
      ))}
    </div>
  );
}
