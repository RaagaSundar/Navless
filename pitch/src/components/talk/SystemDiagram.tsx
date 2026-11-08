'use client';

import { N } from '@/data/facts';
import { TODAY } from '@/data/today';

/**
 * SystemDiagram: the ONE diagram the judges asked for. Five stages left to right: what it senses, how it works out where it is,
 * what is around it, how it decides, how that becomes motor commands. Every card says where it runs and how fast it loops.
 * Numbers come from data/facts.ts (README + logs), so the diagram cannot drift from the script.
 *
 * The slide builds one stage per step; the whole drawing is visible, faint, from the start, so it reads as one picture.
 */
const CW = 292;
const GAP = 67;
const CH = 490;
const TOP = 100;
const W = 5 * CW + 4 * GAP; // 1728
const H = 690;

type Where = 'jetson' | 'pixhawk' | 'both';
interface Stage {
  n: string;
  name: string;
  q: string;
  lines: string[];
  runs: string;
  where: Where;
  rate: string;
  out: string;
}

const STAGES: Stage[] = [
  {
    n: '01',
    name: 'Sense',
    q: 'What it sees',
    lines: ['Stereo camera, 1024×640', '2 side ToF sensors, for the tunnel', 'Down laser, for height'],
    runs: 'Jetson · Pixhawk',
    where: 'both',
    rate: `${N.camera.value.split(' at ')[1]} cam · ${N.tof.value} ToF`,
    out: 'images, ranges',
  },
  {
    n: '02',
    name: 'Estimate',
    q: 'Where am I?',
    lines: ['cuVSLAM, stereo only', `Gate: tracking, under ${N.stale.value} old`, 'PX4 EKF2 fuses it as vision'],
    runs: 'Jetson → Pixhawk',
    where: 'both',
    rate: `${N.vslam.value} → ${N.gate.value}`,
    out: 'pose',
  },
  {
    n: '03',
    name: 'Plan',
    q: 'What is around me?',
    lines: [`Phone-LiDAR scan: ${N.scan.value}`, 'A* on 0.1 m cells, walls +0.28 m', 'Smoothed to a B-spline'],
    runs: 'Jetson',
    where: 'jetson',
    rate: `1 s re-plan period${TODAY.replanning ? '' : ' (off)'}`,
    out: 'path',
  },
  {
    n: '04',
    name: 'Decide',
    q: 'What do I do?',
    lines: ['State machine: wait, take off, route, land', 'Hold, refuse, pause', 'Guards on heading, tracking, net'],
    runs: 'Jetson',
    where: 'jetson',
    rate: N.mission.value,
    out: 'setpoints',
  },
  {
    n: '05',
    name: 'Act',
    q: 'Make it move',
    lines: ['Organisers’ flight_interface', 'PX4 Offboard, position control', 'We never arm or switch mode'],
    runs: 'Jetson → Pixhawk',
    where: 'both',
    rate: `${N.heartbeat.value} heartbeat`,
    out: 'motor commands',
  },
];

const x = (i: number) => i * (CW + GAP);
const cx = (i: number) => x(i) + CW / 2;
const WHERE_TONE: Record<Where, string> = { jetson: 'text-gold', pixhawk: 'text-ice', both: 'text-sand' };

export default function SystemDiagram({ step }: { step: number }) {
  const all = step >= 5;
  const midY = TOP + CH / 2;
  return (
    <div className="relative" style={{ width: W, height: H }}>
      {/* connectors */}
      <svg width={W} height={H} viewBox={`0 0 ${W} ${H}`} className="absolute inset-0" aria-hidden>
        <defs>
          <marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="9" markerHeight="9" orient="auto">
            <path d="M0,0 L10,5 L0,10 z" fill="currentColor" />
          </marker>
        </defs>
        {/* stage to stage */}
        {[0, 1, 2, 3].map((i) => (
          <line
            key={i}
            x1={x(i) + CW + 8}
            y1={midY}
            x2={x(i + 1) - 8}
            y2={midY}
            stroke="#f1dcc0"
            strokeWidth={3}
            markerEnd="url(#arrow)"
            style={{ color: '#f1dcc0', opacity: step >= i + 1 ? 0.9 : 0.25, transition: 'opacity 0.7s ease' }}
          />
        ))}
        {/* side ToF straight to the decision: the tunnel offset skips estimation */}
        <g style={{ color: '#9cc6dd', opacity: step >= 3 ? 0.95 : 0.2, transition: 'opacity 0.7s ease' }}>
          <path d={`M${cx(0)},${TOP + CH} v44 H${cx(3)} V${TOP + CH + 6}`} fill="none" stroke="#9cc6dd" strokeWidth={3} strokeDasharray="10 7" markerEnd="url(#arrow)" />
          <text x={(cx(0) + cx(3)) / 2} y={TOP + CH + 76} textAnchor="middle" fill="#9cc6dd" fontFamily="var(--font-mono)" fontSize={22} letterSpacing={1.5}>
            tunnel offset = (L − R) / 2
          </text>
        </g>
        {/* the loop closes: the drone moves, the camera sees it */}
        <g style={{ color: '#f6b25e', opacity: all ? 0.95 : 0.0, transition: 'opacity 0.9s ease' }}>
          <path d={`M${cx(4)},${TOP - 6} v-40 H${cx(0)} V${TOP - 8}`} fill="none" stroke="#f6b25e" strokeWidth={3} markerEnd="url(#arrow)" />
          <rect x={(cx(0) + cx(4)) / 2 - 250} y={TOP - 66} width={500} height={32} fill="#1b0f0b" />
          <text x={(cx(0) + cx(4)) / 2} y={TOP - 43} textAnchor="middle" fill="#f6b25e" fontFamily="var(--font-mono)" fontSize={22} letterSpacing={1.5}>
            the drone moves · the camera sees it move
          </text>
        </g>
      </svg>

      {STAGES.map((s, i) => {
        const active = all || step === i;
        const seen = step > i;
        const opacity = active ? 1 : seen ? 0.62 : 0.3;
        return (
          <div
            key={s.n}
            className="t-panel absolute"
            style={{
              left: x(i),
              top: TOP,
              width: CW,
              height: CH,
              opacity,
              borderColor: active && !all ? 'rgb(246 178 94 / 0.85)' : undefined,
              transition: 'opacity 0.7s ease, border-color 0.7s ease',
            }}
          >
            <div className="px-[22px] pt-[20px]">
              <div className="t-label text-gold">
                {s.n} · {s.name}
              </div>
              <div className="mt-3 font-display text-[38px] leading-[0.98]">{s.q}</div>
              <ul className="mt-5 space-y-[12px] font-mono text-[22px] leading-[1.22] text-ink/90">
                {s.lines.map((l) => (
                  <li key={l}>{l}</li>
                ))}
              </ul>
            </div>
            <div className="absolute inset-x-[22px] bottom-[16px] space-y-[7px] border-t border-white/12 pt-[10px]">
              <Meta k="runs on" v={s.runs} tone={WHERE_TONE[s.where]} />
              <Meta k="rate" v={s.rate} />
              <Meta k="out" v={s.out} />
            </div>
          </div>
        );
      })}
    </div>
  );
}

function Meta({ k, v, tone = 'text-ink' }: { k: string; v: string; tone?: string }) {
  return (
    <div>
      <div className="font-mono text-[13px] uppercase tracking-[0.2em] text-ink/45">{k}</div>
      <div className={`font-mono text-[20px] leading-[1.15] ${tone}`}>{v}</div>
    </div>
  );
}
