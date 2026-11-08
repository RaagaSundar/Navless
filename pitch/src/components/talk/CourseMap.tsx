'use client';

import { memo, useMemo } from 'react';
import type { RunNote } from '@/data/facts';
import { COURSE, ROUTE } from '@/lib/course';
import type { Vec2, Wall } from '@/lib/course-types';
import { PAL } from '@/lib/palette';
import type { EvidenceRun } from './evidence';

/**
 * CourseMap: a flat top-down drawing of the as-built course (north up), from the same course file the planner and the simulator
 * read. Used for the problem slide (whole course, checkpoints) and the evidence slide (the north half, with a real flight's
 * believed track on it). Course coordinates in, SVG out: the only flip is y -> -y so north is up.
 */
export interface MapView {
  x0: number;
  x1: number;
  y0: number;
  y1: number;
}
export const WHOLE: MapView = { x0: -3.35, x1: 3.35, y0: -3.35, y1: 3.35 };
export const NORTH: MapView = { x0: -3.3, x1: 1.05, y0: 0.15, y1: 3.3 };

const r3 = (n: number) => Math.round(n * 1000) / 1000;
const pt = (x: number, y: number) => `${r3(x)},${r3(-y)}`;

/** a wall as one or two segments: the window wall has a gap where the opening is */
function segments(w: Wall): [Vec2, Vec2][] {
  if (!w.window) return [[w.a, w.b]];
  const [ax, ay] = w.a;
  const [bx, by] = w.b;
  const len = Math.hypot(bx - ax, by - ay);
  const ux = (bx - ax) / len;
  const uy = (by - ay) / len;
  const t = (w.window.center[0] - ax) * ux + (w.window.center[1] - ay) * uy;
  const hw = w.window.width / 2;
  return [
    [w.a, [ax + ux * (t - hw), ay + uy * (t - hw)]],
    [[ax + ux * (t + hw), ay + uy * (t + hw)], w.b],
  ];
}

const ROUTE_D = 'M' + ROUTE.waypoints.map((w) => pt(w[0], w[1])).join(' L');

const LABELLED = new Set(['MN', 'MD', 'MT']);

interface Props {
  view: MapView;
  /** the real flight to draw: believed track, the setpoints it was sent, and the impact */
  run?: EvidenceRun | null;
  note?: RunNote | null;
  /** other flights, drawn faint (the overview before a run is picked, and the summary after) */
  others?: EvidenceRun[];
  checkpoints?: 'full' | 'faint' | 'none';
  /** extra emphasis: label the 36 in window and the 1 m gap */
  dims?: boolean;
  className?: string;
}

function CourseMapInner({ view, run = null, note = null, others = [], checkpoints = 'full', dims = false, className }: Props) {
  const { x0, x1, y0, y1 } = view;
  const N = COURSE.net.halfX;
  const believed = useMemo(() => (run ? 'M' + run.points.map((p) => pt(p[1] as number, p[2] as number)).join(' L') : ''), [run]);
  const sent = useMemo(() => {
    if (!run) return '';
    const rows = run.points.filter((p) => p[3] !== null && p[4] !== null);
    return rows.length > 1 ? 'M' + rows.map((p) => pt(p[3] as number, p[4] as number)).join(' L') : '';
  }, [run]);

  const otherPaths = useMemo(() => others.map((o) => ({ run: o, d: 'M' + o.points.map((p) => pt(p[1] as number, p[2] as number)).join(' L') })), [others]);

  const hitWall = note?.hit === 'MD' ? COURSE.walls.find((w) => w.name === 'MD') : undefined;
  const fs = (x1 - x0) / 46; // label size relative to the shown width, so labels stay legible whether zoomed or not

  return (
    <svg viewBox={`${x0} ${-y1} ${x1 - x0} ${y1 - y0}`} className={className} role="img" aria-label="Top-down map of the course, north up" style={{ overflow: 'hidden' }}>
      {/* floor and net */}
      <rect x={-N} y={-N} width={2 * N} height={2 * N} fill={PAL.regolith} fillOpacity={0.3} stroke={PAL.sand} strokeOpacity={0.5} strokeWidth={0.03} />

      {/* tunnel (dark inside) */}
      <rect
        x={COURSE.tunnel.xMin}
        y={-COURSE.tunnel.yMax}
        width={COURSE.tunnel.xMax - COURSE.tunnel.xMin}
        height={COURSE.tunnel.yMax - COURSE.tunnel.yMin}
        rx={0.25}
        fill={PAL.void}
        fillOpacity={0.55}
        stroke={PAL.dust}
        strokeOpacity={0.6}
        strokeWidth={0.025}
        strokeDasharray="0.09 0.07"
      />
      <text x={(COURSE.tunnel.xMin + COURSE.tunnel.xMax) / 2} y={-(COURSE.tunnel.yMin + COURSE.tunnel.yMax) / 2} fontSize={fs} textAnchor="middle" fill={PAL.sand} fillOpacity={0.55} fontFamily="var(--font-mono)" letterSpacing={fs * 0.15}>
        TUNNEL · DARK
      </text>

      {/* boulders and pad */}
      {COURSE.boulders.map((b, i) => (
        <circle key={i} cx={b.center[0]} cy={-b.center[1]} r={b.diameter / 2} fill={PAL.dust} fillOpacity={0.4} />
      ))}
      <g fill="none" stroke={PAL.dust} strokeOpacity={0.75} strokeWidth={0.02}>
        <circle cx={COURSE.pad.center[0]} cy={-COURSE.pad.center[1]} r={COURSE.pad.rOuter} />
        {COURSE.pad.rings.map((r) => (
          <circle key={r} cx={COURSE.pad.center[0]} cy={-COURSE.pad.center[1]} r={r} />
        ))}
      </g>

      {/* walls */}
      {COURSE.walls.map((w) =>
        segments(w).map(([a, b], i) => (
          <line key={`${w.name}${i}`} x1={a[0]} y1={-a[1]} x2={b[0]} y2={-b[1]} stroke={w.name === hitWall?.name ? PAL.danger : PAL.sand} strokeOpacity={w.name === hitWall?.name ? 1 : 0.85} strokeWidth={w.thickness * (w.name === hitWall?.name ? 1.7 : 1)} />
        )),
      )}
      {note?.hit === 'north net' ? <line x1={-N} y1={-N} x2={N} y2={-N} stroke={PAL.danger} strokeWidth={0.07} /> : null}
      {note?.hit === 'west net' ? <line x1={-N} y1={-N} x2={-N} y2={N} stroke={PAL.danger} strokeWidth={0.07} /> : null}

      {/* wall names, on the side the drone flies */}
      {COURSE.walls
        .filter((w) => LABELLED.has(w.name))
        .map((w) => {
          const len = Math.hypot(w.b[0] - w.a[0], w.b[1] - w.a[1]);
          const ux = (w.b[0] - w.a[0]) / len;
          const uy = (w.b[1] - w.a[1]) / len;
          let nx = -uy;
          let ny = ux;
          if (ny < 0 || (Math.abs(ny) < 1e-6 && nx < 0)) {
            nx = -nx;
            ny = -ny;
          }
          // MN hugs the north net, so its name goes on the room side
          if (w.name === 'MN') {
            nx = -nx;
            ny = -ny;
          }
          const mx = (w.a[0] + w.b[0]) / 2 + nx * 0.2;
          const my = (w.a[1] + w.b[1]) / 2 + ny * 0.2;
          return (
            <text key={w.name} x={mx} y={-my} fontSize={fs * 1.05} textAnchor="middle" dominantBaseline="middle" fill={w.name === hitWall?.name ? PAL.danger : PAL.sand} fillOpacity={0.9} fontFamily="var(--font-mono)" fontWeight={700}>
              {w.name}
            </text>
          );
        })}

      {/* the gap and the window, when the problem slide wants them measured */}
      {dims ? (
        <g fontFamily="var(--font-mono)" fontSize={fs} fill={PAL.ice}>
          <line x1={COURSE.gates.gate_A.a[0]} y1={-COURSE.gates.gate_A.a[1]} x2={COURSE.gates.gate_A.b[0]} y2={-COURSE.gates.gate_A.b[1]} stroke={PAL.ice} strokeWidth={0.03} strokeDasharray="0.07 0.05" />
          <text x={COURSE.gates.gate_A.a[0] + 0.12} y={-(COURSE.gates.gate_A.a[1] + 0.5)} fillOpacity={0.9}>
            1 m gap
          </text>
          <text x={-0.1} y={-(-1.95)} fillOpacity={0.9}>
            36 in window
          </text>
        </g>
      ) : null}

      {/* the real route we plan */}
      <path d={ROUTE_D} fill="none" stroke={PAL.gold} strokeOpacity={run ? 0.4 : 0.85} strokeWidth={0.035} strokeDasharray="0.12 0.08" strokeLinejoin="round" />

      {/* checkpoints */}
      {checkpoints !== 'none'
        ? COURSE.checkpoints.map((cp) => (
            <g key={cp.n} opacity={checkpoints === 'faint' ? 0.5 : 1}>
              <circle cx={cp.at[0]} cy={-cp.at[1]} r={0.19} fill={PAL.void} fillOpacity={0.8} stroke={PAL.gold} strokeWidth={0.03} />
              <text x={cp.at[0]} y={-cp.at[1]} fontSize={0.24} textAnchor="middle" dominantBaseline="central" fill={PAL.gold} fontFamily="var(--font-mono)" fontWeight={800}>
                {cp.n}
              </text>
              {checkpoints === 'full' ? (
                <text x={cp.at[0] + 0.27} y={-cp.at[1] + 0.02} fontSize={fs * 1.05} dominantBaseline="central" fill={PAL.sand} fillOpacity={0.9} fontFamily="var(--font-mono)" fontWeight={700}>
                  +{cp.points}
                </text>
              ) : null}
            </g>
          ))
        : null}

      {/* launch mark */}
      <g transform={`translate(${COURSE.launch[0]} ${-COURSE.launch[1]}) rotate(45)`}>
        <rect x={-0.08} y={-0.08} width={0.16} height={0.16} fill={PAL.gold} />
      </g>
      <text x={COURSE.launch[0] + 0.16} y={-COURSE.launch[1] + 0.26} fontSize={fs} fill={PAL.gold} fontFamily="var(--font-mono)" fontWeight={700} letterSpacing={fs * 0.1}>
        START
      </text>

      {/* the other flights, faint */}
      {otherPaths.map(({ run: o, d }) => (
        <g key={o.id} opacity={0.8}>
          <path d={d} fill="none" stroke={PAL.ice} strokeOpacity={0.65} strokeWidth={0.045} strokeLinecap="round" strokeLinejoin="round" />
          <circle cx={o.contact[0]} cy={-o.contact[1]} r={0.2375} fill={PAL.ice} fillOpacity={0.12} stroke={PAL.ice} strokeOpacity={0.8} strokeWidth={0.025} strokeDasharray="0.06 0.04" />
          <text x={o.contact[0]} y={-o.contact[1]} fontSize={fs * 1.1} textAnchor="middle" dominantBaseline="central" fill={PAL.sand} fontFamily="var(--font-mono)" fontWeight={800}>
            {o.label.replace('Run ', '')}
          </text>
        </g>
      ))}

      {/* a real flight: setpoints sent, then what the drone believed */}
      {run ? (
        <g key={run.id}>
          {sent ? <path d={sent} fill="none" stroke={PAL.dust} strokeWidth={0.03} strokeDasharray="0.05 0.06" strokeOpacity={0.9} /> : null}
          <path d={believed} pathLength={1} className="t-draw" fill="none" stroke={PAL.ice} strokeWidth={0.055} strokeLinecap="round" strokeLinejoin="round" />
          <g className="t-fade" style={{ animationDelay: '3.2s' }}>
            <circle cx={run.contact[0]} cy={-run.contact[1]} r={0.2375} fill={PAL.ice} fillOpacity={0.14} stroke={PAL.ice} strokeWidth={0.03} strokeDasharray="0.06 0.04" />
            <path d={`M${r3(run.contact[0] - 0.07)},${r3(-run.contact[1])} h0.14 M${r3(run.contact[0])},${r3(-run.contact[1] - 0.07)} v0.14`} stroke={PAL.ice} strokeWidth={0.03} />
            <text x={run.contact[0] + 0.3} y={-run.contact[1] - 0.25} fontSize={fs * 1.1} fill={PAL.ice} fontFamily="var(--font-mono)" fontWeight={700}>
              {`touched at +${run.tContact.toFixed(1)} s`}
            </text>
          </g>
        </g>
      ) : null}
    </svg>
  );
}

export default memo(CourseMapInner);
