'use client';

import type { ComponentType } from 'react';
import { N, RUN_3, RUN_NOTES, TEAM } from '@/data/facts';
import { NOT_DONE, type SlideId } from '@/data/script';
import { TODAY } from '@/data/today';
import { COURSE } from '@/lib/course';
import CourseMap, { NORTH, WHOLE } from './CourseMap';
import SystemDiagram from './SystemDiagram';
import { EVIDENCE } from './evidence';
import { Label, Reveal } from './ui';

/**
 * The slide visuals. Each one lays itself out on the 1920x1080 stage and builds up by `step`; what is said at each step is in
 * data/script.ts. Slide text is phrases and numbers only: the sentences are spoken.
 */
export type SlideProps = { step: number; setStep?: (s: number) => void };

/* ───────────────────────────── 0 · title ───────────────────────────── */
const SPEC: [string, string][] = [
  ['GPS', 'none'],
  ['EYES', 'two stereo cameras, 31 readings a second'],
  ['BRAIN', 'a Jetson Orin Nano, on board'],
];

function Title() {
  return (
    <div className="absolute inset-0">
      <div className="absolute bottom-[150px] left-[96px] border-l border-white/20 pl-5">
        {SPEC.map(([k, v]) => (
          <div key={k} className="flex gap-5 py-[5px]">
            <span className="t-label w-[96px] flex-none">{k}</span>
            <span className="font-mono text-[25px] leading-[1.4] text-ink/90">{v}</span>
          </div>
        ))}
      </div>
      <div className="absolute inset-x-0 bottom-[96px] text-center">
        <div className="t-label text-gold">DroneHacks · judging pitch · 3:20 PM</div>
        <div className="mt-2 font-mono text-[23px] text-ink/80">Raaga Sundar · Bryan Pham · Favour Adekola · Shining Wang · Koji Shukawa</div>
      </div>
    </div>
  );
}

/* ───────────────────────────── 1 · the problem ───────────────────────────── */
const CP_NOTE: Record<number, string> = { 2: '1 m gap', 3: 'dark, ~3 s blind', 4: '36 in window', 5: 'bullseye' };

function Problem({ step }: SlideProps) {
  return (
    <>
      <div className="absolute left-[96px] top-[250px] w-[880px]">
        <Reveal at={0} step={step}>
          <Label tone="gold">The course · {N.points.value}</Label>
          <div className="mt-3">
            {COURSE.checkpoints.map((cp) => (
              <div key={cp.n} className="flex items-baseline gap-5 py-[3px] text-[29px] leading-[1.25]">
                <span className="w-[30px] font-display text-[32px] text-gold">{cp.n}</span>
                <span className="flex-1">
                  {cp.name.charAt(0) + cp.name.slice(1).toLowerCase()}
                  {CP_NOTE[cp.n] ? <span className="text-ink/55"> · {CP_NOTE[cp.n]}</span> : null}
                </span>
                <span className="font-mono tabular-nums text-sand">{cp.points}</span>
              </div>
            ))}
          </div>
          <div className="mt-2 text-[24px] text-ink/60">
            Drone {N.droneWidth.value} wide with props · gap leaves {N.gapSide.value} a side
          </div>
        </Reveal>

        <Reveal at={1} step={step} className="mt-8">
          <Label tone="gold">What mattered most</Label>
          <div className="mt-2 font-display text-[54px] leading-none">Clearance, not speed</div>
          <div className="mt-2 text-[26px] text-ink/80">
            {N.room.value} planned prop room. Real errors: {N.errors.value}.
          </div>
        </Reveal>

        <Reveal at={2} step={step} className="mt-8">
          <Label tone="gold">We chose not to</Label>
          <div className="mt-2 space-y-[9px]">
            {NOT_DONE.map((d) => (
              <div key={d.title} className="leading-[1.2]">
                <span className="text-[27px]">{d.title}</span>
                <span className="text-[23px] text-ink/55"> · {d.why}</span>
              </div>
            ))}
          </div>
        </Reveal>
      </div>

      <div className="t-panel absolute right-[96px] top-[246px] h-[730px] w-[730px] p-[10px]" style={{ background: 'rgb(14 8 6 / 0.88)' }}>
        <CourseMap view={WHOLE} dims className="h-full w-full" />
      </div>
    </>
  );
}

/* ───────────────────────────── 2 · how it works ───────────────────────────── */
function System({ step }: SlideProps) {
  return (
    <div className="absolute left-[96px] top-[262px]">
      <SystemDiagram step={step} />
    </div>
  );
}

/* ───────────────────────────── 3 · why this approach ───────────────────────────── */
function Approach({ step }: SlideProps) {
  const cards = [
    {
      n: '01',
      name: 'The map',
      tried: 'Plan on the organisers’ map',
      found: `Our phone scan put five walls elsewhere. M3 sat ${N.m3.value} off, and the route passed 2.8 cm from it`,
      now: 'Plan on our own scan. One course file: the overnight maze rebuild was a new route, not new code',
      dropped: 'Floor tape: fixes sideways offset, not progress along the line',
    },
    {
      n: '02',
      name: 'The tunnel',
      tried: 'Freeze when tracking drops',
      found: `Hit the tube ${N.freeze.value} times in simulation`,
      now: `The route runs down the middle of the tube. When the camera goes blind, ride PX4’s estimate for up to ${N.blindCarry.value.split(';')[0]}. Simulation: landed 4 of 4, 6 clean passes`,
      dropped: `Built, not flown: side-ToF centring (bench ${N.tof.value}) and stereo centring${TODAY.tofReason ? ` · ${TODAY.tofReason}` : ''}`,
    },
    {
      n: '03',
      name: 'The compass',
      tried: 'Keep the default PX4 parameters, guard in our own code',
      found: `On the floor the compass read ${N.compass.value} off and kept flipping heading`,
      now: TODAY.compassOff ? 'Compass off in PX4 (EKF2_MAG_TYPE 5). Heading from the camera alone' : 'Guard stays; takeoff refuses when heading disagrees',
      dropped: TODAY.compassOff ? 'Goes back to 0 before any outdoor flight' : '',
    },
  ];
  return (
    <div className="absolute left-[96px] top-[262px] flex w-[1728px] gap-[24px]">
      {cards.map((c, i) => (
        <Reveal key={c.n} at={i} step={step} className="t-panel relative flex-1 p-[28px]" style={{ height: 660 }}>
          <Label tone="gold">
            {c.n} · {c.name}
          </Label>
          <div className="mt-5">
            <Label>We tried</Label>
            <div className="mt-1 text-[28px] leading-[1.2]">{c.tried}</div>
          </div>
          <div className="mt-6">
            <Label tone="amber">What happened</Label>
            <div className="mt-1 text-[26px] leading-[1.25] text-ink/90">{c.found}</div>
          </div>
          <div className="mt-6">
            <Label tone="ice">Now</Label>
            <div className="mt-1 text-[26px] leading-[1.25] text-ink">{c.now}</div>
          </div>
          {c.dropped ? <div className="absolute inset-x-[28px] bottom-[24px] border-t border-white/12 pt-3 text-[21px] leading-[1.25] text-ink/55">{c.dropped}</div> : null}
        </Reveal>
      ))}
    </div>
  );
}

/* ───────────────────────────── 4 · generalizes vs tuned ───────────────────────────── */
const GENERAL: [string, string][] = [
  ['Stereo odometry into PX4', 'needs light and texture, not our course'],
  ['The odometry gate', 'tracking, and not stale'],
  ['The state machine and its guards', 'pre-arm checks, heading guard, freeze, hold'],
  ['The planner', 'grid, A*, B-spline: any map'],
  ['Blind carry', 'rides the autopilot through any short vision gap'],
  ['The flight recorder', 'every tick of every run'],
];
const TUNED: [string, string][] = [
  ['The as-built map', 'this net and this maze: MN, MD, MT'],
  ['The route', 'north-east off one launch mark'],
  [`Takeoff lean ${N.lean.value}`, 'cancels this airframe’s liftoff slide'],
  [`Height offset ${N.heightOffset.value}`, 'this Pixhawk’s floor reading'],
  [`Net margin ${N.netClamp.value}, blind carry ${N.blindCarry.value.split(';')[0]}`, `sized for this net and a ${N.blind.value} tunnel`],
  TODAY.compassOff ? ['Compass off', 'this room’s magnetics'] : ['Pre-arm heading check', 'tuned to this room’s compass'],
];

function Column({ title, tone, rows, at, step, note }: { title: string; tone: 'ice' | 'gold'; rows: [string, string][]; at: number; step: number; note: string }) {
  return (
    <Reveal at={at} step={step} className="t-panel flex-1 px-[30px] py-[26px]" style={{ height: 520 }}>
      <Label tone={tone}>{title}</Label>
      <div className="mt-1 text-[22px] text-ink/55">{note}</div>
      <div className="mt-4 space-y-[11px]">
        {rows.map(([a, b]) => (
          <div key={a} className="leading-[1.18]">
            <div className="text-[28px]">{a}</div>
            <div className="text-[22px] text-ink/60">{b}</div>
          </div>
        ))}
      </div>
    </Reveal>
  );
}

function General({ step }: SlideProps) {
  return (
    <div className="absolute left-[96px] top-[262px] w-[1728px]">
      <div className="flex gap-[24px]">
        <Column title="Generalizes" tone="ice" rows={GENERAL} at={0} step={step} note="works in another room with light and texture" />
        <Column title="Tuned to this course" tone="gold" rows={TUNED} at={1} step={step} note="redo it if the course, the drone or the room changes" />
      </div>
      <Reveal at={2} step={step} className="mt-8 max-w-[1500px] text-[30px] leading-[1.3]">
        <span className="text-gold">The line:</span> if it needs this room&rsquo;s map, this airframe or this launch mark, it is tuned. If it needs only a textured room and a stereo pair, it generalizes.
      </Reveal>
    </div>
  );
}

/* ───────────────────────────── 5 · what breaks it ───────────────────────────── */
interface BreakRow {
  what: string;
  notices: string;
  does: string;
  open?: boolean;
}
const BREAKS: BreakRow[] = [
  { what: 'Camera loses tracking, or the light goes', notices: `Gate: not tracking, or pose older than ${N.stale.value}`, does: `Freeze the route target. Hold after ${N.noProgress.value} with no progress` },
  { what: `Dark tunnel, about ${N.blind.value} blind`, notices: 'The map says we are inside the tube', does: `Ride PX4’s estimate up to ${N.blindCarry.value.split(';')[0]} along the planned centreline` },
  { what: 'Heading disagrees', notices: 'PX4 vs camera over 15°, or PX4 resets its heading', does: 'Stop steering the nose. Takeoff needs them within 10°' },
  { what: 'Bad start', notices: 'Height off by over 30 cm, or heading off by over 10°', does: 'Refuse: stream nothing, no takeoff. It did, overnight (run 3)' },
  { what: 'Drift, or stuck off the route', notices: 'A steady push while holding; no progress', does: `Learn the push. Keep setpoints ${N.netClamp.value} inside the net. HOLD at ${N.noProgress.value}` },
  { what: 'Odometry jump', notices: 'Not yet: the latch is on a branch, not in the flight build', does: `Today it flies on. Run 2 kept going 20 s after the pose jumped ${N.jump.value}`, open: true },
];

function Breaks({ step }: SlideProps) {
  return (
    <div className="absolute left-[96px] top-[258px] w-[1728px]">
      <div className="grid grid-cols-[470px_540px_1fr] gap-x-[16px] px-[20px] pb-3">
        <Label>What breaks</Label>
        <Label tone="ice">How it notices</Label>
        <Label tone="gold">What it does</Label>
      </div>
      <div className="space-y-[8px]">
        {BREAKS.map((r, i) => (
          <Reveal key={r.what} at={Math.floor(i / 2)} step={step}>
            <div className="t-panel grid grid-cols-[470px_540px_1fr] items-center gap-x-[16px] px-[20px] py-[11px]" style={{ minHeight: 88, borderColor: r.open ? 'rgb(242 178 74 / 0.7)' : undefined }}>
              <div className={`font-display text-[27px] leading-[1.02] ${r.open ? 'text-amber' : ''}`}>{r.what}</div>
              <div className="text-[23px] leading-[1.22] text-ink/85">{r.notices}</div>
              <div className="text-[23px] leading-[1.22] text-ink">{r.does}</div>
            </div>
          </Reveal>
        ))}
      </div>
      <Reveal at={2} step={step} className="mt-5 flex items-baseline justify-between text-[24px] text-ink/70">
        <span>
          <span className="text-gold">When it gives up:</span> hold or land. It never arms. Any mode switch on the transmitter hands control back to the safety pilot.
        </span>
        <span className="t-label shrink-0 pl-8">L · break it live</span>
      </Reveal>
    </div>
  );
}

/* ───────────────────────────── 6 · evidence ───────────────────────────── */
const KIND_TEXT = {
  belief: ['Belief wrong from the start', 'start placement and heading'],
  motion: ['Belief right, motion wrong', 'the liftoff slide, the turn'],
} as const;

function Stat({ big, small }: { big: string; small: string }) {
  return (
    <div className="flex-1 border-l border-white/15 pl-4">
      <div className="font-display text-[54px] leading-none text-gold">{big}</div>
      <div className="mt-1 text-[20px] uppercase tracking-[0.12em] text-ink/60">{small}</div>
    </div>
  );
}

function Evidence({ step, setStep }: SlideProps) {
  const runs = EVIDENCE.runs;
  const idx = step >= 1 && step <= 4 ? step - 1 : null;
  const run = idx !== null ? runs[idx] : null;
  const note = run ? RUN_NOTES[run.id] : null;
  const others = idx === null ? runs : [];

  return (
    <>
      <div className="t-panel absolute left-[96px] top-[246px] overflow-hidden" style={{ width: 1000, height: 730, background: 'rgb(14 8 6 / 0.88)' }}>
        <CourseMap view={NORTH} run={run} note={note} others={others} checkpoints="faint" className="h-full w-full" />
        <div className="absolute bottom-3 left-4 flex items-center gap-5 text-[18px] text-ink/65">
          <span className="flex items-center gap-2"><i className="inline-block h-[3px] w-[26px] bg-ice" /> believed (cuVSLAM)</span>
          <span className="flex items-center gap-2"><i className="inline-block h-[3px] w-[26px] border-t-[3px] border-dotted border-dust" /> setpoints sent</span>
          <span className="flex items-center gap-2"><i className="inline-block h-[3px] w-[26px] bg-danger" /> what it touched</span>
        </div>
      </div>

      <div className="absolute right-[96px] top-[246px] flex w-[690px] flex-col" style={{ height: 730 }}>
        <div className="flex gap-3">
          {runs.map((r, i) => (
            <button key={r.id} type="button" onClick={() => setStep?.(i + 1)} className={`pointer-events-auto border px-4 py-2 font-mono text-[21px] transition-colors ${idx === i ? 'border-gold bg-gold/15 text-ink' : 'border-white/20 text-ink/65 hover:text-ink'}`}>
              {r.label}
            </button>
          ))}
        </div>

        <div className="mt-5 min-h-0 flex-1">
          {run && note ? (
            <div key={run.id} className="talk-slide">
              <Label tone="gold">
                {run.label} · {run.clock}
              </Label>
              <div className="mt-2 font-display text-[44px] leading-[1.02]">{run.outcome}</div>
              <div className="mt-5">
                <Label tone="ice">It believed</Label>
                <div className="mt-1 text-[26px] leading-[1.22]">{note.believed}</div>
              </div>
              <div className="mt-4">
                <Label>What happened</Label>
                <div className="mt-1 text-[26px] leading-[1.22]">{note.happened}</div>
              </div>
              <div className="mt-4 text-[23px] leading-[1.28] text-ink/65">{run.cause}</div>
              <div className="mt-4 font-mono text-[21px] text-ink/60">
                +{run.tContact.toFixed(1)} s after takeoff · {run.inM.toFixed(1)} m from the mark · {run.code.split(' ')[0]}
              </div>
            </div>
          ) : step === 5 ? (
            <div className="talk-slide space-y-4">
              {(['belief', 'motion'] as const).map((k) => (
                <div key={k} className="t-panel px-5 py-4">
                  <Label tone={k === 'belief' ? 'ice' : 'amber'}>{KIND_TEXT[k][0]}</Label>
                  <div className="mt-1 text-[26px] leading-[1.2]">
                    {runs.filter((r) => RUN_NOTES[r.id].kind === k).map((r) => r.label).join(' and ')}: {KIND_TEXT[k][1]}
                  </div>
                </div>
              ))}
              <div className="text-[24px] leading-[1.28] text-ink/80">
                In every run the camera’s track stayed smooth until the touch. Simulation, route-file build: <span className="text-gold">{N.simLaps.value}</span>, 0 contacts.
              </div>
              {TODAY.scored.length ? (
                <div className="t-panel px-5 py-4">
                  <Label tone="gold">Scored runs</Label>
                  {TODAY.scored.map((r) => (
                    <div key={r.slot} className="mt-1 text-[25px] leading-[1.22]">
                      <span className="text-gold">{r.slot}</span> · {r.result}
                    </div>
                  ))}
                </div>
              ) : null}
            </div>
          ) : (
            <div className="talk-slide">
              <Label tone="gold">The first course runs, overnight</Label>
              <div className="mt-3 text-[28px] leading-[1.3] text-ink/90">
                Every tick of every flight is logged. Pick a run to replay what the drone believed.
              </div>
              <div className="mt-4 text-[22px] leading-[1.3] text-ink/60">{RUN_3}</div>
            </div>
          )}
        </div>

        <div className="flex gap-3 border-t border-white/12 pt-5">
          <Stat big="5" small="attempts" />
          <Stat big="4" small="contacts" />
          <Stat big="1" small="refused" />
          <Stat big="0" small="finished" />
        </div>
      </div>
    </>
  );
}

/* ───────────────────────────── 7 · hardest problem, next day, team ───────────────────────────── */
const BARS: { k: string; lo: number; hi: number; room?: boolean }[] = [
  { k: 'Planned prop room, tightest wall', lo: 17.9, hi: 17.9, room: true },
  { k: 'Wide on the turn, run 2', lo: 18, hi: 19 },
  { k: 'Start heading off ~12°, 1.5 m in', lo: 32, hi: 32 },
  { k: 'Liftoff slide', lo: 25, hi: 40 },
  { k: 'Start pose off, run 5', lo: 40, hi: 40 },
];
const SCALE = 11; // px per cm

function Close({ step }: SlideProps) {
  const dayTwo = TODAY.localizerSteering ? 'Validate the localizer on more real flights' : 'Validate the localizer on the real drone: shadow first, then bounded correction';
  const days = ['Odometry-jump protection in the flight build: a bad pose means hold, not fly on', dayTwo, `More room at the tip of MD (path leaves ${N.mdTip.value}) and slower into that turn`];
  return (
    <div className="absolute left-[96px] top-[262px] flex w-[1728px] gap-[24px]">
      <Reveal at={0} step={step} className="t-panel flex-[1.15] p-[28px]" style={{ height: 650 }}>
        <Label tone="gold">The hardest problem</Label>
        <div className="mt-2 text-[26px] leading-[1.22] text-ink/80">
          Room we plan for, against the error we measure (cm)
        </div>
        <div className="relative mt-7 space-y-[24px]">
          <div className="absolute inset-y-0" style={{ left: 17.9 * SCALE, borderLeft: '2px dashed rgb(241 220 192 / 0.55)' }} />
          {BARS.map((b) => (
            <div key={b.k} className="relative">
              <div className="text-[22px] text-ink/85">{b.k}</div>
              <div className="relative mt-1 h-[22px]" style={{ width: 45 * SCALE }}>
                <div className={`absolute left-0 top-0 h-full ${b.room ? 'bg-sand/80' : 'bg-amber'}`} style={{ width: b.lo * SCALE, opacity: b.room || b.lo === b.hi ? 1 : 0.4 }} />
                {b.hi > b.lo ? <div className="absolute top-0 h-full bg-amber" style={{ left: b.lo * SCALE, width: (b.hi - b.lo) * SCALE }} /> : null}
                <span className="absolute top-[-1px] font-mono text-[20px] text-ink" style={{ left: b.hi * SCALE + 10 }}>
                  {b.lo === b.hi ? b.lo : `${b.lo} to ${b.hi}`}
                </span>
              </div>
            </div>
          ))}
        </div>
        <div className="mt-6 text-[22px] leading-[1.25] text-ink/60">
          Sim gate: a 10 cm, 3° start error leaves {N.margin.value}.
        </div>
      </Reveal>

      <Reveal at={1} step={step} className="t-panel flex-1 p-[28px]" style={{ height: 650 }}>
        <Label tone="ice">With one more day</Label>
        <div className="mt-5 space-y-[26px]">
          {days.map((d, i) => (
            <div key={d} className="flex gap-4">
              <span className="font-display text-[44px] leading-none text-gold">{i + 1}</span>
              <span className="text-[26px] leading-[1.25]">{d}</span>
            </div>
          ))}
        </div>
      </Reveal>

      <Reveal at={2} step={step} className="t-panel relative flex-1 p-[28px]" style={{ height: 650 }}>
        <Label tone="gold">Who did what</Label>
        <div className="mt-5 space-y-[18px]">
          {TEAM.map((m) => (
            <div key={m.name} className="leading-[1.18]">
              <div className="text-[27px]">{m.name}</div>
              <div className="text-[21px] text-ink/60">{m.does}</div>
            </div>
          ))}
        </div>
        <div className="absolute inset-x-[28px] bottom-[22px] text-[21px] text-ink/50">Any of us can be asked about any part.</div>
      </Reveal>
    </div>
  );
}

export const VISUALS: Record<SlideId, ComponentType<SlideProps>> = {
  title: Title,
  problem: Problem,
  system: System,
  approach: Approach,
  general: General,
  breaks: Breaks,
  evidence: Evidence,
  close: Close,
};
