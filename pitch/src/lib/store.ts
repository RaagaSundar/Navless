/**
 * store.ts: shared UI + planner state.
 *
 * React-visible state lives in the zustand store (changes rarely: a drop, a replan, a log batch).
 * Per-frame state (the live drone pose while flying) lives in the plain `flight` object: R3F and the
 * telemetry readouts read it every frame without ever touching React.
 */
import { create } from 'zustand';
import { kick } from './choreography';
import { COURSE, GUIDES, START, VIA, routeAt } from './course';
import type { Vec2 } from './course-types';
import {
  DEFAULT_CONFIG,
  occupiedAt,
  planClearanceAgainst,
  planMission,
  type DynObstacle,
  type LogLine,
  type LogTag,
  type PlanResult,
} from './planner';

/* ───────────────────────────── types ───────────────────────────── */

export interface Obstacle extends DynObstacle {
  /** performance.now() when it hit the floor, drives the drop animation */
  born: number;
}
export interface TermLine extends LogLine {
  id: number;
  stamp: string;
}
export type PlanStatus = 'nominal' | 'replanned' | 'sealed';
export type FlightMode = 'idle' | 'flying' | 'landed' | 'hold';
export type Verdict = 'ok' | 'seal' | 'wall' | 'dup';

/** live, per-frame pose of the demo drone. Mutated by InteractivePlanner; read everywhere. */
export const flight = {
  mode: 'idle' as FlightMode,
  x: START[0],
  y: START[1],
  /** height above the floor, m */
  z: 0.45,
  /** nose direction (ENU radians) */
  heading: routeAt(0).heading,
  /** arc length travelled on the active plan, m */
  s: 0,
  speed: 0,
  /** index into VIA of the next checkpoint (for mid-flight replans) */
  nextVia: 0,
  /** checkpoints passed (0..5) for the HUD */
  checkpoints: 0,
};

/* ───────────────────────────── log queue ───────────────────────────── */

let lineSeq = 0;
const queue: TermLine[] = [];
let sessionStart = 0;

const stampOf = (sec: number) => `+${sec.toFixed(3).padStart(7, '0')}`;

export function enqueueLogs(lines: LogLine[]) {
  if (!sessionStart) sessionStart = performance.now();
  const base = (performance.now() - sessionStart) / 1000;
  for (const l of lines) queue.push({ ...l, id: ++lineSeq, stamp: stampOf(base + l.t / 1000) });
}
export function log(tag: LogTag, text: string, level: LogLine['level'] = 'info') {
  enqueueLogs([{ t: 0, tag, text, level }]);
}
export const drainLogs = (n: number) => queue.splice(0, n);
export const clearLogQueue = () => {
  queue.length = 0;
};
export const queuedLogs = () => queue.length;

/* ───────────────────────────── store ───────────────────────────── */

const snap = (v: number, g = 0.2) => Math.round(v / g) * g;
const OBSTACLE_HALF = 0.2; // 0.4 m column

interface DeckState {
  ready: boolean;
  override: boolean;
  armed: boolean;
  obstacles: Obstacle[];
  plan: PlanResult | null;
  failed: PlanResult | null;
  prevPlan: PlanResult | null;
  planStatus: PlanStatus;
  /** bumps on every replan: scenes key their draw-in animations on it */
  planEpoch: number;
  planStamp: number;
  flightMode: FlightMode;
  hover: { x: number; y: number; verdict: Verdict; delta?: number } | null;
  lines: TermLine[];
  quality: 'high' | 'medium' | 'low';
  /** ?q=high|medium|low pins the tier and turns the PerformanceMonitor off */
  qualityLocked: boolean;
  /** the AutoDirector is running the 2-minute demo */
  auto: boolean;
  paused: boolean;

  setAuto: (v: boolean) => void;
  setPaused: (v: boolean) => void;
  setReady: () => void;
  setQuality: (q: 'high' | 'medium' | 'low') => void;
  setOverride: (v: boolean) => void;
  setArmed: (v: boolean) => void;
  setHover: (h: DeckState['hover']) => void;
  appendLines: (l: TermLine[]) => void;
  clearLines: () => void;
  initPlan: () => void;
  drop: (x: number, y: number) => 'ok' | 'wall' | 'dup';
  dropOnPath: () => void;
  undo: () => void;
  clear: () => void;
  engage: () => void;
  resetFlight: () => void;
  setFlightMode: (m: FlightMode) => void;
}

const CAP = 120;

export const useDeck = create<DeckState>((set, get) => {
  /** run the planner against the current obstacle list from the drone's live pose */
  const replan = (list: Obstacle[], event: Parameters<typeof planMission>[5]) => {
    const flying = flight.mode === 'flying' || flight.mode === 'hold';
    const from: Vec2 = flying ? [flight.x, flight.y] : START;
    const via = VIA.slice(flying ? flight.nextVia : 0);
    return planMission(COURSE, via.length ? via : VIA.slice(-1), list, from, DEFAULT_CONFIG, event, GUIDES);
  };

  const apply = (list: Obstacle[], res: PlanResult, kind: 'inject' | 'clear' | 'replan') => {
    const s = get();
    const now = performance.now();
    if (res.ok) {
      set({
        obstacles: list,
        plan: res,
        failed: null,
        prevPlan: s.plan,
        planStatus: list.length ? 'replanned' : 'nominal',
        planEpoch: s.planEpoch + 1,
        planStamp: now,
      });
      if (flight.mode === 'hold') {
        flight.mode = 'flying';
        flight.s = 0;
        set({ flightMode: 'flying' });
      } else if (flight.mode === 'flying') {
        flight.s = 0; // the new plan starts at the live pose
      }
    } else {
      set({ obstacles: list, failed: res, prevPlan: s.plan, planStatus: 'sealed', planEpoch: s.planEpoch + 1, planStamp: now });
      if (flight.mode === 'flying') {
        flight.mode = 'hold';
        flight.speed = 0;
        set({ flightMode: 'hold' });
      }
    }
    void kind;
  };

  return {
    ready: false,
    override: false,
    armed: false,
    obstacles: [],
    plan: null,
    failed: null,
    prevPlan: null,
    planStatus: 'nominal',
    planEpoch: 0,
    planStamp: 0,
    flightMode: 'idle',
    hover: null,
    lines: [],
    quality: 'high',
    qualityLocked: false,
    auto: false,
    paused: false,

    setAuto: (auto) => set({ auto, paused: false }),
    setPaused: (paused) => set({ paused }),
    setReady: () => set({ ready: true }),
    setQuality: (quality) => set({ quality }),
    setOverride: (override) => {
      if (override && !get().override) {
        enqueueLogs([
          { t: 0, tag: 'SYS', text: 'OVERRIDE MODE engaged: judge control. Planner is live.', level: 'warn' },
          { t: 0, tag: 'SYS', text: 'map = as-built course (drone19/flight), M1-M3 removed, MN/MD/MT in', level: 'dim' },
        ]);
      }
      set({ override, armed: override ? get().armed : false });
    },
    setArmed: (armed) => set({ armed }),
    setHover: (hover) => set({ hover }),
    appendLines: (l) => set((s) => ({ lines: [...s.lines, ...l].slice(-CAP) })),
    clearLines: () => set({ lines: [] }),
    setFlightMode: (flightMode) => set({ flightMode }),

    initPlan: () => {
      if (get().plan) return;
      if (!sessionStart) sessionStart = performance.now();
      const res = planMission(COURSE, VIA, [], START, DEFAULT_CONFIG, { kind: 'initial' }, GUIDES);
      enqueueLogs(res.log);
      set({ plan: res, planStamp: performance.now(), planEpoch: 1 });
    },

    drop: (x, y) => {
      const s = get();
      const sx = snap(x);
      const sy = snap(y);
      const occ = occupiedAt(COURSE, s.obstacles, sx, sy);
      if (occ === 'static') {
        log('MAP', `drop at (${sx.toFixed(2)}, ${sy.toFixed(2)}) is inside the static map: nothing to add`, 'warn');
        return 'wall';
      }
      if (occ === 'dynamic') return 'dup';
      const ob: Obstacle = { id: s.obstacles.length ? Math.max(...s.obstacles.map((o) => o.id)) + 1 : 1, x: sx, y: sy, hw: OBSTACLE_HALF, hd: OBSTACLE_HALF, born: performance.now() };
      const list = [...s.obstacles, ob];
      const prior = s.plan ? planClearanceAgainst(COURSE, s.plan, [ob]) : undefined;
      const res = replan(list, { kind: 'inject', obstacle: ob, priorMinClearance: prior });
      enqueueLogs(res.log);
      apply(list, res, 'inject');
      kick(0.25); // the big thud fires from the blocker when it lands
      return 'ok';
    },

    /** presenter shortcut: drop a blocker on the current path, somewhere a replan is feasible */
    dropOnPath: () => {
      const s = get();
      if (!s.plan || !s.plan.ok) return;
      const n = s.plan.samples.length / 2;
      // try spots along the plan from the middle outwards; keep the first that still has a route
      const order: number[] = [];
      for (let k = 0; k < 40; k++) {
        const f = 0.12 + (k / 39) * 0.8;
        order.push(Math.round(f * (n - 1)));
      }
      order.sort((a, b) => Math.abs(a - n * 0.45) - Math.abs(b - n * 0.45));
      for (const i of order) {
        const x = snap(s.plan.samples[2 * i]);
        const y = snap(s.plan.samples[2 * i + 1]);
        if (occupiedAt(COURSE, s.obstacles, x, y) !== 'free') continue;
        const probe: Obstacle = { id: 999, x, y, hw: OBSTACLE_HALF, hd: OBSTACLE_HALF, born: 0 };
        const test = replan([...s.obstacles, probe], { kind: 'inject', obstacle: probe });
        if (test.ok && !test.usedFallback) {
          get().drop(x, y);
          return;
        }
      }
      get().drop(snap(s.plan.samples[2 * Math.round(n * 0.3)]), snap(s.plan.samples[2 * Math.round(n * 0.3) + 1]));
    },

    undo: () => {
      const s = get();
      if (!s.obstacles.length) return;
      const list = s.obstacles.slice(0, -1);
      log('EVENT', `operator undo: obstacle #${s.obstacles[s.obstacles.length - 1].id} removed`, 'warn');
      const res = replan(list, { kind: list.length ? 'replan' : 'clear' });
      enqueueLogs(res.log);
      apply(list, res, 'clear');
    },

    clear: () => {
      const s = get();
      if (!s.obstacles.length) return;
      const res = replan([], { kind: 'clear' });
      enqueueLogs(res.log);
      apply([], res, 'clear');
    },

    engage: () => {
      const s = get();
      if (!s.plan || !s.plan.ok || s.planStatus === 'sealed') return;
      if (flight.mode === 'landed') {
        flight.x = START[0];
        flight.y = START[1];
        flight.z = 0.45;
        flight.nextVia = 0;
        flight.checkpoints = 0;
        const res = replan(s.obstacles, { kind: 'replan' });
        enqueueLogs(res.log);
        apply(s.obstacles, res, 'replan');
      }
      flight.mode = 'flying';
      flight.s = 0;
      set({ flightMode: 'flying' });
      log('FSM', 'ENGAGE: ARM -> OFFBOARD -> TRACK. Following the B-spline at cruise speed.', 'ok');
    },

    resetFlight: () => {
      flight.mode = 'idle';
      flight.x = START[0];
      flight.y = START[1];
      flight.z = 0.45;
      flight.s = 0;
      flight.speed = 0;
      flight.nextVia = 0;
      flight.checkpoints = 0;
      flight.heading = routeAt(0).heading;
      set({ flightMode: 'idle' });
      const s = get();
      const res = replan(s.obstacles, { kind: 'replan' });
      enqueueLogs(res.log);
      apply(s.obstacles, res, 'replan');
      log('FSM', 'drone returned to the launch mark; hover', 'dim');
    },
  };
});
