/**
 * replay.ts: REPLAY THE REAL FLIGHTS.
 *
 * In OVERRIDE MODE a judge can replay what skybeta-19's flight recorder logged on the first course runs: the demo drone flies the
 * track the real drone BELIEVED (cuVSLAM odometry, one point per 0.1 s tick, from src/data/evidence.json), a chase camera follows,
 * and the replay stops at the first impact the log shows. Nothing here is simulated: positions, heights and headings are the
 * recorder's. (The planner demo next to it is a simulation; this is not.)
 *
 * `replay` holds the per-frame state (read every frame by R3F, never React-rendered); `useReplay` holds what the DOM needs.
 */
import { create } from 'zustand';
import { EVIDENCE } from './evidence';
import { flight, useDeck } from './store';
import { camOverride } from './choreography';

export const LEAD_S = 1.2;

export const replay = {
  active: false,
  run: -1,
  /** seconds since takeoff in the log; starts at -LEAD_S so the drone sits on the mark for a moment first */
  t: 0,
  hit: false,
};

interface ReplayState {
  run: number;
  /** the replay has reached the impact */
  hit: boolean;
}
export const useReplay = create<ReplayState>(() => ({ run: -1, hit: false }));

export const runCount = () => EVIDENCE.runs.length;

export function startReplay(i: number) {
  if (i < 0 || i >= EVIDENCE.runs.length) return;
  const st = useDeck.getState();
  st.setArmed(false);
  if (st.obstacles.length) st.clear(); // the real course had no blockers dropped on it
  replay.active = true;
  replay.run = i;
  replay.t = -LEAD_S;
  replay.hit = false;
  useReplay.setState({ run: i, hit: false });
}

export function stopReplay() {
  if (!replay.active) return;
  replay.active = false;
  replay.run = -1;
  replay.hit = false;
  camOverride.on = false;
  useReplay.setState({ run: -1, hit: false });
  useDeck.getState().resetFlight(); // park the demo drone on the launch mark and re-plan
}

/** the believed pose at replay time t (seconds since takeoff): linear between the recorder's 0.1 s ticks */
export function sampleRun(i: number, t: number) {
  const P = EVIDENCE.runs[i].points;
  const tt = Math.max(0, Math.min(t, P[P.length - 1][0] as number));
  let k = 0;
  while (k < P.length - 2 && (P[k + 1][0] as number) <= tt) k++;
  const a = P[k];
  const b = P[k + 1];
  const span = (b[0] as number) - (a[0] as number) || 1;
  const u = Math.max(0, Math.min(1, (tt - (a[0] as number)) / span));
  const L = (j: number) => (a[j] as number) + ((b[j] as number) - (a[j] as number)) * u;
  // heading: shortest way round between the two yaws (degrees)
  let dy = (b[7] as number) - (a[7] as number);
  dy = ((dy + 540) % 360) - 180;
  const yaw = (a[7] as number) + dy * u;
  return { x: L(1), y: L(2), z: L(5), heading: Math.PI / 2 + (yaw * Math.PI) / 180, vx: ((b[1] as number) - (a[1] as number)) / span, vy: ((b[2] as number) - (a[2] as number)) / span };
}

export { flight };
