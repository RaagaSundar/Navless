/**
 * telemetry.ts: the stream behind <TelemetryHUD />.
 *
 * Every line is SIMULATED (the HUD header says so), but nothing in it is made up out of thin air: the subsystems, node names,
 * parameters and failure modes come from this repo and from skybeta-19's real bench and flight logs:
 *
 *   cuVSLAM 31.2 Hz, max gap 49 ms          ground dry run on 19, 2026-11-08
 *   odom_gate 28.43 Hz out, 59.89 ms P50    hardware_slam/README.md (bench, PX4 powered)
 *   Height offset 19 cm                     PX4 height read 18.5-19.8 cm above cuVSLAM in all three 11-07 logs (fa7d2d3)
 *   mag heading reset ~100 deg              PX4 jumps to the magnetometer ~0.5 s into a vision gap (seen on 19, 11-07)
 *   blind_carry <= 4 s, tube ToF 9.6 Hz     navless_nav / tube/ (README)
 *   heading_guard 15 deg                    PX4-vs-camera disagreement limit (course_mission)
 *
 * The numbers wobble around those anchors. Planner lines (A* expansions, EGO iterations, clearances) are NOT from here:
 * they are measured from the real planner run in lib/planner.ts and queued by the store.
 */
import type { LogLine, LogTag } from './planner';

export type TelemetryCtx = 'boot' | 'hero' | 'tour' | 'tube' | 'override';

const L = (tag: LogTag, text: string, level: LogLine['level'] = 'info'): LogLine => ({ t: 0, tag, text, level });
const w = (t: number, f: number, a: number) => a * (Math.sin(t * f) * 0.6 + Math.sin(t * f * 2.31 + 1.7) * 0.4);
const f1 = (v: number) => v.toFixed(1);
const f2 = (v: number) => v.toFixed(2);
const n0 = (v: number) => Math.round(v).toLocaleString('en-US');

type Gen = (t: number) => LogLine;

const COMMON: Gen[] = [
  (t) => L('VIO', `cuvslam tracking ok | ${f1(31.2 + w(t, 1.1, 0.25))} Hz | gap_max 49 ms | vo_state 1`, 'ok'),
  (t) => L('GATE', `odom_gate pass | age ${f1(59.9 + w(t, 0.9, 2.6))} ms | out ${f1(28.4 + w(t, 1.3, 0.3))} Hz`),
  (t) => L('CAM', `oak-d-s2 stereo 640x400@30 | depth valid ${f1(94.1 + w(t, 0.5, 1.4))} % | features ${n0(1284 + w(t, 0.7, 60))}`),
  (t) => L('EKF2', `ext-vision fused | pos innov ${(0.02 + Math.abs(w(t, 0.8, 0.012))).toFixed(3)} m | yaw err ${f1(1.1 + Math.abs(w(t, 0.6, 0.6)))} deg`, 'dim'),
  (t) => L('HEAD', `heading_guard px4-vs-camera ${f1(2.8 + Math.abs(w(t, 0.7, 1.4)))} deg (limit 15) ok`, 'dim'),
  (t) => L('SYS', `jetson orin nano | gpu ${Math.round(58 + w(t, 0.4, 7))} % cpu ${Math.round(34 + w(t, 0.6, 6))} % | ${Math.round(52 + w(t, 0.2, 2))} C (sim)`, 'dim'),
  (t) => L('SYS', 'ros2 humble | domain 219 | 14 nodes | /fmu/out/vehicle_local_position 100 Hz', 'dim'),
];

const HERO: Gen[] = [
  ...COMMON,
  () => L('PX4', 'Height offset: 19cm (px4 z above cuvslam z) | pre-arm limit 30 cm | PASS', 'ok'),
  (t) => L('NAV', `course_mission state=HOVER | route 286 wp 14.2 m | heading_mode=travel | cruise ${f1(0.5)} m/s`),
  () => L('MAP', 'esdf 61x61 @ 0.1 m | walls inflated 0.28 m | as-built 2026-11-07'),
  (t) => L('MAG', `ekf2 mag_type default | heading ${f1(((t * 3) % 360))} deg | vision yaw agree`, 'dim'),
];

const TOUR: Gen[] = [
  ...COMMON,
  (t) => L('A*', `A* voxel expansion: ${Math.round(38 + w(t, 0.9, 9))}ms | open ${n0(1284 + w(t, 0.5, 220))} closed ${n0(3901 + w(t, 0.6, 400))}`),
  (t) => L('EGO', `EGO B-spline recalculating | ctrl ${Math.round(41 + w(t, 0.4, 3))} | J_s ${f2(0.84 + w(t, 0.5, 0.12))} J_c ${f2(0.31 + Math.abs(w(t, 0.7, 0.1)))}`),
  (t) => L('NAV', `carrot ${f2(0.5 + w(t, 0.6, 0.04))} m ahead | setpoint err ${f1(4 + Math.abs(w(t, 0.8, 3)))} cm | cruise 0.5 m/s`),
  () => L('PX4', 'Height offset: 19cm | course_mission adds it to every height it sends', 'dim'),
  (t) => L('WALL', `wall_correction off | cloud_localize median ${f1(4.6 + w(t, 0.4, 0.5))} cm (sim route)`, 'dim'),
];

const TUBE: Gen[] = [
  (t) => L('TUBE', `tof L ${f2(0.74 + w(t, 2.1, 0.012))} m R ${f2(0.72 + w(t, 1.7, 0.012))} m | sum ${f2(1.46)} m ok | 9.6 Hz | valid 99.4 %`),
  (t) => L('TUBE', `offset ${(w(t, 1.2, 1.4)).toFixed(1)} cm | correction clamp 15 cm | route target on centre`, 'dim'),
  (t) => L('BLIND', `blind_carry ${f1(1.2 + ((t * 0.2) % 2.8))} s of 4.0 s | px4 estimate`, 'warn'),
  (t) => L('GATE', `odom_gate hold | pose stale ${Math.round(180 + ((t * 40) % 120))} ms > 0.25 s limit`, 'warn'),
];

const OVERRIDE: Gen[] = [
  (t) => L('VIO', `cuvslam tracking ok | ${f1(31.2 + w(t, 1.1, 0.25))} Hz | vo_state 1`, 'dim'),
  (t) => L('MAP', `esdf age ${Math.round(40 + w(t, 2.1, 20))} ms | dynamic cells live`, 'dim'),
  (t) => L('NAV', 'course_mission state=HOVER | awaiting ENGAGE or obstacle', 'dim'),
  (t) => L('SYS', `jetson orin nano | gpu ${Math.round(58 + w(t, 0.4, 7))} % | ${Math.round(52 + w(t, 0.2, 2))} C (sim)`, 'dim'),
];

const POOLS: Record<TelemetryCtx, Gen[]> = { boot: HERO, hero: HERO, tour: TOUR, tube: TUBE, override: OVERRIDE };
const RATE: Record<TelemetryCtx, number> = { boot: 5, hero: 4.2, tour: 5.2, tube: 6, override: 1.6 };

/** one-off sequences that play when a context begins: the real launch order, and the real dark-tunnel failure mode */
const SEQUENCES: Partial<Record<TelemetryCtx, [number, LogLine][]>> = {
  hero: [
    [0, L('SYS', '$ ros2 launch navless_nav navless_course.launch.py', 'warn')],
    [220, L('NAV', 'course_mission: heading_mode=travel route_source=file (286 wp, 14.2 m)')],
    [440, L('VIO', 'cuvslam init: stereo only, imu off, odom_gate armed (pass while tracking, age < 0.25 s)')],
    [660, L('CAM', 'oak-d-s2 online: 2 x mono 75 mm baseline, stereo depth 30 Hz', 'ok')],
    [880, L('PX4', 'Height offset: 19cm | pre-arm limit 30 cm | PASS', 'ok')],
    [1100, L('TUBE', '2 x VL53L1X on separate i2c buses | 9.6 Hz | valid 99+ %', 'ok')],
    [1320, L('MAP', 'as-built map: M1-M3 removed, MN MD MT added (2026-11-07)', 'dim')],
  ],
  tube: [
    [0, L('CAM', 'OAK-D-S2 depth dropped | tube is dark, nothing to track', 'err')],
    [380, L('VIO', 'cuvslam vo_state 0 | tracking lost | odom_gate holds output', 'err')],
    [760, L('BLIND', 'blind_carry start | px4 estimate | max 4.0 s', 'warn')],
    [1150, L('TUBE', 'tof L 0.74 m R 0.72 m | sum 1.46 m = tube width | using it', 'ok')],
    [1550, L('TUBE', 'offset +1.0 cm | route target shifted onto the tube centre', 'ok')],
    [4300, L('VIO', 'cuvslam tracking recovered | vo_state 1 | blind_carry 2.9 s', 'ok')],
    [4700, L('MAG', 'ekf2: no heading reset (mag_type default) | seen +102 deg on 19 under load, 11-07', 'dim')],
  ],
};

export interface TelemetryState {
  ctx: TelemetryCtx | null;
  acc: number;
  last: number;
  pending: { at: number; line: LogLine }[];
}
export const createTelemetry = (): TelemetryState => ({ ctx: null, acc: 0, last: 0, pending: [] });

/** advance the stream; returns new lines (0..n) for the log queue */
export function stepTelemetry(s: TelemetryState, now: number, ctx: TelemetryCtx): LogLine[] {
  const out: LogLine[] = [];
  const dt = s.last ? Math.min(0.25, (now - s.last) / 1000) : 0;
  s.last = now;

  if (ctx !== s.ctx) {
    s.ctx = ctx;
    const seq = SEQUENCES[ctx];
    if (seq) for (const [d, line] of seq) s.pending.push({ at: now + d, line });
    s.acc = 0;
  }
  for (let i = s.pending.length - 1; i >= 0; i--) {
    if (s.pending[i].at <= now) out.push(...s.pending.splice(i, 1).map((p) => p.line));
  }
  // keep chronological order inside one frame
  out.reverse();

  s.acc += dt * RATE[ctx];
  const pool = POOLS[ctx];
  while (s.acc >= 1) {
    s.acc -= 1;
    out.push(pool[Math.floor(Math.random() * pool.length)](now / 1000));
  }
  return out;
}

/** which telemetry context a point on the timeline belongs to */
export function ctxOf(p: number, override: boolean): TelemetryCtx {
  if (override) return 'override';
  if (p < 0.02) return 'boot';
  if (p < 0.37) return 'hero';
  if (p >= 0.8 && p < 0.9) return 'tube';
  return 'tour';
}
