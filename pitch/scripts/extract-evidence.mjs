#!/usr/bin/env node
/**
 * extract-evidence.mjs
 *
 * Pulls skybeta-19's REAL course attempts out of git (read-only `git show`, nothing is checked out) and reduces each flight
 * recorder CSV to a small JSON the deck can plot: where the system BELIEVED it was (cuVSLAM odometry), where it told the
 * drone to go (the setpoints it sent), and when the contact happened. Writes src/data/evidence.json.
 *
 *   node scripts/extract-evidence.mjs                      (reads origin/flight-logs/2026-11-08)
 *   EVIDENCE_REF=flight-logs/2026-11-08 node scripts/extract-evidence.mjs
 *
 * Nothing here is invented: positions come straight from the recorder (one row per 0.1 s tick), converted to course
 * coordinates the same way docs/media/make_readme_media.py does it. The *outcome* of each run (what hit what, and our
 * reading of why) is copied from the notes in flight_logs/2026-11-08_skybeta19_course/README.md, which Shining wrote.
 */
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const pitch = path.resolve(here, '..');
const repo = path.resolve(pitch, '..');
const REF = process.env.EVIDENCE_REF || 'origin/flight-logs/2026-11-08';
const DIR = 'flight_logs/2026-11-08_skybeta19_course';
const STEP_M = 0.25; // metres per 0.1 s tick that no drone in a 1 m/s envelope can do
const YAW_DEG = 10; // degrees per tick (100 deg/s) that a held heading does not do
const GAP_DEG = 2; // PX4 heading vs camera heading: they agree within this until the airframe is knocked

const course = JSON.parse(readFileSync(path.join(pitch, 'src', 'data', 'course.json'), 'utf8'));
const lm = course.launch; // launch mark, course ENU metres

const git = (...args) => execFileSync('git', ['-C', repo, ...args], { maxBuffer: 256 * 1024 * 1024 }).toString('utf8');

/** minimal RFC-4180 CSV: the recorder quotes its log lines, which can contain commas */
function parseCsv(text) {
  const rows = [];
  let row = [];
  let cell = '';
  let q = false;
  for (let i = 0; i < text.length; i++) {
    const c = text[i];
    if (q) {
      if (c === '"' && text[i + 1] === '"') {
        cell += '"';
        i++;
      } else if (c === '"') q = false;
      else cell += c;
    } else if (c === '"') q = true;
    else if (c === ',') {
      row.push(cell);
      cell = '';
    } else if (c === '\n' || c === '\r') {
      if (c === '\r' && text[i + 1] === '\n') i++;
      row.push(cell);
      rows.push(row);
      row = [];
      cell = '';
    } else cell += c;
  }
  if (cell !== '' || row.length) {
    row.push(cell);
    rows.push(row);
  }
  return rows;
}

// outcome and cause: Shining's notes in the flight-logs README (2026-11-08), nothing added
const RUNS = [
  {
    id: '20261108-091401',
    label: 'Run 1',
    clock: '01:14 PST',
    code: 'nav-stabilize-travel-heading 2ffac5d',
    setup: 'route file, nose along travel, drift correction on, no localizer',
    outcome: 'Hit MD, 1.5 m in',
    cause: 'By odometry it was on its line, props about 32 cm clear. The real drone was closer: a ~12° start-heading error does exactly that.',
  },
  {
    id: '20261108-092321',
    label: 'Run 2',
    clock: '01:23 PST',
    code: 'nav-stabilize-travel-heading 2ffac5d',
    setup: 'same, armed further back than the launch mark',
    outcome: 'Hit the north net on the turn east',
    cause: 'Through the 1 m gap, then it swung 18 cm wide with 25 cm of lag, side-on to the net. After the hit the camera jumped 2.7 m while still "tracking".',
  },
  {
    id: '20261108-114602',
    label: 'Run 4',
    clock: '03:46 PST',
    code: 'main-localizer (planner + localizer, wall correction on)',
    setup: 'planner path, localizer on, compass on',
    outcome: 'Slid 34 cm left into the west net during the climb',
    cause: 'The liftoff push. The drift correction only learns above 0.4 m, and the planner takeoff leaned straight north. It never reached the route.',
  },
  {
    id: '20261108-133310',
    label: 'Run 5',
    clock: '05:33 PST',
    code: 'main-localizer (planner path, no localizer, compass off)',
    setup: 'planner path, re-planning off, takeoff lean 0.15 m, EKF2_MAG_TYPE 5',
    outcome: 'Hit the front wall about 1.9 m in',
    cause: 'By odometry it flew 40 cm clear of every wall, so the start pose was off by about 13° or 40 cm. It also slid 24 cm left at takeoff.',
  },
];

const out = { generatedBy: 'pitch/scripts/extract-evidence.mjs', ref: REF, refCommit: git('rev-parse', '--short', REF).trim(), launch: lm, runs: [] };

for (const meta of RUNS) {
  const csv = git('show', `${REF}:${DIR}/${meta.id}_course.csv`);
  const rows = parseCsv(csv);
  const head = rows[0];
  const col = (name) => head.indexOf(name);
  const need = ['kind', 't', 'state', 'x', 'y', 'sent_x', 'sent_y', 'vo_state'].map((n) => [n, col(n)]);
  const missing = need.filter(([, i]) => i < 0).map(([n]) => n);
  if (missing.length) throw new Error(`${meta.id}: columns missing: ${missing.join(', ')} in ${head.join(',')}`);
  const I = Object.fromEntries(need);
  const ticks = rows.slice(1).filter((r) => r[I.kind] === 'tick');
  const num = (v) => (v === '' || v === undefined ? NaN : Number(v));
  // believed track in course coordinates: the recorder's odom frame has x north, y west, origin on the launch mark
  const toCourse = (x, y) => [lm[0] - y, lm[1] + x];
  const iArmed = col('armed');
  const iOff = col('offboard');
  const iYaw = col('yaw_deg');
  const iZ = col('z');

  // `t` in the CSV is wall-clock epoch seconds; everything below is seconds since the log started
  const tLog0 = num(ticks[0][I.t]);
  const T = ticks.map((r) => num(r[I.t]) - tLog0);
  const iTakeoff = ticks.findIndex((r) => r[I.state] === 'TAKEOFF');
  if (iTakeoff < 0) throw new Error(`${meta.id}: the run never reached TAKEOFF`);
  const tTakeoff = T[iTakeoff];

  // The recorder keeps writing after a crash, with garbage odometry, so the end of the flight has to be found in the log.
  //  1. POSE JUMP: the first tick after takeoff where the camera pose does something the drone cannot do on its own,
  //     a step of more than 0.25 m in one 0.1 s tick (the envelope is 1 m/s, so 0.1 m) or a heading change of more than 10 deg.
  //  2. CONTACT: where that disturbance began. Walk back from the jump while PX4's heading and the camera's heading disagree
  //     by more than 2 deg. They agree within about that all flight, until the airframe is knocked.
  // Nothing after the contact is plotted as believed.
  const wrap = (d) => ((d + 540) % 360) - 180;
  const iPx4 = col('px4_yaw_deg');
  if (iPx4 < 0) throw new Error(`${meta.id}: no px4_yaw_deg column`);
  const gap = (k) => Math.abs(wrap(num(ticks[k][iPx4]) - num(ticks[k][iYaw])));
  let iJump = -1;
  for (let k = iTakeoff + 1; k < ticks.length; k++) {
    const step = Math.hypot(num(ticks[k][I.x]) - num(ticks[k - 1][I.x]), num(ticks[k][I.y]) - num(ticks[k - 1][I.y]));
    const dyaw = Math.abs(wrap(num(ticks[k][iYaw]) - num(ticks[k - 1][iYaw])));
    if (step > STEP_M || dyaw > YAW_DEG) {
      iJump = k;
      break;
    }
  }
  if (iJump < 0) throw new Error(`${meta.id}: no contact found (no pose jump after takeoff)`);
  let iHit = iJump;
  while (iHit - 1 > iTakeoff && gap(iHit - 1) > GAP_DEG) iHit--;
  const tHit = T[iHit];

  let tOffLost = null;
  let tDisarm = null;
  let sawOff = false;
  for (let k = iTakeoff; k < ticks.length; k++) {
    if (ticks[k][iOff] === '1') sawOff = true;
    if (sawOff && tOffLost === null && ticks[k][iOff] === '0') tOffLost = +(T[k] - tTakeoff).toFixed(1);
    if (tDisarm === null && ticks[k][iArmed] === '0') tDisarm = +(T[k] - tTakeoff).toFixed(1);
  }

  const pts = [];
  const states = [];
  let lastState = '';
  for (let k = iTakeoff; k <= iHit; k++) {
    const r = ticks[k];
    const t = +(T[k] - tTakeoff).toFixed(1);
    if (r[I.state] !== lastState) {
      states.push([t, r[I.state]]);
      lastState = r[I.state];
    }
    const x = num(r[I.x]);
    const y = num(r[I.y]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    const b = toCourse(x, y);
    const sx = num(r[I.sent_x]);
    const sy = num(r[I.sent_y]);
    const sent = Number.isFinite(sx) && Number.isFinite(sy) ? toCourse(sx, sy) : [null, null];
    const rd = (v) => (v === null ? null : +v.toFixed(3));
    pts.push([t, rd(b[0]), rd(b[1]), rd(sent[0]), rd(sent[1]), +(num(r[iZ]) || 0).toFixed(2), num(r[I.vo_state]) || 0, +(num(r[iYaw]) || 0).toFixed(1)]);
  }
  if (pts.length < 5) throw new Error(`${meta.id}: only ${pts.length} airborne ticks before contact`);
  const last = pts[pts.length - 1];
  const inM = Math.hypot(last[1] - lm[0], last[2] - lm[1]);
  out.runs.push({
    ...meta,
    tTakeoffLog: +tTakeoff.toFixed(1),
    tContact: +(tHit - tTakeoff).toFixed(1),
    tPoseJump: +(T[iJump] - tTakeoff).toFixed(1),
    tOffboardLost: tOffLost,
    tDisarm,
    contact: [last[1], last[2]],
    inM: +inM.toFixed(2),
    states,
    // t since takeoff, believed x, believed y, sent x, sent y (course ENU metres), believed height, vo_state
    points: pts,
  });
  console.log(`${meta.id}: takeoff ${tTakeoff.toFixed(1)} s into the log, contact ${(tHit - tTakeoff).toFixed(1)} s later, ${pts.length} points, believed ${inM.toFixed(2)} m from the launch mark at (${last[1].toFixed(2)}, ${last[2].toFixed(2)}); offboard lost ${tOffLost}, disarm ${tDisarm}`);
}

writeFileSync(path.join(pitch, 'src', 'data', 'evidence.json'), JSON.stringify(out));
console.log('wrote src/data/evidence.json');
