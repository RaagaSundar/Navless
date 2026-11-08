/**
 * facts.ts: every number the talk uses, written once, with where it came from.
 *
 * The slides, the speaker script (`npm run script` writes SCRIPT.md) and the on-screen numbers sheet (press Q) all read from
 * here, so they cannot disagree. Sources are files in the Navless repo (README.md etc., on origin/main), the Slack channel, or
 * the flight-recorder CSVs on origin/flight-logs/2026-11-08. When a number changes, change it here and nowhere else.
 * The judges will ask "what is that number and where did it come from": the `source` is the answer.
 */
export interface Num {
  id: string;
  /** what the audience sees */
  value: string;
  /** what the number IS */
  what: string;
  /** where it came from: a file, a log, a person */
  source: string;
  /** which section of the pitch uses it, 1..7 */
  section: number;
}

const n = (section: number, id: string, value: string, what: string, source: string): Num => ({ id, value, what, source, section });

export const N = {
  /* 1 · the problem */
  points: n(1, 'points', '35 points', 'Five checkpoints worth 4 + 7 + 10 + 7 + 7: corridor, maze, tunnel, chicane, landing', 'README "The course"; src/data/course.json'),
  droneWidth: n(1, 'droneWidth', '47.5 cm', 'skybeta-19 width with props (16 cm ahead of centre, 23 behind, 24 to each side)', 'README "The course"; gazebo_sim/course/course_asbuilt.json'),
  gap: n(1, 'gap', '1 m', 'The maze gap between the tip of the long diagonal wall (MD) and the north net', 'README "The course"; course_asbuilt.json gate_A'),
  gapSide: n(1, 'gapSide', '26 cm', 'Room each side of the drone in that gap: (100 cm - 47.5 cm) / 2', 'derived from the two lines above'),
  window: n(1, 'window', '8.6 inches', 'Room each side of the drone in the 36 in window', 'README "The course"'),
  blind: n(1, 'blind', '3 s', 'How long the camera tracks nothing inside the dark tunnel', 'README "The course" and section 4'),
  room: n(1, 'room', '17.9 cm', 'Planned prop room to the tightest wall (Barrier 1, behind the launch mark), nose north', 'route-window.json meta.minClearanceM = 0.179; README section 2'),
  errors: n(1, 'errors', '5 to 40 cm', 'Real errors measured on the floor: start heading, liftoff slide, corner lag, drift', 'README "Nov 8" chart caption; flight_logs README'),
  cruise: n(1, 'cruise', '0.5 m/s', 'Speed on straights (0.6 m carrot lead, 0.25 m/s² in turns)', 'README section 3'),

  /* 2 · how it works */
  camera: n(2, 'camera', '1024×640 at 30 Hz', 'OAK-D-S2 stereo pair into cuVSLAM, IMU fusion off', 'README section 1'),
  vslam: n(2, 'vslam', '31.2 Hz', 'cuVSLAM output rate on skybeta-19 (worst gap 49 ms, Jetson quiet)', 'README section 1; hardware_slam/'),
  gate: n(2, 'gate', '28.4 Hz', 'Poses that pass the odometry gate (tracking and under 0.25 s old); median age 59.89 ms', 'README section 1; hardware_slam/odom_gate.py'),
  tof: n(3, 'tof', '9.6 Hz', 'Side time-of-flight sensors, bench-tested on 19 (99 to 99.6 % valid, steady to ±0.3 cm). Built, not flown: we no longer use them', 'README section 4 (bench on skybeta-19); Raaga, 2026-11-08: no longer used'),
  mission: n(2, 'mission', '10 Hz', 'course_mission tick rate; the flight recorder writes one row per tick', 'README "How we test"; flight_logs README'),
  heartbeat: n(2, 'heartbeat', '10 Hz', 'flight_interface heartbeat that keeps PX4 in Offboard (organisers’ code)', 'README "How it works"'),
  grid: n(2, 'grid', '0.1 m cells, walls +0.28 m', 'Planner grid and wall padding, then A* and a B-spline', 'README section 6; planner/config.py'),
  scan: n(2, 'scan', '596k points', 'Phone-LiDAR scan the as-built course is built from; aligned to the official plan to within 0.3 to 6 cm', 'README "The course"'),
  starved: n(2, 'starved', '29.4 → 25.6 Hz', 'cuVSLAM rate when a simulation container shared the Jetson (worst gap 39 ms → 300 ms)', 'README section 1'),

  /* 3 · why this approach */
  m3: n(3, 'm3', '76 cm', 'How far wall M3 sat from where the official map put it; a route planned on the map passed 2.8 cm from it', 'README "Nov 6" caption'),
  freeze: n(3, 'freeze', '3 of 3', 'Master sim: freezing when tracking drops hit the tube 3 of 3 times', 'README section 4 (master sim, the drone’s own launch, whose ToF chain was on by default then)'),
  blindCarry: n(3, 'blindCarry', '4 s', 'Longest gap blind_carry rides through on PX4’s estimate, only inside the tube; landed 4/4, 6/6 clean passes at 0.5 m/s', 'README section 4 (master sim, the drone’s own launch, whose ToF chain was on by default then: it does not isolate blind carry)'),
  compass: n(3, 'compass', '45 to 155°', 'How far the compass read from the camera heading indoors on 19, varying by spot', 'Shining’s handoff, Slack 2026-11-08'),
  compassSnap: n(3, 'compassSnap', '76° sim, ~100° floor', 'PX4 heading snap to the magnetometer about 0.5 s into a vision gap', 'README section 1'),

  /* 4 · what generalizes */
  heightOffset: n(4, 'heightOffset', '18.5 to 19.8 cm', 'PX4’s floor height reads this much above the camera’s on 19; measured at every arming', 'README section 3'),
  slide: n(4, 'slide', '25 to 40 cm', 'Liftoff slide to the left on 19, every takeoff', 'README (25-31 cm in the flight tests); Shining’s handoff (25-40 cm)'),
  lean: n(4, 'lean', '30 cm', 'Takeoff lean (drift_seed_body_y_m = -0.30) that cancels this airframe’s slide', 'Shining’s handoff, Slack 2026-11-08 (main-localizer, local edits)'),
  netClamp: n(4, 'netClamp', '25 cm', 'No setpoint is ever sent within 25 cm of the net (flight_interface clamps altitude only)', 'README section 3'),

  /* 5 · what breaks it */
  headingGuard: n(5, 'headingGuard', '15° / 10°', 'Stop steering the nose when PX4 and the camera disagree by more than 15° (or PX4 resets); takeoff needs them within 10°', 'README section 3'),
  stale: n(5, 'stale', '0.25 s', 'A pose older than this, or not tracking, is dropped by the odometry gate', 'README section 1'),
  noProgress: n(5, 'noProgress', '20 s', 'No progress along the route for this long switches to HOLD', 'navless_nav course_mission; run 092321 log (ROUTE persisted 19.5 s before HOLD)'),
  preHeight: n(5, 'preHeight', '30 cm for 2 s', 'Pre-arm height match: nothing streams until PX4 and the camera agree this closely', 'README section 3'),
  jump: n(5, 'jump', '2.8 m', 'Run 2: the camera pose jumped 1.1 m then 1.7 m within 0.2 s of the hit, still reporting "tracking"', 'flight log 20261108-092321 at 47.1-47.2 s (README says 2.9 m, Shining 2.7 m)'),

  /* 6 · evidence */
  attempts: n(6, 'attempts', '5 attempts', 'Course attempts overnight, before the scored runs: 4 contacts, 1 refused takeoff, 0 finished', 'flight_logs/2026-11-08_skybeta19_course (branch flight-logs/2026-11-08)'),
  simLaps: n(6, 'simLaps', '3 of 3 laps', 'Gazebo: 0 contacts, 39.6 to 39.9 s, landing 6.4 to 7.6 cm from the pad centre, closest wall 16.0 cm. Route-file build 865db6b, 5 mm odometry noise, 50 ms delay, PX4 19 cm high', 'README "Results"'),
  rate: n(6, 'rate', '10 Hz', 'Rows in the flight recorder; the plot draws every row from take-off to the first impact', 'pitch/scripts/extract-evidence.mjs'),

  /* 7 · hardest problem */
  margin: n(7, 'margin', '2 to 6 cm', 'Prop room left in the sim gate when the start is 10 cm or 3° off', 'Shining’s handoff, Slack 2026-11-08 ("Raaga’s gate agrees")'),
  mdTip: n(7, 'mdTip', '14 cm', 'Prop room the planner’s path leaves past the tip of MD', 'Shining’s message, Slack 2026-11-08'),
} as const satisfies Record<string, Num>;

export const NUMBERS: Num[] = Object.values(N);

/** Per-run reading for the evidence plot. Outcomes and causes are the notes in flight_logs/2026-11-08_skybeta19_course/README.md. */
export interface RunNote {
  believed: string;
  happened: string;
  /** which solid it touched, when the notes name it (drawn in the plot) */
  hit: 'MD' | 'north net' | 'west net' | null;
  /** belief = the drone's idea of where it was started wrong; motion = the belief was right and the motion was wrong */
  kind: 'belief' | 'motion';
}

export const RUN_NOTES: Record<string, RunNote> = {
  '20261108-091401': { believed: 'On its line: props 32 cm clear of MD', happened: 'Hit MD, 1.5 m in', hit: 'MD', kind: 'belief' },
  '20261108-092321': { believed: 'Through the 1 m gap, then wide on the turn east', happened: 'Hit the north net, side-on. Then the camera jumped 2.8 m while still "tracking"', hit: 'north net', kind: 'motion' },
  '20261108-114602': { believed: 'Sliding left in the climb: the odometry saw it', happened: 'Slid 34 cm into the west net before the route started', hit: 'west net', kind: 'motion' },
  '20261108-133310': { believed: '40 cm clear of every wall', happened: 'Hit the front wall about 1.9 m in', hit: null, kind: 'belief' },
};

/** The refused takeoff is run 3 (114307): no track to plot, the pre-arm check did its job. */
export const RUN_3 = 'Run 3 · refused takeoff: PX4 heading was on the compass, ~100° off the camera. The pre-arm check stopped it.';

export const TEAM = [
  { name: 'Raaga Sundar', does: 'Flight program (navless_nav): route, guards, blind carry, the sim gate', say: 'the flight program, the guards and the sim gate' },
  { name: 'Koji Shukawa', does: 'Stereo odometry on the Jetson, the odometry gate, split sim, networking', say: 'stereo odometry on the Jetson and the odometry gate' },
  { name: 'Shining Wang', does: 'Phone-scan course, stereo depth, localizer, flight tests, recorder', say: 'the phone scan, the localizer, the flight tests and the recorder' },
  { name: 'Favour Adekola', does: 'Path planner and live re-planning', say: 'the path planner' },
  { name: 'Bryan Pham', does: 'Testing: master sim, FlyX Lab, camera pad landing, flight-day checks', say: 'testing, the master sim, pad landing and the flight-day checks' },
] as const;
