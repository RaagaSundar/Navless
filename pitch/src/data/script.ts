/**
 * script.ts: the 20-minute judging pitch, in the seven-part order the judging page asks for, plus what to say.
 *
 * ONE source for three things: the slides' build order (`steps`), what is said at each step (`beats`, shown in the notes panel with
 * N) and SCRIPT.md (`npm run script`). Numbers come from facts.ts and today.ts, so the script and the slides cannot disagree.
 *
 * The order is the judges': 1 the problem as we read it, 2 how it works (one diagram), 3 why this approach and what we tried
 * first, 4 what generalizes vs what is tuned, 5 what breaks it, 6 evidence, 7 the hardest problem, the next day, who did what.
 * About 10 minutes of speech (12:50 with pauses and clicks) fits the 13-minute slot the organisers named and leaves the questions, which are the judges', alone.
 */
import { N, TEAM } from './facts.ts';
import { TODAY } from './today.ts';

export type SlideId = 'title' | 'problem' | 'system' | 'approach' | 'general' | 'breaks' | 'evidence' | 'close';

export interface Beat {
  /** the build step on the slide this is said at (0 = when the slide opens) */
  step: number;
  /** what happens on screen, or what to do: not spoken */
  cue?: string;
  say: string;
  /** a fact that still needs filling in: shown as a warning in the notes and in SCRIPT.md */
  todo?: string;
}

export interface Slide {
  id: SlideId;
  /** 0 for the title, then 1..7 in the judges' order */
  chapter: number;
  /** short name for the chrome */
  label: string;
  /** the headline on the slide: a claim, not a topic */
  claim: string;
  steps: number;
  seconds: number;
  /** what the 3D scene does behind the slide: scroll position on the master timeline, and how much to dim it */
  backdrop: { p: number; dim: number };
  beats: Beat[];
}

const v = (k: keyof typeof N) => N[k].value;

/* ───────── facts that depend on what is true today ───────── */
export interface NotDone {
  title: string;
  /** on the slide */
  why: string;
  /** said aloud */
  say: string;
}

/** What we chose NOT to attempt, and why. Drops an item when today's flags say it is actually in use. */
export const NOT_DONE: NotDone[] = [
  {
    title: 'Race',
    why: `${v('cruise')}. A clean lap beats a crash`,
    say: `We don't race. We fly at ${v('cruise')}, because a clean lap beats a fast crash.`,
  },
  ...(TODAY.replanning
    ? []
    : [
        {
          title: 'Re-plan live',
          why: 'never gated, saw phantom obstacles',
          say: "We don't re-plan live. It's built, but stereo depth saw phantom obstacles on the floor, and we never gated it.",
        },
      ]),
  ...(TODAY.localizerSteering
    ? []
    : [
        {
          title: 'Steer by the localizer',
          why: 'never flown for real',
          say: "The localizer doesn't steer. It's built and tested in simulation, but it has never steered a real flight.",
        },
      ]),
  {
    title: 'Land by camera',
    why: 'works in sim, off on the drone',
    say: "And we don't land by camera. The pad detector works in simulation, and it's off. We land on the dead-reckoned pad.",
  },
];
const countWord = ['', 'One', 'Two', 'Three', 'Four', 'Five'][NOT_DONE.length];

const compassStory = TODAY.compassOff
  ? `On the real floor the compass read ${v('compass')} off, and kept flipping PX4's heading. So overnight we turned it off in PX4, and heading comes from the camera alone.`
  : `On the real floor the compass read ${v('compass')} off, and kept flipping PX4's heading. We still guard against it in our own code, and the heading checks refuse to fly when they disagree.`;

const tofWhy = TODAY.tofReason ? ` ${TODAY.tofReason}` : '';
const scored = TODAY.scored.length ? `And our scored runs. ${TODAY.scored.map((r) => `${r.slot}: ${r.result}.`).join(' ')}` : '';

const tunedList = [
  'the as-built map of this net and this maze',
  'the route, north-east off one launch mark',
  `the takeoff lean, ${v('lean')}, that cancels this airframe's liftoff slide`,
  `this Pixhawk's ${v('heightOffset')} height offset`,
  `the ${v('netClamp')} net margin and the ${v('blindCarry')} blind carry, sized for a ${v('blind')} tunnel`,
  ...(TODAY.compassOff ? ["and the compass setting, for this room's magnetics"] : []),
];

export const SLIDES: Slide[] = [
  {
    id: 'title',
    chapter: 0,
    label: 'Navless',
    claim: 'Zero GPS. Pure autonomy.',
    steps: 1,
    seconds: 20,
    backdrop: { p: 0.3, dim: 0 },
    beats: [
      {
        step: 0,
        cue: 'Title. Names are on screen.',
        say: `We're Navless. Our drone is skybeta-19: no GPS, stereo cameras, a Jetson on board. About ten minutes from us, then it's your questions. I'm Raaga. Bryan, Favour, Shining and Koji can each take any part of it.`,
      },
    ],
  },
  {
    id: 'problem',
    chapter: 1,
    label: 'The problem',
    claim: 'Fly the whole net without touching it.',
    steps: 3,
    seconds: 90,
    backdrop: { p: 0.9, dim: 0.8 },
    beats: [
      {
        step: 0,
        cue: 'The course appears with its five checkpoints.',
        say: `The course is a 20-by-20-foot net with five checkpoints worth ${v('points')}: a corridor, a maze, a dark tunnel, a window chicane, and a landing on a bullseye. Our drone is ${v('droneWidth')} wide with props. So the maze gap, one metre, leaves ${v('gapSide')} a side, and the window leaves ${v('window')} a side.`,
      },
      {
        step: 1,
        cue: 'What mattered most.',
        say: `What mattered most was clearance, not speed. Our planned prop room is ${v('room')} at the tightest wall, and the real errors we measured are ${v('errors')}: start heading, liftoff slide, drift. That is the same size as our margin. And the biggest checkpoint, the tunnel at 10 points, is dark. The camera sees nothing in it for about ${v('blind')}.`,
      },
      {
        step: 2,
        cue: 'What we chose not to attempt.',
        say: `${countWord} things we chose not to do. ${NOT_DONE.map((t) => t.say).join(' ')}`,
      },
    ],
  },
  {
    id: 'system',
    chapter: 2,
    label: 'How it works',
    claim: 'Five stages, two computers, one loop.',
    steps: 6,
    seconds: 165,
    backdrop: { p: 0.58, dim: 0.88 },
    beats: [
      {
        step: 0,
        cue: 'The whole diagram. Sense lights up.',
        say: `This is the whole system, left to right. Sense: one stereo camera, ${v('camera')}, and a downward laser for height. No GPS, no optical flow. All of it is on the drone.`,
      },
      {
        step: 1,
        cue: 'Estimate.',
        say: `Estimate: where am I. NVIDIA's cuVSLAM runs on the Jetson on the stereo pair alone, ${v('vslam')}, worst gap 49 milliseconds. A gate drops any pose that isn't tracking or is older than ${v('stale')}, because a stale pose looks like a drone standing still. What's left goes to PX4's EKF2 on the Pixhawk as external vision.`,
      },
      {
        step: 2,
        cue: 'Plan.',
        say: `Plan: what's around me. We scanned the room with a phone's LiDAR, ${v('scan')}, and built the course as it really is. The planner turns that into a grid of 10-centimetre cells, pads every wall by 28 centimetres, runs A-star and smooths the path into a spline. That runs on the Jetson too.`,
      },
      {
        step: 3,
        cue: 'Decide.',
        say: `Decide. course_mission is a state machine at ${v('mission')}: wait, take off, route, land, plus hold and refuse. It walks the path with a speed-limited carrot at ${v('cruise')}. Every guard in it came from something we saw in a log, and I'll show those in a few minutes. When the camera goes blind in the dark tunnel, it carries on PX4's own estimate for at most ${v('blindCarry')}.`,
      },
      {
        step: 4,
        cue: 'Act.',
        say: `Act. It publishes setpoints to the organisers' flight interface, which hands them to PX4 in Offboard mode, ${v('heartbeat')} heartbeat. We never arm, never change mode, never write to PX4 directly. The safety pilot has the RC the whole time.`,
      },
      {
        step: 5,
        cue: 'The loop closes. Two computers.',
        say: `And it closes on itself: the drone moves, the camera sees it move, the pose comes back. Two computers: the Jetson does vision, planning and decisions, the Pixhawk does estimation and control. Nothing else runs on the Jetson while we fly. When a simulation shared it, the camera rate fell from 29 to 26 Hz with gaps up to 300 milliseconds.`,
      },
    ],
  },
  {
    id: 'approach',
    chapter: 3,
    label: 'Why this approach',
    claim: 'Three times, the course changed our mind.',
    steps: 3,
    seconds: 105,
    backdrop: { p: 0.9, dim: 0.86 },
    beats: [
      {
        step: 0,
        cue: 'The map.',
        say: `First, the map. We started from the organisers' plan. Our phone scan showed five walls in the wrong place. One sat ${v('m3')} off, and a route planned on the plan passed 2.8 centimetres from it. So we plan on our own scan. And when the organisers rebuilt the maze the night before, it was one course file: we re-exported the route and re-ran the sim gate that evening.`,
      },
      {
        step: 1,
        cue: 'The tunnel.',
        say: `Second, the tunnel. Our first answer to a dark tunnel was: when tracking drops, freeze. In simulation that hit the tube ${v('freeze')} times. Now the route runs down the middle of the tube, and when the camera goes blind the drone rides on PX4's own estimate for at most ${v('blindCarry')}. In simulation that landed 4 out of 4, with 6 clean passes. We also built centring for it, with side time-of-flight sensors and with stereo, and we don't fly with either.${tofWhy}`,
        todo: TODAY.tofReason ? undefined : 'A judge may ask why the side ToF sensors are not flown. Add one plain sentence as tofReason in src/data/today.ts and it appears here, on /tech and in the Q&A.',
      },
      {
        step: 2,
        cue: 'The compass. The floor tape is the dropped idea.',
        say: `Third, the compass. We began by keeping the organisers' default parameters and guarding against the compass in our own code. ${compassStory} We also looked at following the floor tape, and dropped it: it fixes sideways offset, not progress along the line, and it's the old maze's tape.`,
      },
    ],
  },
  {
    id: 'general',
    chapter: 4,
    label: 'What generalizes',
    claim: 'Here is exactly where the line is.',
    steps: 3,
    seconds: 75,
    backdrop: { p: 0.9, dim: 0.86 },
    beats: [
      {
        step: 0,
        cue: 'Left column: generalizes.',
        say: `These would work in another room: stereo visual odometry fused into PX4, the odometry gate, the state machine and its guards, the planner, blind carry through any short vision gap, and the flight recorder. They need a textured room and a camera. Nothing in them knows our course.`,
      },
      {
        step: 1,
        cue: 'Right column: tuned to this course.',
        say: `These are tuned to this course and this drone: ${tunedList.join('; ')}.`,
      },
      {
        step: 2,
        cue: 'The test of the line.',
        say: `The test of that line: when the organisers moved three walls the night before, the change was data. One course file, a new route, a re-run of the sim gate.`,
      },
    ],
  },
  {
    id: 'breaks',
    chapter: 5,
    label: 'What breaks it',
    claim: 'Six ways it breaks. One is still open.',
    steps: 3,
    seconds: 105,
    backdrop: { p: 0.7, dim: 0.82 },
    beats: [
      {
        step: 0,
        cue: 'Tracking and the tunnel.',
        say: `Camera loses tracking, or the light goes: the gate drops stale poses, the route target freezes, and after ${v('noProgress')} of no progress it holds. Dark tunnel: it knows from the map that it's inside the tube, carries on PX4's estimate along the route for at most ${v('blindCarry')}.`,
      },
      {
        step: 1,
        cue: 'Heading and a bad start.',
        say: `Heading disagrees: if PX4 and the camera differ by more than 15 degrees, or PX4 resets, the node stops steering the nose. Before takeoff they must agree within 10, or it refuses to fly. That happened overnight: the compass was about 100 degrees off, and the drone wouldn't take off. Bad start: it checks height against the camera and streams nothing until they agree within 30 centimetres.`,
      },
      {
        step: 2,
        cue: 'Drift, and the open one. Optional: press L for the live blocker demo.',
        say: `Drift: it learns a steady push, and no setpoint goes within ${v('netClamp')} of the net. And the one we haven't closed: an odometry jump. In run two the camera's position jumped almost 3 metres after the hit, still reporting tracking, and the node kept flying for 20 seconds. A latch for that exists on a branch. It is not in the flight build. When anything goes wrong, the drone holds or lands, and any mode switch on the transmitter hands control back to the safety pilot.`,
      },
    ],
  },
  {
    id: 'evidence',
    chapter: 6,
    label: 'Evidence',
    claim: 'What it believed. What happened.',
    steps: 6,
    seconds: 130,
    backdrop: { p: 0.9, dim: 0.9 },
    beats: [
      {
        step: 0,
        cue: 'The course, no run selected yet.',
        say: `This is what the flight recorder logged on our first course runs, overnight. Blue is where the drone believed it was. The ring is where it touched. Five attempts: four contacts, one refused takeoff, none finished. Every tick is logged at ${v('rate')}, and this plot is drawn from those files.`,
      },
      {
        step: 1,
        cue: 'Run 1.',
        say: `Run one. By its own tracking the drone was on its line, props 32 centimetres clear of this wall, and it hit it. A 12-degree error in the starting heading does exactly that.`,
      },
      {
        step: 2,
        cue: 'Run 2.',
        say: `Run two. Through the one-metre gap, then wide on the turn east, into the north net. Then the camera's position jumped almost 3 metres while still saying it was tracking.`,
      },
      {
        step: 3,
        cue: 'Run 4.',
        say: `Run four. The liftoff slide: 34 centimetres left during the climb, into the west net, before the route had started. The odometry saw it. Nothing in the takeoff pushed back.`,
      },
      {
        step: 4,
        cue: 'Run 5.',
        say: `Run five. By odometry, 40 centimetres clear of every wall. It hit one anyway. So the start pose was wrong: about 13 degrees, or 40 centimetres.`,
      },
      {
        step: 5,
        cue: 'Two kinds of failure, and the sim numbers.',
        todo: TODAY.scored.length ? undefined : 'No scored-run result entered. Edit TODAY.scored in src/data/today.ts and it appears here and on the slide.',
        say: `Two kinds of failure. Belief wrong from the start: placement and heading, runs one and five. Belief right, motion wrong: the slide and the turn. In every run the camera's track stayed smooth until the touch. In simulation the route-file build flew ${v('simLaps')}, no contacts, landing 6 to 8 centimetres from the pad. That gap is what we're closing: the sim starts exact, the floor doesn't. ${scored}`.trim(),
      },
    ],
  },
  {
    id: 'close',
    chapter: 7,
    label: 'Next',
    claim: 'The hardest part is the last 10 cm.',
    steps: 3,
    seconds: 80,
    backdrop: { p: 0.46, dim: 0.84 },
    beats: [
      {
        step: 0,
        cue: 'Margin against error.',
        say: `The hardest problem is the last 10 centimetres. In simulation, a 10-centimetre, 3-degree start error leaves ${v('margin')} of room. On the floor we have seen errors up to 40. Every failure overnight came from an error in that size range.`,
      },
      {
        step: 1,
        cue: 'One more day.',
        say: `With one more day, three things. One: odometry-jump protection in the flight build, so a bad pose means hold, not fly on. Two: ${TODAY.localizerSteering ? 'validate the localizer on more real flights' : 'validate the localizer on the real drone, shadow first, then bounded correction, before it steers'}. Three: more room at the tip of the diagonal wall and slower into that turn. Today's path leaves ${v('mdTip')} there.`,
      },
      {
        step: 2,
        cue: 'Who did what. End on Questions.',
        say: `Who did what. ${TEAM.map((m) => `${m.name.split(' ')[0]}: ${m.say}.`).join(' ')} Any of us can answer any part. Questions.`,
      },
    ],
  },
];

export const wordsOf = (s: Slide) => s.beats.reduce((n, b) => n + b.say.split(/\s+/).filter(Boolean).length, 0);
export const totalSeconds = () => SLIDES.reduce((n, s) => n + s.seconds, 0);

export interface QA {
  q: string;
  a: string;
}

/** Likely questions, short answers. Shown on the numbers sheet (Q) and in SCRIPT.md. */
export const QA_LIST: QA[] = [
  { q: 'What is that number, and where did it come from?', a: 'Press Q: every number on the slides is in the sheet with its file, log or person.' },
  {
    q: 'Why stereo odometry and not something else?',
    a: 'We carry no GPS or optical-flow sensor. The stereo pair is the sensor we had, cuVSLAM runs on the Jetson at 31 Hz, and PX4 fuses vision natively. We looked at the floor tape and at tunnel centring, with stereo and with side time-of-flight sensors, and none of them is in the flight.',
  },
  {
    q: 'Why are the side ToF sensors not flying?',
    a: `We built them and bench-tested them (9.6 Hz, 99 to 99.6 % valid, steady to ±0.3 cm), and a closed-loop sim brought a 10 to 12 cm offset to within 2 cm. We decided not to fly with them.${tofWhy}`,
  },
  {
    q: 'Has the drone flown the tunnel for real?',
    a: 'No. None of the five overnight attempts got that far, so every tunnel result is simulation. Those sim runs used the course launch, whose ToF chain was on by default at the time, so blind carry alone has not been shown separately.',
  },
  {
    q: 'Has the localizer steered a real flight?',
    a: TODAY.localizerSteering
      ? 'Yes, see the scored runs. It is still lightly tested on the real drone.'
      : 'No. It is built and tested in simulation. Its one real run crashed in the takeoff climb, before the route. Real validation is the next step.',
  },
  {
    q: 'How many real runs, and how many finished?',
    a: `Overnight: five course attempts, four contacts, one refused takeoff, none finished.${TODAY.scored.length ? ` Then scored: ${TODAY.scored.map((r) => `${r.slot}, ${r.result}`).join('; ')}.` : ''} The refusal was the pre-arm heading check working.`,
  },
  {
    q: 'Why does the sim pass and the floor fail?',
    a: 'The sim starts exact. On the floor the start pose was off by 12 to 13 degrees or about 40 cm in two runs, and the drone slides 25 to 40 cm at liftoff. A 10 cm, 3 degree start error already leaves only 2 to 6 cm in the gate.',
  },
  {
    q: 'Is the planner in the demo the real one?',
    a: 'No. The demo is a TypeScript re-implementation of the same idea: grid, A-star, spline, 0.25 m keep-out against 0.28 m on the drone. Its telemetry is labelled SIM. The real planner is planner/ in the repo.',
  },
  {
    q: 'What does EKF2_MAG_TYPE 5 do?',
    a: 'It turns the compass off in PX4, so heading comes from the camera only. Shining set it in QGC after the compass read 45 to 155 degrees off indoors. The old value was 0, and it goes back to 0 before any outdoor flight.',
  },
  {
    q: 'What if the camera loses tracking mid-flight?',
    a: 'The odometry gate drops stale poses, the route target freezes, and after 20 s of no progress it holds. Inside the tunnel only, it rides PX4’s estimate for up to 4 s. Not yet covered: a pose that jumps while still saying tracking.',
  },
  {
    q: 'Does cuVSLAM use the IMU?',
    a: 'No, stereo only, IMU fusion off. PX4’s own EKF2 uses its IMU as usual.',
  },
  {
    q: 'Which build did the simulation numbers come from?',
    a: 'The route-file build, drone19/flight 865db6b: 3 laps, 0 contacts. The planner and localizer build (main-localizer) has not been through the gate with threaded re-planning, travel heading and the yaw localizer.',
  },
];
