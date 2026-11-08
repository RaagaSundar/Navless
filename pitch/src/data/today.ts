/**
 * today.ts: the few things that change on the day. Edit, save, then reload (dev server) or `npm run build` (production).
 * The slides, the speaker script and `npm run script` (SCRIPT.md) all read from here, so they stay in step.
 *
 * Defaults are what was true at the 2026-11-08 ~12:00 PST handoff. Check each one before 3:20 PM.
 */
export interface ScoredRun {
  /** slot as the organisers list it, e.g. "11:18 AM" */
  slot: string;
  /** one plain sentence: what happened (checkpoints reached, what it touched, or that it finished) */
  result: string;
}

export const TODAY = {
  /**
   * The scored runs that have already happened, in order. Leave empty until there is something true to say.
   * Example: { slot: '11:18 AM', result: 'reached checkpoint 2, then touched the north net on the turn east' }
   */
  scored: [] as ScoredRun[],

  /** Live re-planning around stereo obstacles: on in the runs we fly? (it was off: it saw phantom obstacles on the floor) */
  replanning: false,

  /** Did the course-model localizer / wall_correction steer a real flight yet? (as of the handoff: never) */
  localizerSteering: false,

  /** PX4 EKF2_MAG_TYPE = 5, compass off, heading from the camera only (Shining set it in QGC overnight; old value 0) */
  compassOff: true,
};
