/** Typed view of src/data/evidence.json (written by scripts/extract-evidence.mjs from the real flight-recorder CSVs). */
import raw from '@/data/evidence.json';

export interface EvidenceRun {
  id: string;
  label: string;
  clock: string;
  code: string;
  setup: string;
  outcome: string;
  cause: string;
  /** seconds into the log when TAKEOFF began */
  tTakeoffLog: number;
  /** seconds from takeoff to the first impact, read from the log */
  tContact: number;
  tPoseJump: number;
  tOffboardLost: number | null;
  tDisarm: number | null;
  /** believed position at the impact, course ENU metres */
  contact: [number, number];
  /** believed distance from the launch mark at the impact */
  inM: number;
  states: [number, string][];
  /** t since takeoff, believed x, believed y, sent x, sent y (course ENU m), believed height, vo_state, believed yaw (deg: 0 = north, + = towards west) */
  points: (number | null)[][];
}

export const EVIDENCE = raw as unknown as {
  generatedBy: string;
  ref: string;
  refCommit: string;
  launch: [number, number];
  runs: EvidenceRun[];
};
