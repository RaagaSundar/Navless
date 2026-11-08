/** Shapes of src/data/course.json + route-window.json (see scripts/extract-course-data.mjs). ENU metres. */
export type Vec2 = [number, number];
export type Vec3 = [number, number, number];

export interface WallWindow {
  center: Vec2;
  width: number;
  height: number;
  bottom: number;
  top: number;
}

export interface Wall {
  name: string;
  kind: 'barrier' | 'approach' | 'chicane' | 'chicane_window' | 'maze' | string;
  a: Vec2;
  b: Vec2;
  thickness: number;
  height: number;
  window?: WallWindow;
}

export interface Tunnel {
  xMin: number;
  xMax: number;
  yMin: number;
  yMax: number;
  centerX: number;
  ovalWidth: number;
  ovalHeight: number;
  length: number;
  axis: string;
  zBottom: number;
  zTop: number;
  note?: string;
}

export interface Checkpoint {
  n: number;
  name: string;
  points: number;
  at: Vec2;
}

export interface CourseData {
  frame: string;
  notes: string;
  net: { halfX: number; halfY: number; height: number };
  walls: Wall[];
  removedWalls: string[];
  tunnel: Tunnel;
  boulders: { center: Vec2; diameter: number; height: number }[];
  pad: { center: Vec2; rOuter: number; rings: number[]; rApproach: number };
  launch: Vec2;
  gates: Record<string, { a: Vec2; b: Vec2; note?: string }>;
  checkpoints: Checkpoint[];
  drone: Record<string, unknown>;
}

export interface RouteData {
  meta: {
    type: string;
    frame: string;
    launch: Vec2;
    spacingM: number;
    lengthM: number;
    tipRadiusM: number;
    minClearanceM: number;
    minClearanceAt: string;
    tightSpotsM: Record<string, number>;
    notes: string;
    maze: string;
  };
  waypoints: Vec3[];
}
