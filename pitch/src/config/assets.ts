/**
 * ASSET SLOTS: swap these URLs (files live in /public) to inject your own CAD.
 *
 *  drone   .glb / .gltf. Rendered with `<primitive object={scene} />` in DroneModel.tsx.
 *          Optional contract: name empties  prop_FL prop_FR prop_RL prop_RR  to get spinning props
 *          (extras.dir = 1 | -1, extras.radius, metres), and  anchor_camera anchor_jetson ...  for HUD callouts.
 *          Give nodes extras.role ('plate' | 'arm' | 'chrome' | 'lens' ...) to choose their liquid-chrome / glass material.
 *          Nodes without a role fall back to chrome. See README.md > "Asset slots".
 *
 *  maze    .glb / .gltf / .obj. Rendered with `<primitive object={scene} />` in MazeEnvironment.tsx.
 *          Nodes are styled by extras.kind (GLB) or by a regex on the node name (OBJ): wall / tube / boulder / pad / net / floor.
 *
 *  scan    optional LiDAR-style point cloud (course-scan.bin). Set to null to skip the scan -> map reveal.
 */
/** bump when you regenerate a model: /models/* is served `immutable` in production builds */
const V = '3';

export const ASSETS = {
  drone: `/models/navless-uav.glb?v=${V}`,
  maze: `/models/dronehacks-course.glb?v=${V}`,
  scan: `/models/course-scan.bin?v=${V}` as string | null,
  /** WebGL display font. troika reads .woff / .ttf / .otf (not woff2). */
  fontDisplay: '/fonts/archivo-black-latin-400-normal.woff',
} as const;

export const BRAND = {
  name: 'NAVLESS',
  drone: 'SKYBETA-19',
  event: 'DRONEHACKS INDOOR COURSE',
} as const;
