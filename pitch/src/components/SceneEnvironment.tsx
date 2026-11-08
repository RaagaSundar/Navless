'use client';

import MazeEnvironment from './MazeEnvironment';
import RouteLine from './three/RouteLine';
import ScanCloud from './three/ScanCloud';
import StudioLights from './three/StudioLights';

/**
 * SceneEnvironment: everything that is not the drone, the headline type or the judge's blockers.
 *
 *   StudioLights      sharp directional lights, dim ambient, procedural strip-light environment for the chrome
 *   MazeEnvironment   the as-built course as a wireframe (EdgesGeometry over hologram glass), floor grid, net, checkpoints
 *   ScanCloud         the LiDAR-style point cloud the map resolves out of
 *   RouteLine         skybeta-19's real route as a MeshLine glow, with the stereo cone and the blind-carry tag
 */
export default function SceneEnvironment() {
  return (
    <>
      <StudioLights />
      <MazeEnvironment />
      <ScanCloud />
      <RouteLine />
    </>
  );
}
