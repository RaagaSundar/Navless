'use client';

import { Environment, Lightformer } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import * as THREE from 'three';
import { view } from '@/lib/choreography';
import { lerp } from '@/lib/math';

/**
 * GOLDEN HOUR. A low amber sun behind the subject, a soft warm bounce in front, a cool dusty-blue fill for the shadows.
 * Everything is generated in code (no HDRI downloads), so the deck works offline in a hackathon hall.
 *
 * The chrome is only as good as what it reflects, so the environment is a dim dusk with a few deliberate sources: a tall sun
 * strip low on one side, a glowing horizon band behind, a dusty-rose bounce from the ground, a small patch of cool sky, and
 * a pale ice strip opposite the sun. Chrome turns those into long, warm streaks with a cool edge. The directional lights
 * add the specular hits. All of it is static: only the intensity ramps up with the reveal, and warms a touch in override.
 */
const SUN = new THREE.Color('#ffb56a');
const SUN_OVERRIDE = new THREE.Color('#ff9d5e');

export default function StudioLights() {
  const sun = useRef<THREE.DirectionalLight>(null!);
  const bounce = useRef<THREE.DirectionalLight>(null!);
  const fill = useRef<THREE.DirectionalLight>(null!);
  const top = useRef<THREE.DirectionalLight>(null!);

  useFrame(() => {
    // lights come up with the reveal
    const up = Math.min(1, view.droneReveal * 1.4);
    if (sun.current) {
      sun.current.intensity = lerp(0, 3.0, up);
      sun.current.color.lerpColors(SUN, SUN_OVERRIDE, view.o);
    }
    if (bounce.current) bounce.current.intensity = lerp(0, 1.9, up);
    if (fill.current) fill.current.intensity = lerp(0, 0.9, up);
    if (top.current) top.current.intensity = lerp(0, 0.35, up);
  });

  return (
    <>
      <Environment resolution={256} frames={1} environmentIntensity={1.35}>
        <color attach="background" args={['#0a0504']} />
        {/* the sun: a tall warm strip low behind-left, and a wide horizon band all the way round */}
        <Lightformer form="rect" intensity={13} color="#ffb066" position={[-7, 0.8, -5]} scale={[0.9, 9, 1]} target={[0, 0, 0]} />
        <Lightformer form="rect" intensity={5} color="#ff8a3d" position={[0, -0.2, -10]} scale={[26, 0.9, 1]} target={[0, 0, 0]} />
        {/* soft warm front-left: the face of the airframe catches this */}
        <Lightformer form="rect" intensity={5.5} color="#ffd2a1" position={[-6, 1.4, 6]} scale={[6, 3, 1]} target={[0, 0, 0]} />
        {/* the cool side: a pale ice strip opposite the sun, and a small patch of blue sky overhead */}
        <Lightformer form="rect" intensity={6.5} color="#a9cde6" position={[6.5, 1, 2]} scale={[0.45, 8, 1]} target={[0, 0, 0]} />
        <Lightformer form="rect" intensity={1.6} color="#8fb0d8" position={[4, 6.5, 3]} scale={[6, 4, 1]} rotation-x={Math.PI / 2} />
        {/* overhead dome and the ground bounce: dusty rose off the regolith */}
        <Lightformer form="rect" intensity={1.1} color="#ffe2c4" position={[0, 9, 0]} scale={[20, 20, 1]} rotation-x={Math.PI / 2} />
        <Lightformer form="rect" intensity={2.0} color="#8a4a30" position={[0, -6, 0]} scale={[14, 14, 1]} rotation-x={-Math.PI / 2} />
      </Environment>

      <ambientLight intensity={0.05} color="#7a5a4a" />
      {/* the sun: low, behind-left, so the airframe is edged in gold */}
      <directionalLight ref={sun} position={[-7, 2.2, -5]} color="#ffb56a" />
      {/* warm bounce from the front-left, low: the long soft light on the face */}
      <directionalLight ref={bounce} position={[-6, 2.4, 7]} color="#ffd6a8" />
      {/* cool fill from the right: golden-hour shadows lean blue */}
      <directionalLight ref={fill} position={[6, 5, 5]} color="#8aa6d0" />
      {/* a narrow overhead hit so flat plates gradient as the drone turns */}
      <directionalLight ref={top} position={[1.5, 9, 0.5]} color="#ffe8cc" />
    </>
  );
}
