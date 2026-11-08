'use client';

import { useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { view } from '@/lib/choreography';
import { PAL } from '@/lib/palette';

/**
 * Dust: Mars dust drifting up through the course in the low golden light. Pure atmosphere for the eye: about 500 motes, rising
 * a few centimetres a second with a lazy sway, each fading in at the floor and out at the top so none ever pops. It only exists
 * once the maze does (view.mazeIn), so the hero stays clean. Slow on purpose: nothing here moves faster than a breath.
 */
const N = 520;
const TOP = 3.6;
const BASE = new THREE.Color(PAL.gold).multiplyScalar(1.5);

function disc() {
  const c = document.createElement('canvas');
  c.width = c.height = 32;
  const g = c.getContext('2d')!;
  const grd = g.createRadialGradient(16, 16, 0, 16, 16, 16);
  grd.addColorStop(0, 'rgba(255,255,255,1)');
  grd.addColorStop(0.45, 'rgba(255,255,255,0.45)');
  grd.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grd;
  g.fillRect(0, 0, 32, 32);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

const smooth = (a: number, b: number, x: number) => {
  const t = Math.max(0, Math.min(1, (x - a) / (b - a)));
  return t * t * (3 - 2 * t);
};

export default function Dust() {
  const mat = useRef<THREE.PointsMaterial>(null!);
  const data = useMemo(() => {
    const pos = new Float32Array(N * 3);
    const col = new Float32Array(N * 3);
    const seed = new Float32Array(N * 3);
    let s = 1234567; // a fixed seed: the same dust every load
    const rnd = () => (s = (s * 16807) % 2147483647) / 2147483647;
    for (let i = 0; i < N; i++) {
      pos[i * 3] = (rnd() * 2 - 1) * 4.4;
      pos[i * 3 + 1] = rnd() * TOP;
      pos[i * 3 + 2] = (rnd() * 2 - 1) * 4.4;
      seed[i * 3] = 0.3 + rnd() * 0.9;
      seed[i * 3 + 1] = rnd() * Math.PI * 2;
      seed[i * 3 + 2] = 0.4 + rnd() * 0.8;
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    return { geo, pos, col, seed, map: disc() };
  }, []);

  useFrame((st, dtRaw) => {
    const m = mat.current;
    if (!m) return;
    const a = view.mazeIn;
    m.opacity = 0.5 * a;
    if (a < 0.01) return;
    const dt = Math.min(dtRaw, 0.05);
    const t = st.clock.elapsedTime;
    const { pos, col, seed, geo } = data;
    for (let i = 0; i < N; i++) {
      const k = seed[i * 3];
      const ph = seed[i * 3 + 1];
      const sp = seed[i * 3 + 2];
      pos[i * 3] += Math.sin(t * 0.13 * k + ph) * 0.0014;
      pos[i * 3 + 1] += dt * 0.04 * sp;
      pos[i * 3 + 2] += Math.cos(t * 0.11 * k + ph) * 0.0014;
      if (pos[i * 3 + 1] > TOP) pos[i * 3 + 1] -= TOP;
      const y = pos[i * 3 + 1];
      const f = smooth(0, 0.5, y) * (1 - smooth(TOP - 0.8, TOP, y));
      col[i * 3] = BASE.r * f;
      col[i * 3 + 1] = BASE.g * f;
      col[i * 3 + 2] = BASE.b * f;
    }
    geo.attributes.position.needsUpdate = true;
    geo.attributes.color.needsUpdate = true;
  });

  return (
    <points geometry={data.geo} frustumCulled={false} renderOrder={4}>
      <pointsMaterial ref={mat} map={data.map} vertexColors size={0.05} sizeAttenuation transparent opacity={0} depthWrite={false} blending={THREE.AdditiveBlending} toneMapped={false} />
    </points>
  );
}
