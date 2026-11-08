'use client';

import { useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { view } from '@/lib/choreography';
import { mulberry32, smoothstep } from '@/lib/math';

/**
 * cuVSLAM-style feature cloud around the hero drone: a couple of thousand sparse points on a virtual room
 * (floor, a ring of walls, clutter) that sweep in as a radial wave and turn slowly. A few percent are "tracked" features,
 * drawn larger in ice blue (cool = what the drone perceives). Purely illustrative, and static apart from the slow turn:
 * no twinkle, nothing that flickers.
 */
const COUNT = 700;

export default function FeatureCloud() {
  const ref = useRef<THREE.Points>(null!);

  const { geometry, material } = useMemo(() => {
    const rnd = mulberry32(1107);
    const pos = new Float32Array(COUNT * 3);
    const seed = new Float32Array(COUNT);
    const track = new Float32Array(COUNT);
    for (let i = 0; i < COUNT; i++) {
      const m = rnd();
      let x: number, y: number, z: number;
      if (m < 0.38) {
        // floor
        const r = 1.4 + Math.sqrt(rnd()) * 7;
        const a = rnd() * Math.PI * 2;
        x = Math.cos(a) * r;
        z = Math.sin(a) * r;
        y = -2.2 + (rnd() - 0.5) * 0.06;
      } else if (m < 0.78) {
        // ring of walls, with doorway-like gaps
        const a = rnd() * Math.PI * 2;
        const gap = Math.sin(a * 3.0) > 0.78;
        const r = (gap ? 8.2 : 6.6) + (rnd() - 0.5) * 0.12;
        x = Math.cos(a) * r;
        z = Math.sin(a) * r;
        y = -2.2 + Math.pow(rnd(), 0.8) * 5.4;
      } else {
        // clutter: tight clusters standing on the floor
        const c = Math.floor(rnd() * 9);
        const ca = (c / 9) * Math.PI * 2 + 0.4;
        const cr = 3.2 + (c % 3) * 1.1;
        x = Math.cos(ca) * cr + (rnd() - 0.5) * 0.9;
        z = Math.sin(ca) * cr + (rnd() - 0.5) * 0.9;
        y = -2.2 + rnd() * 1.6;
      }
      pos[i * 3] = x;
      pos[i * 3 + 1] = y;
      pos[i * 3 + 2] = z;
      seed[i] = rnd();
      track[i] = rnd() < 0.045 ? 1 : 0;
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('aSeed', new THREE.BufferAttribute(seed, 1));
    g.setAttribute('aTrack', new THREE.BufferAttribute(track, 1));
    g.computeBoundingSphere();
    if (g.boundingSphere) g.boundingSphere.radius = 12;

    const m = new THREE.ShaderMaterial({
      transparent: true,
      depthWrite: false,
      blending: THREE.AdditiveBlending,
      toneMapped: false,
      uniforms: {
        uTime: { value: 0 },
        uReveal: { value: 0 },
        uAlpha: { value: 0 },
        uScale: { value: 600 },
        uNear: { value: new THREE.Color('#f3dfc4') },
        uFar: { value: new THREE.Color('#a45f46') },
        uTrack: { value: new THREE.Color('#9cc6dd') },
      },
      vertexShader: /* glsl */ `
        attribute float aSeed; attribute float aTrack;
        uniform float uTime; uniform float uReveal; uniform float uScale;
        varying float vA; varying float vTrack; varying float vDepth;
        void main(){
          vec3 p = position;
          p += 0.012 * vec3(sin(uTime * 0.3 + aSeed * 30.0), cos(uTime * 0.25 + aSeed * 20.0), sin(uTime * 0.2 + aSeed * 11.0));
          float d = length(p);
          float wave = smoothstep(d - 1.6, d, uReveal * 10.5);
          vA = wave * 0.85;
          vTrack = aTrack;
          vDepth = clamp(d / 8.5, 0.0, 1.0);
          vec4 mv = modelViewMatrix * vec4(p, 1.0);
          float size = (0.020 + aTrack * 0.030) * (0.6 + 0.8 * wave);
          gl_PointSize = clamp(size * uScale / -mv.z, 1.0, 9.0);
          gl_Position = projectionMatrix * mv;
        }
      `,
      fragmentShader: /* glsl */ `
        uniform vec3 uNear; uniform vec3 uFar; uniform vec3 uTrack; uniform float uAlpha;
        varying float vA; varying float vTrack; varying float vDepth;
        void main(){
          vec2 c = gl_PointCoord - 0.5;
          float r = length(c) * 2.0;
          if (r > 1.0) discard;
          float core = smoothstep(1.0, 0.0, r);
          float ring = smoothstep(0.62, 0.78, r) * (1.0 - smoothstep(0.86, 1.0, r));
          vec3 col = mix(uNear, uFar, vDepth);
          float a = mix(core * core, ring + core * 0.25, vTrack);
          col = mix(col, uTrack, vTrack);
          gl_FragColor = vec4(col * (1.0 + vTrack * 1.3), a * vA * uAlpha * (1.0 - vDepth * 0.35));
        }
      `,
    });
    return { geometry: g, material: m };
  }, []);

  useFrame((state) => {
    const pts = ref.current;
    if (!pts) return;
    const cloudIn = smoothstep(0.1, 0.3, view.p) * (1 - smoothstep(0.33, 0.43, view.p));
    pts.visible = cloudIn > 0.002;
    if (!pts.visible) return;
    const u = material.uniforms;
    u.uTime.value = state.clock.elapsedTime;
    u.uReveal.value = smoothstep(0.1, 0.3, view.p);
    u.uAlpha.value = cloudIn * 0.7;
    const cam = state.camera as THREE.PerspectiveCamera;
    u.uScale.value = (state.size.height * state.viewport.dpr) / (2 * Math.tan((cam.fov * Math.PI) / 360));
    pts.rotation.y = state.clock.elapsedTime * 0.04 + view.heroYaw * 0.12;
  });

  return <points ref={ref} geometry={geometry} material={material} frustumCulled={false} renderOrder={2} visible={false} />;
}
