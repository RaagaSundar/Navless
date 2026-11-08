'use client';

import { useFrame } from '@react-three/fiber';
import { use, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { ASSETS } from '@/config/assets';
import { view } from '@/lib/choreography';
import { START, routeAt } from '@/lib/course';
import { holo } from './holo';
import { mulberry32 } from '@/lib/math';

/**
 * The phone-LiDAR stage of the story: a point cloud that floods outward from the launch mark and then lets the wireframe
 * walls rise through it. Later, while the ghost drone flies, every point inside its stereo cone lights up ice-blue,
 * which is what "stereo odometry tracks features on the walls" looks like.
 *
 * Honesty note: the raw Scaniverse scan (596k points) isn't in git, only the walls fitted to it. This cloud is a
 * LiDAR-style resample of the as-built map (scripts/obj-to-glb.mjs). The HUD labels it that way.
 */

const cache = new Map<string, Promise<ArrayBuffer>>();
function loadBinary(url: string) {
  let p = cache.get(url);
  if (!p) {
    // register with three's loading manager so the HUD loader waits for this file too
    THREE.DefaultLoadingManager.itemStart(url);
    p = fetch(url)
      .then((r) => {
        if (!r.ok) throw new Error(`${url}: ${r.status}`);
        return r.arrayBuffer();
      })
      .finally(() => THREE.DefaultLoadingManager.itemEnd(url));
    cache.set(url, p);
  }
  return p;
}
if (typeof window !== 'undefined' && ASSETS.scan) void loadBinary(ASSETS.scan);

export default function ScanCloud() {
  if (!ASSETS.scan) return null;
  return <Cloud url={ASSETS.scan} />;
}

const _p = new THREE.Vector3();
const _d = new THREE.Vector3();

function Cloud({ url }: { url: string }) {
  const buf = use(loadBinary(url));
  const ref = useRef<THREE.Points>(null!);

  const { geometry, material } = useMemo(() => {
    const dv = new DataView(buf);
    if (dv.getUint32(0, true) !== 0x4e4c5343) throw new Error('course-scan.bin: bad magic');
    const count = dv.getUint32(4, true);
    const pos = new Float32Array(buf, 8, count * 3);
    const kinds = new Uint8Array(buf, 8 + count * 12, count);
    const rnd = mulberry32(7);
    const kind = new Float32Array(count);
    const seed = new Float32Array(count);
    for (let i = 0; i < count; i++) {
      kind[i] = kinds[i];
      seed[i] = rnd();
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('aKind', new THREE.BufferAttribute(kind, 1));
    g.setAttribute('aSeed', new THREE.BufferAttribute(seed, 1));
    g.computeBoundingSphere();

    const m = new THREE.ShaderMaterial({
      transparent: true,
      depthWrite: false,
      blending: THREE.AdditiveBlending,
      toneMapped: false,
      uniforms: {
        uTime: holo.uTime,
        uAccent: holo.uAccent,
        uReveal: { value: 0 },
        uAlpha: { value: 0 },
        uScale: { value: 700 },
        uOrigin: { value: new THREE.Vector2(START[0], -START[1]) },
        uConePos: { value: new THREE.Vector3() },
        uConeDir: { value: new THREE.Vector3(0, 0, -1) },
        uConeCos: { value: Math.cos((40 * Math.PI) / 180) },
        uConeOn: { value: 0 },
        uTubeDim: { value: 0 },
      },
      vertexShader: /* glsl */ `
        attribute float aKind; attribute float aSeed;
        uniform float uTime; uniform float uReveal; uniform float uScale; uniform vec2 uOrigin;
        uniform vec3 uConePos; uniform vec3 uConeDir; uniform float uConeCos; uniform float uConeOn; uniform float uTubeDim;
        varying float vA; varying float vKind; varying float vHot;
        void main(){
          vec3 p = position;
          float d = length(p.xz - uOrigin) + p.y * 0.35;
          float R = uReveal * 17.0;
          float wave = smoothstep(d - 2.6, d, R);
          float front = exp(-pow((d - R) * 1.5, 2.0)) * step(0.001, uReveal);
          vec3 rel = p - uConePos;
          float dist = max(length(rel), 1e-3);
          float inCone = step(uConeCos, dot(rel / dist, uConeDir)) * (1.0 - smoothstep(1.4, 4.6, dist));
          vHot = inCone * uConeOn;
          // no per-point twinkle (thousands of points blinking at random reads as static noise): a very slow, very small swell
          float tw = 0.9 + 0.1 * sin(uTime * 0.35 + aSeed * 60.0);
          vA = wave * (0.55 * tw) + front * 0.7 + vHot * 1.0;
          if (aKind == 3.0) vA *= mix(1.0, 0.1, uTubeDim);
          vKind = aKind;
          vec4 mv = modelViewMatrix * vec4(p, 1.0);
          float size = 0.014 + 0.018 * front + 0.03 * vHot;
          gl_PointSize = clamp(size * uScale / -mv.z, 1.0, 8.0);
          gl_Position = projectionMatrix * mv;
        }
      `,
      fragmentShader: /* glsl */ `
        uniform vec3 uAccent; uniform float uAlpha;
        varying float vA; varying float vKind; varying float vHot;
        void main(){
          vec2 c = gl_PointCoord - 0.5;
          float r = length(c) * 2.0;
          if (r > 1.0) discard;
          float core = smoothstep(1.0, 0.0, r);
          vec3 base = mix(vec3(1.0, 0.93, 0.82), uAccent, vKind == 2.0 ? 0.25 : 0.65);
          float dim = vKind == 0.0 ? 0.55 : (vKind == 1.0 ? 0.7 : 1.0);
          // points inside the stereo cone light up ice-blue: the drone is looking at them
          vec3 col = mix(base, vec3(0.61, 0.78, 0.87) * 1.9, clamp(vHot, 0.0, 1.0));
          gl_FragColor = vec4(col, core * core * vA * dim * uAlpha);
        }
      `,
    });
    return { geometry: g, material: m };
  }, [buf]);

  useFrame((state) => {
    const pts = ref.current;
    if (!pts) return;
    // the cloud is the first thing the map shows; it stays faintly alive afterwards
    const show = view.scanIn;
    pts.visible = show > 0.003;
    if (!pts.visible) return;
    const u = material.uniforms;
    u.uReveal.value = THREE.MathUtils.smoothstep(view.p, 0.34, 0.5);
    u.uAlpha.value = show;
    const cam = state.camera as THREE.PerspectiveCamera;
    u.uScale.value = (state.size.height * state.viewport.dpr) / (2 * Math.tan((cam.fov * Math.PI) / 360));
    // stereo cone follows the ghost drone
    const g = routeAt(view.ghostU);
    _p.set(g.x, g.z, -g.y);
    _d.set(Math.cos(g.heading), 0, -Math.sin(g.heading));
    u.uConePos.value.copy(_p);
    u.uConeDir.value.copy(_d);
    u.uConeOn.value = view.ghostShow * (0.25 + 0.75 * view.calloutMix[1]);
    u.uTubeDim.value = view.calloutMix[2];
  });

  return <points ref={ref} geometry={geometry} material={material} frustumCulled={false} renderOrder={3} visible={false} />;
}
