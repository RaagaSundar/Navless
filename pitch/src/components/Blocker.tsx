'use client';

import { Html } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { memo, useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { kick } from '@/lib/choreography';
import { WALL_H } from '@/lib/course';
import { clamp, smoothstep } from '@/lib/math';
import { PAL } from '@/lib/palette';
import { DEFAULT_CONFIG } from '@/lib/planner';
import { useDeck, type Obstacle } from '@/lib/store';

/**
 * Blocker: the obstacle a judge drops in OVERRIDE MODE.
 *
 * It is the one thing in the scene that reads as a warning, so it is the only vermilion object on screen. It falls in about
 * half a second, fading in as it drops, settles with one soft ring on the floor, and then simply stands there: a steady
 * translucent volume with clean sand edges, and a flat keep-out footprint on the ground (the planner's inflation).
 * Deliberately NOT here: glitching, tearing, flicker, sparks, a sky beam, screen flashes. Nothing about it blinks.
 */
const H = WALL_H;
const INFL = DEFAULT_CONFIG.inflation;
const DROP_S = 0.46;
const DANGER = new THREE.Color(PAL.danger);

export const Blocker = memo(function Blocker({ o }: { o: Obstacle }) {
  // drei's <Html> ignores a parent's `visible={false}`: the tag must be dropped from the tree when override is off,
  // or it floats over the hero / tour after the judge exits
  const override = useDeck((s) => s.override);
  const group = useRef<THREE.Group>(null!);
  const ring = useRef<THREE.Mesh>(null!);
  const halo = useRef<THREE.Mesh>(null!);
  const impacted = useRef(false);

  const bodyGeo = useMemo(() => new THREE.BoxGeometry(o.hw * 2, H, o.hd * 2), [o.hw, o.hd]);
  const edgeGeo = useMemo(() => new THREE.EdgesGeometry(bodyGeo), [bodyGeo]);

  const bodyMat = useMemo(
    () =>
      new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
        toneMapped: false,
        uniforms: { uIn: { value: 0 }, uColor: { value: DANGER.clone() } },
        vertexShader: /* glsl */ `
          varying vec3 vN; varying vec3 vV; varying float vY;
          void main(){
            vec4 wp = modelMatrix * vec4(position, 1.0);
            vN = normalize(mat3(modelMatrix) * normal);
            vV = cameraPosition - wp.xyz;
            vY = position.y / ${H.toFixed(4)} + 0.5;
            gl_Position = projectionMatrix * viewMatrix * wp;
          }
        `,
        fragmentShader: /* glsl */ `
          uniform float uIn; uniform vec3 uColor;
          varying vec3 vN; varying vec3 vV; varying float vY;
          void main(){
            float fres = pow(1.0 - abs(dot(normalize(vN), normalize(vV))), 1.8);
            vec3 col = vec3(0.10, 0.03, 0.02) + uColor * (0.5 * fres + 0.2 + 0.45 * smoothstep(0.9, 1.0, vY));
            float a = (0.28 + 0.45 * fres + 0.18 * smoothstep(0.85, 1.0, vY)) * uIn;
            gl_FragColor = vec4(col, clamp(a, 0.0, 1.0));
          }
        `,
      }),
    [],
  );
  const edgeMat = useMemo(() => new THREE.LineBasicMaterial({ color: new THREE.Color(PAL.sand).multiplyScalar(1.25), toneMapped: false, transparent: true, opacity: 0 }), []);
  const capMat = useMemo(() => new THREE.MeshBasicMaterial({ color: new THREE.Color(PAL.danger).multiplyScalar(1.3), toneMapped: false, transparent: true, opacity: 0, depthWrite: false }), []);

  // the keep-out footprint on the ground: a flat fill with a clean outline (no hatching, no motion)
  const haloMat = useMemo(
    () =>
      new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,
        toneMapped: false,
        uniforms: { uB: { value: o.hw + INFL }, uIn: { value: 0 }, uHw: { value: o.hw } },
        vertexShader: /* glsl */ `varying vec2 vP; void main(){ vP = position.xy; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
        fragmentShader: /* glsl */ `
          uniform float uB; uniform float uIn; uniform float uHw;
          varying vec2 vP;
          float sdBox(vec2 p, float b, float r){ vec2 d = abs(p) - vec2(b - r); return length(max(d, 0.0)) + min(max(d.x, d.y), 0.0) - r; }
          void main(){
            float d = sdBox(vP, uB, 0.12);
            if (d > 0.01) discard;
            float edge = 1.0 - smoothstep(0.0, 0.025, abs(d));
            float band = step(uHw, max(abs(vP.x), abs(vP.y)));   // the keep-out band, not the blocker itself
            float a = (0.07 + 0.06 * band + edge * 0.55) * uIn;
            gl_FragColor = vec4(vec3(1.0, 0.36, 0.24) * 1.4, a);
          }
        `,
      }),
    [o.hw],
  );

  // one soft ring when it lands
  const ringMat = useMemo(
    () =>
      new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
        toneMapped: false,
        side: THREE.DoubleSide,
        uniforms: { uA: { value: 0 } },
        vertexShader: /* glsl */ `void main(){ gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
        fragmentShader: /* glsl */ `uniform float uA; void main(){ gl_FragColor = vec4(vec3(1.0, 0.36, 0.24) * 1.4, uA); }`,
      }),
    [],
  );

  useEffect(() => {
    return () => {
      bodyGeo.dispose();
      edgeGeo.dispose();
      bodyMat.dispose();
      edgeMat.dispose();
      capMat.dispose();
      haloMat.dispose();
      ringMat.dispose();
    };
  }, [bodyGeo, edgeGeo, bodyMat, edgeMat, capMat, haloMat, ringMat]);

  useFrame(() => {
    const g = group.current;
    if (!g) return;
    const t = (performance.now() - o.born) / 1000;
    const fall = clamp(t / DROP_S);
    g.position.set(o.x, H / 2 + (1 - fall * fall) * 7.5, -o.y); // accelerating drop, no bounce
    g.visible = true;

    if (fall >= 1 && !impacted.current) {
      impacted.current = true;
      kick(0.12); // a barely-there nudge, not a shake
    }
    const since = Math.max(0, t - DROP_S);

    // everything eases in; nothing flashes
    const fadeIn = smoothstep(0, 0.4, t);
    bodyMat.uniforms.uIn.value = fadeIn;
    edgeMat.opacity = fadeIn * 0.9;
    capMat.opacity = fadeIn * 0.4;

    if (halo.current) {
      halo.current.visible = fall >= 1;
      haloMat.uniforms.uIn.value = smoothstep(0, 0.6, since);
    }
    const r = ring.current;
    if (r) {
      r.visible = since > 0 && since < 1.1;
      if (r.visible) {
        const k = since / 1.1;
        const rad = 0.25 + k * 2.2;
        r.scale.set(rad, rad, 1);
        ringMat.uniforms.uA.value = (1 - k) * (1 - k) * 0.4;
      }
    }
  });

  return (
    <>
      <mesh ref={halo} position={[o.x, 0.012, -o.y]} rotation-x={-Math.PI / 2} material={haloMat} visible={false} renderOrder={7}>
        <planeGeometry args={[(o.hw + INFL) * 2 + 0.1, (o.hw + INFL) * 2 + 0.1]} />
      </mesh>
      <mesh ref={ring} position={[o.x, 0.03, -o.y]} rotation-x={-Math.PI / 2} material={ringMat} visible={false} renderOrder={8}>
        <ringGeometry args={[0.93, 1.0, 72]} />
      </mesh>
      <group ref={group} position={[o.x, H / 2 + 7.5, -o.y]} visible={false}>
        <mesh geometry={bodyGeo} material={bodyMat} renderOrder={9} />
        <lineSegments geometry={edgeGeo} material={edgeMat} renderOrder={10} />
        {/* soft top cap */}
        <mesh position={[0, H / 2 + 0.003, 0]} rotation-x={-Math.PI / 2} material={capMat}>
          <planeGeometry args={[o.hw * 2, o.hd * 2]} />
        </mesh>
        {override ? (
          <Html position={[0, H / 2 + 0.28, 0]} center zIndexRange={[22, 0]} style={{ pointerEvents: 'none' }}>
            <div className="obs-tag">OBSTACLE #{o.id}</div>
          </Html>
        ) : null}
      </group>
    </>
  );
});
