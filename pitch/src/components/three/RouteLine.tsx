'use client';

import { Html } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { view } from '@/lib/choreography';
import { COURSE, ROUTE, enu, routeAt } from '@/lib/course';
import { clamp, damp, lerp } from '@/lib/math';
import { PAL } from '@/lib/palette';
import { GlowLine, type GlowLineHandle } from './GlowLine';

/**
 * RouteLine: skybeta-19's real as-built route (286 waypoints, 14.2 m, from git) as a glowing MeshLine gradient (rust to gold,
 * warm = the route) that extends with the timeline. The head carries
 *   - a stereo FOV cone (callout 02): the OAK-D-S2 looking down the corridor, in ice blue (cool = what the drone perceives)
 *   - the blind-carry beat (callout 03): inside the dark tunnel the camera sees nothing, so the cone goes dark, the head's ring
 *     turns from ice to amber and a tag says it is riding PX4's own estimate (blind_carry, at most 4 s)
 * Nothing here flickers: the head marker breathes slowly and the hand-over to amber is a 0.4 s fade.
 */

const RUST = PAL.rust;
const GOLD = PAL.gold;
const ICE = PAL.ice;

const PTS = ROUTE.waypoints.map((w) => new THREE.Vector3(...enu(w[0], w[1], w[2])));

const T = COURSE.tunnel;

export default function RouteLine() {
  const root = useRef<THREE.Group>(null!);
  const full = useRef<GlowLineHandle>(null);
  const halo = useRef<GlowLineHandle>(null);
  const core = useRef<GlowLineHandle>(null);
  const head = useRef<THREE.Group>(null!);
  const ring = useRef<THREE.Mesh>(null!);
  const ringMat = useRef<THREE.MeshBasicMaterial>(null!);
  const tag = useRef<HTMLDivElement>(null);
  /** 0 = the camera sees, 1 = blind and riding PX4's estimate */
  const blind = useRef(0);

  const coneGeo = useMemo(() => {
    const L = 2.7;
    const r = L * Math.tan((40 * Math.PI) / 180);
    const g = new THREE.ConeGeometry(r, L, 4, 1, true);
    g.rotateY(Math.PI / 4);
    g.translate(0, -L / 2, 0); // apex at origin, axis -Y
    g.rotateX(Math.PI / 2); // axis -> -Z (nose)
    return g;
  }, []);
  const coneMat = useMemo(
    () =>
      new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,
        wireframe: true,
        blending: THREE.AdditiveBlending,
        toneMapped: false,
        uniforms: { uAlpha: { value: 0 }, uColor: { value: new THREE.Color(ICE) } },
        vertexShader: /* glsl */ `varying float vL; void main(){ vL = clamp(-position.z / 2.7, 0.0, 1.0); gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
        fragmentShader: /* glsl */ `uniform float uAlpha; uniform vec3 uColor; varying float vL; void main(){ gl_FragColor = vec4(uColor * 1.3, uAlpha * (1.0 - vL * 0.75)); }`,
      }),
    [],
  );

  useFrame((state, dtRaw) => {
    const g = root.current;
    if (!g) return;
    const show = view.ghostShow;
    g.visible = show > 0.002;
    if (!g.visible) return;
    const dt = Math.min(dtRaw, 0.05);

    // progressive draw: the gradient core + its halo grow with the ghost flight, the faint full route is always there
    const vis = clamp(view.ghostU + 0.012);
    for (const h of [core.current, halo.current]) {
      if (!h) continue;
      h.material.visibility = vis;
      h.mesh.visible = vis > 0.01;
      h.material.opacity = h === halo.current ? 0.14 * show : show;
    }
    if (full.current) full.current.material.opacity = show * 0.22;

    // head marker: breathes once every ~4 s, no more
    const gp = routeAt(view.ghostU);
    const hw = enu(gp.x, gp.y, gp.z);
    head.current.position.set(hw[0], hw[1], hw[2]);
    head.current.rotation.y = gp.heading - Math.PI / 2;
    head.current.scale.setScalar(0.92 + 0.08 * Math.sin(state.clock.elapsedTime * 1.6));
    if (ring.current) ring.current.rotation.z = state.clock.elapsedTime * 0.4;

    // blind carry (callout 03): while the ghost is inside the tunnel the camera tracks nothing. The cone goes dark and the ring
    // turns amber: from here the drone rides PX4's own estimate along the route.
    const inTube = gp.y < T.yMax + 0.5 && gp.y > T.yMin - 0.5 && gp.x > T.xMin && gp.x < T.xMax;
    const target = show * view.calloutMix[2] * (inTube ? 1 : 0) > 0.3 ? 1 : 0;
    blind.current = damp(blind.current, target, 5, dt);
    const k = blind.current;

    // stereo cone: on for callout 02, a faint ghost of it elsewhere, dark while blind
    coneMat.uniforms.uAlpha.value = show * (0.04 + 0.4 * view.calloutMix[1]) * (1 - k);
    if (ringMat.current) ringMat.current.color.setRGB(lerp(1.1, 2.4, k), lerp(1.7, 1.6, k), lerp(2.1, 0.6, k));
    if (tag.current) tag.current.style.opacity = String(clamp(k * 1.2));
  });

  return (
    <group ref={root} visible={false}>
      {/* the whole route, faint, so the destination is always legible */}
      <GlowLine ref={full} points={PTS} width={1.4} from={PAL.sand} to={PAL.sand} boost={1} opacity={0.2} renderOrder={4} />
      {/* travelled part: a soft halo + a warm gradient core (rust at the start, gold at the pad) */}
      <GlowLine ref={halo} points={PTS} width={9} from={RUST} to={GOLD} boost={1.0} opacity={0.14} additive renderOrder={5} />
      <GlowLine ref={core} points={PTS} width={3.2} from={RUST} to={GOLD} boost={1.5} opacity={1} renderOrder={6} />

      <group ref={head}>
        <mesh>
          <sphereGeometry args={[0.055, 16, 16]} />
          <meshBasicMaterial color={[3.4, 3.0, 2.5]} toneMapped={false} />
        </mesh>
        <mesh ref={ring} rotation-x={-Math.PI / 2}>
          <ringGeometry args={[0.15, 0.17, 48]} />
          <meshBasicMaterial ref={ringMat} color={[1.1, 1.7, 2.1]} toneMapped={false} transparent opacity={0.85} side={THREE.DoubleSide} depthWrite={false} />
        </mesh>
        <lineSegments>
          <bufferGeometry>
            <bufferAttribute attach="attributes-position" args={[new Float32Array([-0.22, 0, -0.22, 0.22, 0, 0.22, 0.22, 0, -0.22, -0.22, 0, 0.22]), 3]} />
          </bufferGeometry>
          <lineBasicMaterial color={[1.6, 1.45, 1.2]} toneMapped={false} transparent opacity={0.85} />
        </lineSegments>
        {/* altitude plumb line */}
        <lineSegments>
          <bufferGeometry>
            <bufferAttribute attach="attributes-position" args={[new Float32Array([0, 0, 0, 0, -0.85, 0]), 3]} />
          </bufferGeometry>
          <lineBasicMaterial color="#f3c98f" transparent opacity={0.35} toneMapped={false} />
        </lineSegments>
        <mesh geometry={coneGeo} material={coneMat} renderOrder={5} />
        {/* blind carry: the tag that appears while the camera sees nothing */}
        <Html center position={[0, 0.55, 0]} zIndexRange={[18, 0]} style={{ pointerEvents: 'none' }}>
          <div ref={tag} className="blind-tag" style={{ opacity: 0 }}>
            NO CAMERA · PX4 ESTIMATE · MAX 4 S
          </div>
        </Html>
      </group>
    </group>
  );
}
