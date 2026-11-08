'use client';

import { Html, useGLTF } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { ASSETS } from '@/config/assets';
import { view } from '@/lib/choreography';
import { enu } from '@/lib/course';
import { clamp, damp, dampAngle, lerp, smoothstep } from '@/lib/math';
import { flight } from '@/lib/store';
import { createDroneUniforms, createRoleMaterials } from './three/droneMaterials';
import { buildRotor, type Rotor } from './three/rotor';

useGLTF.preload(ASSETS.drone);

/** hero scale: the 0.356 m airframe becomes a ~2 m monument at the origin */
const HERO_SCALE = 8.6;
/** where the corner badge sits, as a fraction of the half-viewport, and how deep in front of the camera */
// depth 1.2: closer to the camera than any wall can get, so the badge is never occluded by the maze
const CORNER = { x: 0.78, y: 0.66, depth: 1.2 };
// the golden rim on the airframe: sun-gold in the tour, a touch hotter in override
const RIM_TOUR = new THREE.Color('#ffb868');
const RIM_OVERRIDE = new THREE.Color('#ff9a5c');

const _p = new THREE.Vector3();
const _q = new THREE.Quaternion();
const _e = new THREE.Euler();
const _s = new THREE.Vector3();
const _heroQ = new THREE.Quaternion();
const _cornerQ = new THREE.Quaternion();
const _startQ = new THREE.Quaternion();
const _heroP = new THREE.Vector3();
const _cornerP = new THREE.Vector3();
const _startP = new THREE.Vector3();

interface Tag {
  key: string;
  label: string;
  sub: string;
  dx: number;
  dy: number;
  pos: THREE.Vector3;
}

/**
 * DroneModel: the Sky Beta CAD as one continuous character.
 *
 * It is a single object that blends between three anchors as you scroll:
 *   HERO    world origin, 5.4x, turning 360 deg on scroll
 *   CORNER  camera-attached badge, top right, slow idle spin
 *   START   world-anchored at the launch mark, true scale, props idling (OVERRIDE MODE); follows `flight` when engaged
 */
export default function DroneModel() {
  // ⟦ DRONE CAD SLOT ⟧ swap ASSETS.drone for your own .glb / .gltf (see src/config/assets.ts)
  const { scene } = useGLTF(ASSETS.drone);
  const u = useMemo(createDroneUniforms, []);
  const mats = useMemo(() => createRoleMaterials(u), [u]);

  const model = useMemo(() => {
    const root = scene.clone(true);
    const rotors: Rotor[] = [];
    const tags: Tag[] = [];
    const mounts: THREE.Object3D[] = [];
    root.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (mesh.isMesh) {
        const role = (mesh.userData.role as string | undefined) ?? 'chrome';
        mesh.material = mats[role] ?? mats.chrome;
        mesh.frustumCulled = true;
      }
      if (o.name.startsWith('prop_')) mounts.push(o);
    });
    for (const m of mounts) {
      const dir = (m.userData.dir as number) ?? 1;
      const radius = (m.userData.radius as number) ?? 0.0775;
      const r = buildRotor(radius, dir, mats.arm, mats.chrome, u);
      m.add(r.group);
      rotors.push(r);
    }
    // three labels, not five: the camera, the computer, the flight controller
    const want: [string, string, number, number][] = [
      ['anchor_camera', 'OAK-D-S2', -130, -70],
      ['anchor_jetson', 'JETSON ORIN NANO', 150, 60],
      ['anchor_pixhawk', 'PIXHAWK 6C', -150, 30],
    ];
    const subs: Record<string, string> = {
      anchor_camera: 'stereo cameras, 75 mm apart',
      anchor_jetson: 'onboard computer',
      anchor_pixhawk: 'flight controller (PX4)',
    };
    for (const [name, label, dx, dy] of want) {
      const n = root.getObjectByName(name);
      if (n) tags.push({ key: name, label, sub: subs[name], dx, dy, pos: n.position.clone() });
    }
    return { root, rotors, tags };
  }, [scene, mats, u]);

  useEffect(() => {
    return () => {
      for (const r of model.rotors) {
        r.disc.geometry.dispose();
        r.disc.material.dispose();
      }
    };
  }, [model]);

  const anchor = useRef<THREE.Group>(null!);
  const tagEls = useRef<(HTMLDivElement | null)[]>([]);
  const state = useRef({ yaw: 0, bank: 0, omega: 10, startYaw: 0 });

  useFrame((s, dtRaw) => {
    const dt = Math.min(dtRaw, 0.05);
    const t = s.clock.elapsedTime;
    const g = anchor.current;
    if (!g) return;
    const cam = s.camera as THREE.PerspectiveCamera;
    const st = state.current;

    g.visible = view.droneReveal > 0.002;
    // hero HUD tags (DOM, written directly: no React renders). Done first so they also clear when the drone is hidden.
    const els = tagEls.current;
    for (let i = 0; i < els.length; i++) {
      const el = els[i];
      if (el) el.style.opacity = g.visible ? String(clamp(view.hudIn * 1.5 - i * 0.14)) : '0';
    }
    if (!g.visible) return;

    // ───── shared uniforms
    u.uTime.value = t;
    u.uReveal.value = view.droneReveal;
    u.uDroneInv.value.copy(g.matrixWorld).invert();
    u.uAccent.value.lerpColors(RIM_TOUR, RIM_OVERRIDE, view.o);
    u.uRim.value = lerp(0.2, 0.3, view.o);

    // ───── HERO anchor
    const yawHero = view.heroYaw;
    _heroP.set(0, Math.sin(t * 0.9) * 0.05, 0);
    _e.set(0.2 + Math.sin(t * 0.55) * 0.035, yawHero, Math.sin(t * 0.43) * 0.05 - view.vel * 0.05, 'YXZ');
    _heroQ.setFromEuler(_e);
    const heroS = HERO_SCALE * (0.94 + view.droneReveal * 0.06);

    // ───── CORNER anchor: camera-attached badge
    cam.updateMatrixWorld();
    const halfH = Math.tan(THREE.MathUtils.degToRad(cam.fov / 2)) * CORNER.depth;
    const halfW = halfH * cam.aspect;
    _cornerP.set(CORNER.x * halfW, CORNER.y * halfH, -CORNER.depth).applyMatrix4(cam.matrixWorld);
    st.yaw += dt * 0.55;
    _e.set(0.62, st.yaw, 0.0, 'YXZ');
    _cornerQ.copy(cam.quaternion).multiply(_q.setFromEuler(_e));
    const cornerS = (0.22 * halfH * 2) / 0.36; // body ~22% of the visible height

    // ───── START anchor: world, true scale (a touch larger so the props read)
    const hover = flight.z + Math.sin(t * 2.4) * (flight.mode === 'flying' ? 0.012 : 0.02);
    const e = enu(flight.x, flight.y, hover);
    _startP.set(e[0], e[1], e[2]);
    st.startYaw = dampAngle(st.startYaw, flight.heading - Math.PI / 2, 8, dt);
    st.bank = damp(st.bank, flight.mode === 'flying' ? -0.12 : 0, 3, dt);
    _e.set(flight.mode === 'flying' ? 0.1 : 0.0, st.startYaw, st.bank + Math.sin(t * 1.7) * 0.01, 'YXZ');
    _startQ.setFromEuler(_e);
    const startS = 1.35;

    // ───── blend
    const wH = view.heroW;
    const wC = view.cornerW;
    const wS = view.startW;
    g.position.set(0, 0, 0).addScaledVector(_heroP, wH).addScaledVector(_cornerP, wC).addScaledVector(_startP, wS);
    const sc = Math.exp(wH * Math.log(heroS) + wC * Math.log(cornerS) + wS * Math.log(startS));
    g.scale.setScalar(sc);
    const hc = wH + wC;
    _q.copy(_heroQ).slerp(_cornerQ, hc > 1e-5 ? wC / hc : 0).slerp(_startQ, wS);
    g.quaternion.copy(_q);
    void _s;

    // ───── rotors
    const heroOmega = 12 + view.vel * 80; // spins up a little with scroll, never a blur
    const startOmega = flight.mode === 'flying' ? 100 : 52;
    const target = heroOmega * wH + 42 * wC + startOmega * wS;
    st.omega = damp(st.omega, target, 5, dt);
    const discA = clamp(st.omega / 70) * smoothstep(0.55, 0.95, view.droneReveal);
    for (const r of model.rotors) {
      r.group.rotation.y += r.dir * st.omega * dt;
      r.disc.material.uniforms.uOmega.value = discA;
    }

  });

  return (
    <group ref={anchor} visible={false}>
      {/* ⟦ DRONE CAD SLOT ⟧ the restyled CAD is injected here */}
      <primitive object={model.root} />
      {model.tags.map((tag, i) => (
        <group key={tag.key} position={tag.pos}>
          <Html center zIndexRange={[20, 0]} style={{ pointerEvents: 'none' }}>
            <div
              ref={(el) => {
                tagEls.current[i] = el;
              }}
              className="hud-tag"
              style={{ opacity: 0, transform: `translate(${tag.dx}px, ${tag.dy}px)` }}
            >
              <svg className="hud-leader" width="1" height="1" overflow="visible" aria-hidden>
                <line x1={0} y1={0} x2={-tag.dx} y2={-tag.dy} />
                <circle cx={-tag.dx} cy={-tag.dy} r="3" />
              </svg>
              <div className="hud-tag-box brackets">
                <div className="hud-label">{tag.label}</div>
                <div className="hud-value">{tag.sub}</div>
              </div>
            </div>
          </Html>
        </group>
      ))}
    </group>
  );
}
