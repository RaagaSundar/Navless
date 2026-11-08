'use client';

import { Html } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { memo, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { RUN_NOTES } from '@/data/facts';
import { camOverride } from '@/lib/choreography';
import { COURSE, ROUTE, WALL_H, enu } from '@/lib/course';
import { EVIDENCE } from '@/lib/evidence';
import { clamp, dampAngle, easeOutCubic } from '@/lib/math';
import { PAL } from '@/lib/palette';
import { replay, sampleRun, useReplay } from '@/lib/replay';
import { flight } from '@/lib/store';
import { GlowLine, type GlowLineHandle } from './three/GlowLine';

/**
 * FlightReplay: the 3D side of REPLAY THE REAL FLIGHTS (lib/replay.ts).
 *
 * The demo drone flies the track skybeta-19's flight recorder says it BELIEVED; an ice ribbon draws behind it, the gold dashes are
 * the route we planned, a chase camera rides behind. At the first impact in the log the camera eases back, a ring spreads from
 * the touch and the wall it hit lights up in vermilion (the one colour we keep for obstacles). Slow fades only: nothing flashes.
 */
const ICE = '#7fb8d9';
const MIST = '#c3e1f1';
const HALF = COURSE.net.halfX;
const DANGER = new THREE.Color(PAL.danger);
const ROUTE_PTS = ROUTE.waypoints.map((w) => new THREE.Vector3(...enu(w[0], w[1], w[2])));

/** the wall the log says it hit lights up in vermilion, slowly, after the touch: a light wash and a bright strip along its foot */
function HitHighlight({ hit, T }: { hit: 'MD' | 'north net' | 'west net' | null; T: number }) {
  const md = COURSE.walls.find((w) => w.name === 'MD');
  const wash = useMemo(() => new THREE.MeshBasicMaterial({ color: DANGER.clone().multiplyScalar(1.2), toneMapped: false, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide }), []);
  const foot = useMemo(() => new THREE.MeshBasicMaterial({ color: DANGER.clone().multiplyScalar(2.2), toneMapped: false, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide }), []);
  useFrame(() => {
    const k = clamp((replay.t - T) / 1.4);
    wash.opacity = 0.13 * k;
    foot.opacity = 0.9 * k;
  });
  if (hit === 'MD' && md) {
    const [ax, ay] = md.a;
    const [bx, by] = md.b;
    const len = Math.hypot(bx - ax, by - ay);
    const rot = Math.atan2(by - ay, bx - ax);
    return (
      <group>
        <mesh position={enu((ax + bx) / 2, (ay + by) / 2, WALL_H / 2)} rotation-y={rot} material={wash}>
          <boxGeometry args={[len, WALL_H, md.thickness * 1.5]} />
        </mesh>
        <mesh position={enu((ax + bx) / 2, (ay + by) / 2, 0.03)} rotation-y={rot} material={foot}>
          <boxGeometry args={[len, 0.06, md.thickness * 2.2]} />
        </mesh>
      </group>
    );
  }
  if (hit === 'north net') {
    return (
      <group>
        <mesh position={[0, WALL_H / 2, -HALF]} material={wash}>
          <planeGeometry args={[2 * HALF, WALL_H]} />
        </mesh>
        <mesh position={[0, 0.03, -HALF]} material={foot}>
          <boxGeometry args={[2 * HALF, 0.06, 0.1]} />
        </mesh>
      </group>
    );
  }
  if (hit === 'west net') {
    return (
      <group>
        <mesh position={[-HALF, WALL_H / 2, 0]} rotation-y={Math.PI / 2} material={wash}>
          <planeGeometry args={[2 * HALF, WALL_H]} />
        </mesh>
        <mesh position={[-HALF, 0.03, 0]} material={foot}>
          <boxGeometry args={[0.1, 0.06, 2 * HALF]} />
        </mesh>
      </group>
    );
  }
  return null;
}

const Replay = memo(function Replay({ run }: { run: number }) {
  const data = EVIDENCE.runs[run];
  const note = RUN_NOTES[data.id];
  const T = data.tContact;
  const pts = useMemo(() => data.points.map((p) => new THREE.Vector3(...enu(p[1] as number, p[2] as number, Math.max(0.02, p[5] as number)))), [data]);
  const sent = useMemo(() => {
    const rows = data.points.filter((p) => p[3] !== null && p[4] !== null);
    return rows.map((p) => new THREE.Vector3(...enu(p[3] as number, p[4] as number, Math.max(0.02, p[5] as number))));
  }, [data]);
  const contact = useMemo(() => enu(data.contact[0], data.contact[1], 0.02), [data]);

  const core = useRef<GlowLineHandle>(null);
  const glow = useRef<GlowLineHandle>(null);
  const setp = useRef<GlowLineHandle>(null);
  const ring = useRef<THREE.Mesh>(null!);
  const ringMat = useRef<THREE.MeshBasicMaterial>(null!);
  const stay = useRef<THREE.MeshBasicMaterial>(null!);
  const camHeading = useRef(0);
  const rate = T < 8 ? 0.5 : 1; // the slide in run 4 is over in 3 s: play it at half speed

  useFrame((_, dtRaw) => {
    if (!replay.active || replay.run !== run) return;
    const dt = Math.min(dtRaw, 0.05);
    replay.t += dt * rate;
    const t = replay.t;
    const tt = clamp(t, 0, T);
    const s = sampleRun(run, tt);
    const after = Math.max(0, t - T);

    // the demo drone becomes the real one: pose from the log
    flight.mode = 'flying';
    flight.x = s.x;
    flight.y = s.y;
    flight.heading = dampAngle(flight.heading, s.heading, 9, dt);
    // after the touch it settles to the floor
    const fall = easeOutCubic(clamp(after / 1.5));
    flight.z = s.z * (1 - fall) + 0.1 * fall;
    flight.speed = after > 0 ? 0 : Math.hypot(s.vx, s.vy);

    // ribbons draw up to the drone
    const frac = clamp(tt / T);
    for (const h of [core.current, glow.current, setp.current]) if (h) h.material.visibility = frac;

    // chase camera: behind and above while it flies, easing back after the touch
    if (after <= 0) camHeading.current = flight.heading; // frozen at the touch: the log's heading spins after it
    const k = easeOutCubic(clamp(after / 2.2));
    const back = 2.3 + 1.5 * k;
    const up = 1.5 + 1.3 * k;
    const e = enu(flight.x, flight.y, flight.z);
    const cx = Math.cos(camHeading.current);
    const sy = Math.sin(camHeading.current);
    camOverride.on = true;
    camOverride.fov = 34;
    camOverride.pos.set(e[0] - cx * back, e[1] + up, e[2] + sy * back);
    camOverride.look.set(e[0] + cx * 0.5, e[1] - 0.05, e[2] - sy * 0.5);

    // the touch
    if (after > 0 && !replay.hit) {
      replay.hit = true;
      useReplay.setState({ hit: true });
    }
    const grow = easeOutCubic(clamp(after / 2.4));
    if (ring.current) {
      ring.current.scale.setScalar(1 + 2.4 * grow);
      ring.current.visible = after > 0;
    }
    if (ringMat.current) ringMat.current.opacity = after > 0 ? 0.85 * (1 - grow) : 0;
    if (stay.current) stay.current.opacity = 0.5 * clamp(after / 0.8);
  });

  return (
    <group>
      <GlowLine points={ROUTE_PTS} width={2} from={PAL.gold} to={PAL.gold} boost={1.2} opacity={0.35} dashed={{ array: 0.012, ratio: 0.6 }} renderOrder={10} />
      <GlowLine ref={setp} points={sent} width={1.6} from={PAL.dust} to={PAL.dust} boost={1.3} opacity={0.7} dashed={{ array: 0.02, ratio: 0.55 }} renderOrder={11} />
      <GlowLine ref={glow} points={pts} width={13} from={ICE} to={MIST} boost={1} opacity={0.22} additive renderOrder={12} />
      <GlowLine ref={core} points={pts} width={4.5} from={ICE} to={MIST} boost={1.15} opacity={1} renderOrder={13} />

      {/* the footprint at the touch, and the ring spreading from it */}
      <group position={contact}>
        <mesh rotation-x={-Math.PI / 2}>
          <ringGeometry args={[0.2375, 0.262, 48]} />
          <meshBasicMaterial ref={stay} color={[1.2, 1.8, 2.2]} toneMapped={false} transparent opacity={0} side={THREE.DoubleSide} depthWrite={false} />
        </mesh>
        <mesh ref={ring} rotation-x={-Math.PI / 2} visible={false}>
          <ringGeometry args={[0.2375, 0.27, 64]} />
          <meshBasicMaterial ref={ringMat} color={[DANGER.r * 2, DANGER.g * 2, DANGER.b * 2]} toneMapped={false} transparent opacity={0} side={THREE.DoubleSide} depthWrite={false} />
        </mesh>
      </group>
      <HitHighlight hit={note.hit} T={T} />
      <TouchTag position={[contact[0], 0.9, contact[2]]} text={`TOUCHED · +${T.toFixed(1)} S`} />
    </group>
  );
});

/** a small vermilion tag that fades in at the touch */
function TouchTag({ position, text }: { position: [number, number, number]; text: string }) {
  const hit = useReplay((s) => s.hit);
  if (!hit) return null;
  return (
    <Html center position={position} zIndexRange={[24, 0]} style={{ pointerEvents: 'none' }}>
      <div className="obs-tag t-fade">{text}</div>
    </Html>
  );
}

export default function FlightReplay() {
  const run = useReplay((s) => s.run);
  return run >= 0 ? <Replay key={run} run={run} /> : null;
}
