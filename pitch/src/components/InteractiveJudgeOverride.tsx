'use client';

import { Html } from '@react-three/drei';
import { useFrame, type ThreeEvent } from '@react-three/fiber';
import { memo, useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { view } from '@/lib/choreography';
import { COURSE, FLIGHT_Z, GUIDES, PAD, START, VIA, WALL_H, enu } from '@/lib/course';
import { clamp, dampAngle, easeOutCubic, remap } from '@/lib/math';
import { PAL } from '@/lib/palette';
import { DEFAULT_CONFIG, occupiedAt, planMission, pointAt, type PlanResult } from '@/lib/planner';
import { replay } from '@/lib/replay';
import { flight, log, useDeck, type Obstacle } from '@/lib/store';
import { Blocker } from './Blocker';
import FlightReplay from './FlightReplay';
import { GlowLine, type GlowLineHandle } from './three/GlowLine';

/**
 * InteractiveJudgeOverride: OVERRIDE MODE.
 *
 *   top-down isometric maze  ->  click the grid  ->  a blocker drops (components/Blocker.tsx)
 *     ->  the planner re-runs (A* + EGO B-spline, see lib/planner.ts) and the ice-blue path redraws around it
 *
 * State machine lives in lib/store.ts (obstacles, plan, status). This file is the 3D side: picking, hover preview,
 * path lines, and the flight controller that flies the drone along the B-spline and re-plans mid-flight.
 * Colour roles: ice = the live plan, vermilion = obstacles and the invalidated plan, sand/gold = the world.
 * Nothing in here flashes: a re-plan redraws the path in place, with no screen flash.
 */

const DANGER = new THREE.Color(PAL.danger);
const DEMO_SPEED = 1.15; // m/s: the real course runs 0.3 - 0.5, this is a demo
const HALF = COURSE.net.halfX;


export default function InteractiveJudgeOverride() {
  const root = useRef<THREE.Group>(null!);
  useFrame(() => {
    if (root.current) root.current.visible = view.startW > 0.02;
  });
  return (
    <group ref={root} visible={false}>
      <FloorPicker />
      <LaunchMark />
      <PlanPath />
      <ViaMarkers />
      <Obstacles />
      <HoverGhost />
      <FlightController />
      <FlightReplay />
    </group>
  );
}

/* ───────────────────────────── picking ───────────────────────────── */

const snap = (v: number, g = 0.2) => Math.round(v / g) * g;

function FloorPicker() {
  const armed = useDeck((s) => s.armed);
  const override = useDeck((s) => s.override);
  const last = useRef({ x: 1e9, y: 1e9 });
  const timer = useRef<number | null>(null);

  useEffect(() => {
    if (!override) return;
    document.body.style.cursor = armed ? 'crosshair' : 'default';
    return () => {
      document.body.style.cursor = 'default';
    };
  }, [armed, override]);

  const evaluate = (x: number, y: number) => {
    const s = useDeck.getState();
    const occ = occupiedAt(COURSE, s.obstacles, x, y);
    if (occ === 'static') return s.setHover({ x, y, verdict: 'wall' });
    if (occ === 'dynamic') return s.setHover({ x, y, verdict: 'dup' });
    const probe: Obstacle = { id: 999, x, y, hw: 0.2, hd: 0.2, born: 0 };
    const flying = flight.mode === 'flying' || flight.mode === 'hold';
    const via = VIA.slice(flying ? flight.nextVia : 0);
    const res = planMission(COURSE, via.length ? via : VIA.slice(-1), [...s.obstacles, probe], flying ? [flight.x, flight.y] : START, DEFAULT_CONFIG, { kind: 'inject', obstacle: probe }, GUIDES);
    if (!res.ok) return s.setHover({ x, y, verdict: 'seal' });
    const delta = s.plan?.ok ? res.length - s.plan.length : 0;
    s.setHover({ x, y, verdict: 'ok', delta });
  };

  const onMove = (e: ThreeEvent<PointerEvent>) => {
    if (!armed || !override) return;
    const x = clamp(snap(e.point.x), -HALF + 0.1, HALF - 0.1);
    const y = clamp(snap(-e.point.z), -HALF + 0.1, HALF - 0.1);
    if (x === last.current.x && y === last.current.y) return;
    last.current = { x, y };
    // show the ghost immediately; judge it (a real replan, ~4 ms) on a short trailing timer
    useDeck.getState().setHover({ x, y, verdict: useDeck.getState().hover?.verdict ?? 'ok', delta: useDeck.getState().hover?.delta });
    if (timer.current) window.clearTimeout(timer.current);
    timer.current = window.setTimeout(() => evaluate(x, y), 45);
  };

  const onDown = (e: ThreeEvent<PointerEvent>) => {
    if (!armed || !override) return;
    e.stopPropagation();
    const x = clamp(e.point.x, -HALF + 0.1, HALF - 0.1);
    const y = clamp(-e.point.z, -HALF + 0.1, HALF - 0.1);
    useDeck.getState().drop(x, y);
    last.current = { x: 1e9, y: 1e9 };
  };

  return (
    <mesh rotation-x={-Math.PI / 2} position-y={0.004} onPointerMove={onMove} onPointerDown={onDown} onPointerOut={() => useDeck.getState().setHover(null)} visible>
      <planeGeometry args={[HALF * 2, HALF * 2]} />
      <meshBasicMaterial transparent opacity={0} depthWrite={false} />
    </mesh>
  );
}

/* ───────────────────────────── launch mark ───────────────────────────── */

function LaunchMark() {
  const ring = useRef<THREE.Mesh>(null!);
  const override = useDeck((s) => s.override);
  const p = enu(START[0], START[1], 0.012);
  useFrame((s) => {
    if (!ring.current) return;
    ring.current.scale.setScalar(1 + 0.03 * Math.sin(s.clock.elapsedTime * 1.2)); // a slow, shallow breath
    (ring.current.material as THREE.MeshBasicMaterial).opacity = (flight.mode === 'idle' ? 0.9 : 0.35) * view.startW;
  });
  return (
    <group position={p}>
      <mesh ref={ring} rotation-x={-Math.PI / 2}>
        <ringGeometry args={[0.2, 0.235, 48]} />
        <meshBasicMaterial color={[2.2, 1.6, 0.8]} toneMapped={false} transparent opacity={0} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
      {override ? (
        <Html position={[0, 0.02, 0.38]} center zIndexRange={[14, 0]} style={{ pointerEvents: 'none' }}>
          <div className="via-tag">LAUNCH MARK</div>
        </Html>
      ) : null}
    </group>
  );
}

/* ───────────────────────────── the plan ───────────────────────────── */

function toPoints(plan: PlanResult) {
  const n = plan.samples.length / 2;
  const out: THREE.Vector3[] = [];
  for (let i = 0; i < n; i += 2) out.push(new THREE.Vector3(...enu(plan.samples[2 * i], plan.samples[2 * i + 1], FLIGHT_Z)));
  if (n > 1 && (n - 1) % 2 !== 0) out.push(new THREE.Vector3(...enu(plan.samples[2 * (n - 1)], plan.samples[2 * (n - 1) + 1], FLIGHT_Z)));
  return out;
}

const PATH_FROM = '#7fb8d9'; // ice-blue: the live A* + EGO plan (cool = what the drone decides); a touch deeper than PAL.ice so bloom keeps it blue
const PATH_TO = '#c3e1f1';
const OLD_PATH = PAL.danger; // vermilion: the plan the blocker just invalidated

const PlanPath = memo(function PlanPath() {
  const plan = useDeck((s) => s.plan);
  const prev = useDeck((s) => s.prevPlan);
  const status = useDeck((s) => s.planStatus);
  const failed = useDeck((s) => s.failed);
  const epoch = useDeck((s) => s.planEpoch);

  const pts = useMemo(() => (plan?.ok ? toPoints(plan) : null), [plan]);
  const prevPts = useMemo(() => (prev?.ok ? toPoints(prev) : null), [prev]);
  const redPts = status === 'sealed' ? pts : prevPts;

  const core = useRef<GlowLineHandle>(null);
  const glow = useRef<GlowLineHandle>(null);
  const flow = useRef<GlowLineHandle>(null);
  const old = useRef<GlowLineHandle>(null);
  const head = useRef<THREE.Mesh>(null!);
  const seal = useRef<THREE.Group>(null!);

  useFrame((s, dt) => {
    if (replay.active) {
      // a real flight is on screen: the planner's lines step aside
      for (const h of [core.current, glow.current, flow.current, old.current]) if (h) h.mesh.visible = false;
      if (head.current) head.current.visible = false;
      if (seal.current) seal.current.visible = false;
      return;
    }
    const st = useDeck.getState();
    const now = performance.now();
    const age = (now - st.planStamp) / 1000;
    const gate = remap(view.startW, 0.6, 1);

    if (pts && core.current && glow.current) {
      // draw-in: the new plan is struck from the start mark to the pad in ~0.75 s
      const vis = easeOutCubic(Math.min(clamp(age / 0.75), gate));
      const on = status !== 'sealed' && vis > 0.002;
      for (const h of [core.current, glow.current, flow.current]) {
        if (!h) continue;
        h.mesh.visible = on;
        h.material.visibility = vis;
      }
      glow.current.material.opacity = 0.2; // steady: the halo does not pulse
      if (flow.current) flow.current.material.dashOffset -= dt * 0.12; // dashes drift toward the pad, slowly
      // bright pulse running down the line: direction of travel
      if (head.current) {
        const u = ((s.clock.elapsedTime * 0.16) % 1) * plan!.length;
        const p = pointAt(plan!, u);
        head.current.position.set(...enu(p.x, p.y, FLIGHT_Z));
        head.current.visible = on && gate > 0.9 && flight.mode !== 'flying';
      }
    }

    // old path: vermilion, dashed, fades out after a replan; stays up while the route is sealed
    if (old.current) {
      const h = old.current;
      h.material.visibility = 1;
      if (status === 'sealed') {
        h.mesh.visible = true;
        h.material.opacity = 0.9;
      } else {
        const fade = 1 - clamp(age / 1.4);
        h.mesh.visible = !!prevPts && fade > 0.01 && epoch > 1;
        h.material.opacity = 0.95 * fade;
      }
      h.material.dashOffset -= dt * 0.06;
    }
    if (seal.current) {
      seal.current.visible = status === 'sealed' && !!failed?.blockedAt;
      if (failed?.blockedAt) {
        const e = enu(failed.blockedAt[0], failed.blockedAt[1], 0.02);
        seal.current.position.set(e[0], e[1], e[2]);
        seal.current.rotation.y = s.clock.elapsedTime * 0.4;
      }
    }
  });

  return (
    <group>
      {pts ? (
        <>
          <GlowLine ref={glow} points={pts} width={12} from={PATH_FROM} to={PATH_TO} boost={1.0} opacity={0.2} additive renderOrder={12} />
          <GlowLine ref={core} points={pts} width={4} from={PATH_FROM} to={PATH_TO} boost={1.1} opacity={1} renderOrder={13} />
          <GlowLine ref={flow} points={pts} width={1.6} from={PAL.mist} to={PAL.mist} boost={1.6} opacity={0.9} dashed={{ array: 0.04, ratio: 0.7 }} renderOrder={14} />
        </>
      ) : null}
      {redPts ? <GlowLine ref={old} points={redPts} width={3} from={OLD_PATH} to={OLD_PATH} boost={1.6} opacity={0} dashed={{ array: 0.03, ratio: 0.5 }} renderOrder={11} /> : null}

      <mesh ref={head} visible={false}>
        <sphereGeometry args={[0.07, 16, 16]} />
        <meshBasicMaterial color={[2.4, 2.8, 3.0]} toneMapped={false} />
      </mesh>
      {/* the X where a sealed route dies */}
      <group ref={seal} visible={false}>
        <mesh rotation-x={-Math.PI / 2}>
          <ringGeometry args={[0.32, 0.38, 4]} />
          <meshBasicMaterial color={[DANGER.r * 1.8, DANGER.g * 1.8, DANGER.b * 1.8]} toneMapped={false} side={THREE.DoubleSide} transparent opacity={0.9} depthWrite={false} />
        </mesh>
        {status === 'sealed' ? (
          <Html center position={[0, 0.7, 0]} zIndexRange={[24, 0]} style={{ pointerEvents: 'none' }}>
            <div className="seal-tag">ROUTE SEALED</div>
          </Html>
        ) : null}
      </group>
    </group>
  );
});

/* ───────────────────────────── via markers ───────────────────────────── */

const ViaMarkers = memo(function ViaMarkers() {
  const plan = useDeck((s) => s.plan);
  const override = useDeck((s) => s.override);
  const g = useRef<THREE.Group>(null!);
  useFrame(() => {
    if (g.current) g.current.visible = view.startW > 0.9;
  });
  const used = plan?.via ?? VIA;
  return (
    <group ref={g}>
      {used.map((v) => {
        const p = enu(v.at[0], v.at[1], 0.015);
        const label = v.id === 'tubeN' ? 'TUBE N' : v.id === 'tubeS' ? 'TUBE S' : v.id === 'win' ? 'WINDOW' : v.id === 'gateA' ? 'GATE A' : v.id === 'pad' ? 'PAD' : null;
        return (
          <group key={v.id} position={p}>
            <mesh rotation-x={-Math.PI / 2} rotation-z={Math.PI / 4}>
              <ringGeometry args={[0.085, 0.11, 4]} />
              <meshBasicMaterial color={[1.7, 1.45, 1.1]} toneMapped={false} side={THREE.DoubleSide} transparent opacity={0.8} depthWrite={false} />
            </mesh>
            {label && override ? (
              <Html center position={[0, 0.02, 0.3]} zIndexRange={[14, 0]} style={{ pointerEvents: 'none' }}>
                <div className="via-tag">{label}</div>
              </Html>
            ) : null}
          </group>
        );
      })}
      {/* the pad, drawn as the finish line */}
      <mesh position={enu(PAD[0], PAD[1], 0.02)} rotation-x={-Math.PI / 2}>
        <ringGeometry args={[COURSE.pad.rOuter + 0.02, COURSE.pad.rOuter + 0.05, 64]} />
        <meshBasicMaterial color={[2.0, 1.4, 0.7]} toneMapped={false} side={THREE.DoubleSide} transparent opacity={0.5} depthWrite={false} />
      </mesh>
    </group>
  );
});

/* ───────────────────────────── blockers ───────────────────────────── */

function Obstacles() {
  const list = useDeck((s) => s.obstacles);
  return (
    <>
      {list.map((o) => (
        <Blocker key={o.id} o={o} />
      ))}
    </>
  );
}

/* ───────────────────────────── hover ghost ───────────────────────────── */

function HoverGhost() {
  const hover = useDeck((s) => s.hover);
  const armed = useDeck((s) => s.armed);
  const g = useRef<THREE.Group>(null!);
  const body = useRef<THREE.Mesh>(null!);
  const target = useRef(new THREE.Vector3());
  const mat = useMemo(() => new THREE.MeshBasicMaterial({ color: DANGER, transparent: true, opacity: 0.16, depthWrite: false, toneMapped: false }), []);
  const lineMat = useMemo(() => new THREE.LineBasicMaterial({ color: DANGER, toneMapped: false, transparent: true }), []);
  const geo = useMemo(() => new THREE.BoxGeometry(0.4, WALL_H, 0.4), []);
  const edges = useMemo(() => new THREE.EdgesGeometry(geo), [geo]);

  useEffect(() => {
    if (hover) target.current.set(hover.x, WALL_H / 2, -hover.y);
  }, [hover]);

  useFrame((_, dt) => {
    const gr = g.current;
    if (!gr) return;
    const show = !!hover && armed;
    gr.visible = show;
    if (!show) return;
    gr.position.lerp(target.current, 1 - Math.exp(-26 * dt));
    const v = hover!.verdict;
    const col = v === 'ok' ? DANGER : v === 'seal' ? new THREE.Color(PAL.amber) : new THREE.Color('#9a8a7e');
    mat.color.copy(col);
    lineMat.color.copy(col).multiplyScalar(2.4);
    mat.opacity = 0.14; // steady (the ghost used to throb at ~1.3 Hz)
    if (body.current) body.current.visible = v !== 'wall' && v !== 'dup';
  });

  const label = hover?.verdict === 'ok' ? `REPLAN ${hover.delta !== undefined && hover.delta !== 0 ? (hover.delta > 0 ? '+' : '') + hover.delta.toFixed(2) + ' m' : 'OK'}` : hover?.verdict === 'seal' ? 'SEALS THE ROUTE' : hover?.verdict === 'wall' ? 'STATIC MAP' : 'OCCUPIED';

  return (
    <group ref={g} visible={false}>
      <mesh ref={body} geometry={geo} material={mat} />
      <lineSegments geometry={edges} material={lineMat} />
      {hover && armed ? (
        <Html position={[0, WALL_H / 2 + 0.3, 0]} center zIndexRange={[26, 0]} style={{ pointerEvents: 'none' }}>
          <div className="hover-tag" data-v={hover.verdict}>
            {label}
          </div>
        </Html>
      ) : null}
    </group>
  );
}

/* ───────────────────────────── flight ───────────────────────────── */

function FlightController() {
  const plan = useDeck((s) => s.plan);
  const planRef = useRef<PlanResult | null>(null);
  useEffect(() => {
    planRef.current = plan;
  }, [plan]);

  useFrame((_, dtRaw) => {
    if (replay.active) return; // the replay drives the drone
    const dt = Math.min(dtRaw, 0.05);
    const p = planRef.current;
    if (flight.mode === 'idle' || flight.mode === 'landed') {
      flight.speed = 0;
      return;
    }
    if (flight.mode === 'hold' || !p || !p.ok) {
      flight.speed = Math.max(0, flight.speed - 4 * dt);
      return;
    }
    // trapezoid-ish speed: ease up from the pad, ease down to land
    const remaining = p.length - flight.s;
    const target = DEMO_SPEED * Math.min(1, 0.25 + flight.s / 1.0) * Math.min(1, 0.2 + remaining / 1.0);
    flight.speed += (target - flight.speed) * Math.min(1, 6 * dt);
    flight.s = Math.min(p.length, flight.s + flight.speed * dt);
    const pt = pointAt(p, flight.s);
    flight.x = pt.x;
    flight.y = pt.y;
    flight.heading = dampAngle(flight.heading, pt.heading, 7, dt);
    // climb out of the pad, cruise at route height, descend onto the pad
    const climb = Math.min(1, flight.s / 1.6);
    const land = Math.min(1, remaining / 1.2);
    flight.z = 0.45 + (FLIGHT_Z - 0.45) * climb * 1.0;
    if (land < 1) flight.z = 0.14 + (flight.z - 0.14) * land;

    // checkpoints
    const st = useDeck.getState();
    while (flight.nextVia < VIA.length) {
      const v = VIA[flight.nextVia];
      if (!p.via.some((u) => u.id === v.id)) {
        flight.nextVia++; // the planner skipped this one (Gate B fallback)
        continue;
      }
      if (Math.hypot(flight.x - v.at[0], flight.y - v.at[1]) < 0.4) {
        flight.nextVia++;
        const cp = v.id === 'gateA' ? 2 : v.id === 'tubeS' ? 3 : v.id === 'winOut' ? 4 : v.id === 'pad' ? 5 : flight.checkpoints;
        if (cp > flight.checkpoints) {
          flight.checkpoints = cp;
          if (cp < 5) log('TRACK', `checkpoint ${cp} reached: ${v.label}`, 'ok');
        }
      } else break;
    }
    if (flight.s > 1.6 && flight.checkpoints < 1) {
      flight.checkpoints = 1;
      log('TRACK', 'checkpoint 1 reached: first corridor', 'ok');
    }
    if (flight.s >= p.length - 1e-3) {
      flight.mode = 'landed';
      flight.speed = 0;
      flight.z = 0.14;
      flight.checkpoints = 5;
      st.setFlightMode('landed');
      log('OK', `touchdown on the pad. ${flight.checkpoints}/5 checkpoints, 35/35 points (simulated).`, 'ok');
    }
  });
  return null;
}
