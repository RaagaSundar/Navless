'use client';

import { Html, useGLTF } from '@react-three/drei';
import { useFrame, useLoader } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { ASSETS } from '@/config/assets';
import { view } from '@/lib/choreography';
import { CHECKPOINTS, COURSE, ROUTE, START, enu, routeFrac } from '@/lib/course';
import { easeOutCubic, lerp, remap } from '@/lib/math';
import { PAL } from '@/lib/palette';
import { buildNet, floorMaterial, flatMaterial, holo, lineMaterial, wallMaterial } from './three/holo';

/**
 * MazeEnvironment: the as-built DroneHacks course as a wireframe hologram.
 *
 *   source CAD -> restyle by kind (wall / tube / boulder / pad / net / floor) -> rise out of the floor in route order
 *
 * The geometry is the organisers' course with the 2026-11-07 maze change (MN / MD / MT in, M1-M3 out), read from git.
 * Swap ASSETS.maze for your own .glb / .gltf / .obj; names (or GLB extras.kind) decide how each node is styled.
 */

const IS_OBJ = ASSETS.maze.split('?')[0].toLowerCase().endsWith('.obj');
if (!IS_OBJ) useGLTF.preload(ASSETS.maze);

export default function MazeEnvironment() {
  // ⟦ MAZE CAD SLOT ⟧ the loader below picks .obj vs .glb/.gltf from the extension of ASSETS.maze
  return IS_OBJ ? <ObjMaze /> : <GltfMaze />;
}

function GltfMaze() {
  const { scene } = useGLTF(ASSETS.maze);
  return <MazeScene source={scene} zUp={false} />;
}
function ObjMaze() {
  const obj = useLoader(OBJLoader, ASSETS.maze);
  return <MazeScene source={obj} zUp />;
}

/* ───────────────────────────── classification ───────────────────────────── */

type Kind = 'wall' | 'tube' | 'cradle' | 'boulder' | 'pad' | 'net' | 'floor' | 'other' | 'skip';
interface Info {
  kind: Kind;
  sub?: string;
  wall?: string;
}

function classify(mesh: THREE.Mesh): Info {
  const ud = mesh.userData as { kind?: string; sub?: string; wall?: string };
  if (ud.kind) return { kind: ud.kind as Kind, sub: ud.sub, wall: ud.wall };
  // OBJ / unknown assets: walk up the names, then fall back to the shape of the thing
  let name = mesh.name;
  for (let o: THREE.Object3D | null = mesh.parent; o && !name; o = o.parent) name = o.name;
  const n = name.toLowerCase();
  if (/guideline/.test(n)) return { kind: 'skip' };
  if (/^floor/.test(n)) return { kind: 'floor' };
  if (/net/.test(n)) return { kind: 'net' };
  if (/tube oval|tunnel/.test(n)) return { kind: 'tube' };
  if (/cradle/.test(n)) return { kind: 'cradle' };
  if (/boulder/.test(n)) return { kind: 'boulder' };
  if (/pad|launch|ring|line/.test(n)) return { kind: 'pad', sub: /ring/.test(n) ? 'ring' : 'plate' };
  if (/wall|barrier|divider|chicane|approach|maze/.test(n)) return { kind: 'wall', sub: /maze/.test(n) ? 'maze' : 'plan' };
  mesh.geometry.computeBoundingBox();
  const b = mesh.geometry.boundingBox!;
  const h = b.max.y - b.min.y;
  if (h < 0.02) return { kind: 'pad', sub: 'plate' };
  return h > 0.8 ? { kind: 'wall', sub: 'plan' } : { kind: 'other' };
}

/* ───────────────────────────── styling ───────────────────────────── */

interface Rec {
  mats: THREE.ShaderMaterial[];
  delay: number;
  kind: Kind;
  alpha: number;
}

function nearestRouteFrac(x: number, z: number) {
  let best = 0;
  let bd = Infinity;
  const w = ROUTE.waypoints;
  for (let i = 0; i < w.length; i++) {
    const d = Math.hypot(w[i][0] - x, w[i][1] - -z);
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return routeFrac(best);
}

function styleCourse(source: THREE.Object3D, zUp: boolean) {
  const root = new THREE.Group();
  if (zUp) root.rotation.x = -Math.PI / 2; // ENU Z-up OBJ -> three Y-up
  const clone = source.clone(true);
  root.add(clone);
  root.updateMatrixWorld(true);

  const recs: Rec[] = [];
  const pads: THREE.ShaderMaterial[] = [];
  const meshes: THREE.Mesh[] = [];
  clone.traverse((o) => {
    if ((o as THREE.Mesh).isMesh) meshes.push(o as THREE.Mesh);
  });

  const walls: { mesh: THREE.Mesh; frac: number; rec: Rec }[] = [];
  for (const mesh of meshes) {
    const info = classify(mesh);
    mesh.geometry.computeBoundingBox();
    const bb = mesh.geometry.boundingBox!.clone().applyMatrix4(mesh.matrixWorld);
    const H = Math.max(0.05, bb.max.y);
    const add = (obj: THREE.Object3D) => mesh.add(obj);

    if (info.kind === 'floor' || info.kind === 'net' || info.kind === 'skip' || info.kind === 'other') {
      mesh.visible = false; // the floor grid and net are generated below
      continue;
    }

    if (info.kind === 'wall') {
      const fill = wallMaterial(H);
      // the walls the organisers added are picked out in gold, the rest of the course in sand
      const edge = lineMaterial(H, info.sub === 'maze' ? PAL.gold : PAL.sand);
      const lines = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, 25), edge);
      lines.renderOrder = 6;
      add(lines);
      mesh.material = fill;
      mesh.renderOrder = 5;
      const rec: Rec = { mats: [fill, edge], delay: 0, kind: 'wall', alpha: 1 };
      recs.push(rec);
      const c = bb.getCenter(new THREE.Vector3());
      walls.push({ mesh, frac: nearestRouteFrac(c.x, c.z), rec });
    } else if (info.kind === 'tube') {
      const fill = wallMaterial(H);
      fill.uniforms.uAlpha.value = 0.55;
      const wire = lineMaterial(H, 'accent', { wireframe: true, alpha: 0.28 });
      const ribs = new THREE.Mesh(mesh.geometry, wire);
      ribs.renderOrder = 6;
      add(ribs);
      mesh.material = fill;
      mesh.renderOrder = 5;
      recs.push({ mats: [fill, wire], delay: 0.62, kind: 'tube', alpha: 0.55 });
    } else if (info.kind === 'boulder') {
      const fill = wallMaterial(H);
      fill.uniforms.uAlpha.value = 0.7;
      const wire = lineMaterial(H, PAL.dust, { wireframe: true, alpha: 0.34 });
      const w = new THREE.Mesh(mesh.geometry, wire);
      w.renderOrder = 6;
      add(w);
      mesh.material = fill;
      mesh.renderOrder = 5;
      recs.push({ mats: [fill, wire], delay: 0.7, kind: 'boulder', alpha: 1 });
    } else if (info.kind === 'cradle') {
      const edge = lineMaterial(Math.max(H, 0.3), '#9a8470', { alpha: 0.5 });
      const lines = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, 25), edge);
      add(lines);
      mesh.visible = false;
      recs.push({ mats: [edge], delay: 0.6, kind: 'cradle', alpha: 1 });
    } else if (info.kind === 'pad') {
      const n = (mesh.name || mesh.parent?.name || '').toLowerCase();
      let mat: THREE.ShaderMaterial;
      if (/ring r30|ring r20|ring r9|ring/.test(n) || info.sub === 'ring') mat = flatMaterial('accent', { alpha: 0.6, boost: 1.0, pulse: 0.2 });
      else if (/cross/.test(n) || info.sub === 'cross') mat = flatMaterial(PAL.sand, { alpha: 0.6, boost: 1.0, pulse: 0.1 });
      else if (/launch/.test(n) || info.sub === 'launch') mat = flatMaterial(PAL.gold, { alpha: 0.9, boost: 1.4, pulse: 0.2 });
      else if (/line|approach/.test(n) || info.sub === 'line') mat = flatMaterial('accent', { alpha: 0.28, boost: 1.0, pulse: 0.0 });
      else mat = flatMaterial('accent', { alpha: 0.06, boost: 1.0, pulse: 0.0 }); // the 5 ft pad plate
      mesh.material = mat;
      mesh.renderOrder = 4;
      pads.push(mat);
    }
  }

  // wall rise order = the order the real route meets them (start -> finish)
  walls.sort((a, b) => a.frac - b.frac);
  walls.forEach((w, i) => (w.rec.delay = (i / Math.max(1, walls.length - 1)) * 0.5));

  return { root, recs, pads };
}

/* ───────────────────────────── scene ───────────────────────────── */

// the floor grid's theme colour: copper-gold in the tour, a little redder in override
const FLOOR_TOUR = new THREE.Color('#d99a55');
const FLOOR_OVERRIDE = new THREE.Color('#d9663a');
const NET = buildNet(COURSE.net.halfX, COURSE.net.height);

// where each checkpoint falls along the real route
const CP_F = CHECKPOINTS.map((c) => {
  let best = 0;
  let bd = Infinity;
  ROUTE.waypoints.forEach((w, i) => {
    const d = Math.hypot(w[0] - c.at[0], w[1] - c.at[1]);
    if (d < bd) {
      bd = d;
      best = i;
    }
  });
  return routeFrac(best);
});

function MazeScene({ source, zUp }: { source: THREE.Object3D; zUp: boolean }) {
  const styled = useMemo(() => styleCourse(source, zUp), [source, zUp]);
  const group = useRef<THREE.Group>(null!);
  const floor = useRef<THREE.Mesh<THREE.PlaneGeometry, THREE.ShaderMaterial>>(null!);
  const floorMat = useMemo(() => floorMaterial(), []);
  const cpEls = useRef<(HTMLDivElement | null)[]>([]);

  useFrame((state) => {
    const g = group.current;
    if (!g) return;
    const m = view.mazeIn;
    const wasVisible = g.visible;
    g.visible = m > 0.001;
    if (!g.visible) {
      if (wasVisible) for (const el of cpEls.current) if (el) el.style.opacity = '0';
      return;
    }

    holo.uTime.value = state.clock.elapsedTime;
    holo.uOverride.value = view.o;
    holo.uAccent.value.lerpColors(FLOOR_TOUR, FLOOR_OVERRIDE, view.o);

    for (const r of styled.recs) {
      // 1.08, not 1: the glowing "cut front" sits at the top edge when rise == height, so a finished wall must overshoot it
      // or every wall top keeps glowing like a slab after it has risen
      const rise = easeOutCubic(remap(m, r.delay, r.delay + 0.42)) * 1.08;
      for (const mat of r.mats) mat.uniforms.uRise.value = rise;
      // dim the glass a touch in override so the path and blockers own the frame
      if (r.kind === 'wall') r.mats[0].uniforms.uAlpha.value = lerp(1, 0.7, view.o);
      // tunnel cutaway while the IR-centering beat plays: you should see the drone and the beams inside
      if (r.kind === 'tube') {
        const cut = view.calloutMix[2];
        r.mats[0].uniforms.uAlpha.value = lerp(0.55, 0.05, cut);
        r.mats[1].uniforms.uAlpha.value = lerp(0.28, 0.42, cut);
      }
    }
    const reveal = remap(m, 0.0, 0.85);
    for (const p of styled.pads) p.uniforms.uReveal.value = reveal;
    floorMat.uniforms.uReveal.value = reveal;
    NET.material.uniforms.uAlpha.value = remap(m, 0.3, 1) * lerp(0.55, 0.3, view.o);

    // checkpoint tags (DOM, written directly): only while the planning beat is on screen, so the maze is not label soup
    const gs = view.ghostShow;
    for (let i = 0; i < cpEls.current.length; i++) {
      const el = cpEls.current[i];
      if (!el) continue;
      el.style.opacity = String(gs * view.calloutMix[0]);
      el.dataset.hit = view.ghostU >= CP_F[i] - 0.002 ? '1' : '0';
    }
    void floor;
  });

  return (
    <group ref={group} visible={false}>
      {/* ⟦ MAZE CAD SLOT ⟧ the restyled course is injected here */}
      <primitive object={styled.root} />

      <mesh ref={floor} rotation-x={-Math.PI / 2} position-y={-0.004} material={floorMat} renderOrder={1}>
        <planeGeometry args={[44, 44]} />
      </mesh>
      <primitive object={NET.object} renderOrder={2} />


      {/* checkpoints: the five scored stops, 35 points */}
      {CHECKPOINTS.map((c, i) => {
        const p = enu(c.at[0], c.at[1], 0);
        return (
          <group key={c.n} position={p}>
            <mesh rotation-x={-Math.PI / 2} rotation-z={Math.PI / 4} position-y={0.01}>
              <ringGeometry args={[0.2, 0.235, 4]} />
              <meshBasicMaterial color={[2.2, 1.5, 0.6]} toneMapped={false} transparent opacity={0.85} side={THREE.DoubleSide} depthWrite={false} />
            </mesh>
            <lineSegments>
              <bufferGeometry>
                <bufferAttribute attach="attributes-position" args={[new Float32Array([0, 0, 0, 0, 1.15, 0]), 3]} />
              </bufferGeometry>
              <lineBasicMaterial color="#f3c98f" transparent opacity={0.4} toneMapped={false} />
            </lineSegments>
            <Html position={[0, 1.2, 0]} center zIndexRange={[16, 0]} style={{ pointerEvents: 'none' }}>
              <div
                ref={(el) => {
                  cpEls.current[i] = el;
                }}
                className="cp-tag"
                data-hit="0"
                style={{ opacity: 0 }}
              >
                <span className="cp-n">{c.n}</span>
                <span className="cp-name">{c.name}</span>
              </div>
            </Html>
          </group>
        );
      })}
    </group>
  );
}

void START;
