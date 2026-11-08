#!/usr/bin/env node
/**
 * step-to-glb.mjs: Sky Beta STEP  ->  web-ready GLB for the pitch site.
 *
 *   node scripts/step-to-glb.mjs --dump scratch/skybeta --out public/models/navless-uav.glb
 *   node scripts/step-to-glb.mjs --step ~/Downloads/sky-beta.step --dump scratch/skybeta --python <python with cadquery-ocp>
 *
 * Pipeline
 *   1. (python) scripts/step_extract.py tessellates the STEP with native OpenCascade -> scratch/<dump>/{parts.json,mesh.bin}
 *   2. (node)   this script welds, decimates (per-part triangle budgets), tags every solid with a material ROLE,
 *               rotates CAD axes -> three.js axes, recentres, and writes one GLB.
 *
 * CAD frame (STEP):   X lateral, Y fore/aft (the OAK-D2 camera is at -Y = the NOSE), Z up.   metres
 * GLB frame (output): Y up, nose along -Z (three.js "forward"), origin at the airframe centre.   metres
 *
 * The GLB is the authoring contract for the site (so you can swap in your own CAD):
 *   - every node carries extras.role  ('plate' | 'arm' | 'chrome' | 'motor-bell' | 'jetson' | 'port' | 'pcb' | 'pixhawk' |
 *                                      'oak-body' | 'lens' | 'glass' | 'led' | 'dark' | 'battery' | 'battery-label' | 'heatsink')
 *   - empties  prop_FL / prop_FR / prop_RL / prop_RR   mark the prop hubs (extras.dir = +1 CW / -1 CCW, extras.radius)
 *   - empties  anchor_camera / anchor_jetson / anchor_pixhawk / anchor_battery / anchor_esc / anchor_motor_*  place HUD callouts
 *   Name your own empties the same way and the site animates them with no code changes.
 */
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Document, NodeIO } from '@gltf-transform/core';
import { MeshoptSimplifier } from 'meshoptimizer';

const here = path.dirname(fileURLToPath(import.meta.url));
const pitch = path.resolve(here, '..');

/* ───────────────────────────── args ───────────────────────────── */
const args = Object.fromEntries(
  process.argv.slice(2).reduce((acc, a, i, all) => {
    if (a.startsWith('--')) acc.push([a.slice(2), all[i + 1] && !all[i + 1].startsWith('--') ? all[i + 1] : 'true']);
    return acc;
  }, []),
);
const dump = path.resolve(args.dump ?? path.join(pitch, 'scratch', 'skybeta'));
const outFile = path.resolve(args.out ?? path.join(pitch, 'public', 'models', 'navless-uav.glb'));

if (!fs.existsSync(path.join(dump, 'parts.json'))) {
  if (!args.step) {
    console.error(`no tessellation dump at ${dump}. Pass --step <file.step> (needs python with cadquery-ocp) or --dump <dir>.`);
    process.exit(2);
  }
  const py = args.python ?? process.env.PYTHON ?? 'python';
  console.log(`tessellating ${args.step} with ${py} (about 2-3 minutes for the 78 MB Sky Beta file)...`);
  const r = spawnSync(py, ['-I', path.join(here, 'step_extract.py'), '--step', path.resolve(args.step), '--out', dump], { stdio: 'inherit' });
  if (r.status !== 0) process.exit(r.status ?? 1);
}

/* ───────────────────────────── load dump ───────────────────────────── */
const meta = JSON.parse(fs.readFileSync(path.join(dump, 'parts.json'), 'utf8'));
const raw = fs.readFileSync(path.join(dump, 'mesh.bin'));
const ab = raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.byteLength);
const f32 = (info) => new Float32Array(ab, info.offset, info.length);
const u32 = (info) => new Uint32Array(ab, info.offset, info.length);

await MeshoptSimplifier.ready;

/* ───────────────────────────── frame + roles + budgets ───────────────────────────── */

// Pivot (CAD metres): motor-square centre in plan, mid-height of the 0.116 m airframe.
const PIVOT = { x: 0, y: 0.032, z: 0.058 };
/** CAD (x, y, z) -> three (-x, z, y): a proper rotation (det +1), nose (-Y) -> -Z, up (+Z) -> +Y. */
const toThree = (x, y, z) => [-(x - PIVOT.x), z - PIVOT.z, y - PIVOT.y];

const mm = (v) => v * 1000;
function sizeMM(p) {
  const b = p.bbox;
  return [mm(b[3] - b[0]), mm(b[4] - b[1]), mm(b[5] - b[2])];
}
function centerMM(p) {
  const b = p.bbox;
  return [mm((b[0] + b[3]) / 2), mm((b[1] + b[4]) / 2), mm((b[2] + b[5]) / 2)];
}

function roleOf(p) {
  const n = p.part.toLowerCase();
  const [sx, sy, sz] = sizeMM(p);
  const [cx, cy, cz] = centerMM(p);
  if (n.includes('standoff')) return 'chrome';
  if (n.includes('arm')) return 'arm';
  if (n.includes('plate')) return 'plate';
  if (n.includes('motor')) return p.solid === 0 ? 'motor-bell' : 'chrome';
  if (n.includes('ovonic')) return p.solid === 0 ? 'battery' : 'battery-label';
  if (n.includes('aero selfie')) return p.solid === 0 ? 'pcb' : 'dark';
  if (n.includes('pixhawk')) return 'pixhawk';
  if (n.includes('oak-d2')) {
    if (sy < 1) return 'glass'; // thin front glass plates
    if (sy >= 2 && sy <= 5 && sx === sz && sx <= 13) return 'lens'; // 12 x 4 x 12 lens barrels
    if (sx < 6 && sz < 6) return 'chrome'; // mounting screws
    if (sy < 1.5) return 'led';
    return 'oak-body';
  }
  if (n.includes('jetson')) {
    if (sz <= 2.0 && sx >= 15) return 'pcb';
    if (cz > 24 && cy < 0) return 'heatsink';
    if (sz >= 12 && cy > 25) return 'port';
    return 'jetson';
  }
  return 'dark';
}

/** Triangle budget per solid after decimation. null = keep as is. */
function budgetOf(p, role) {
  const n = p.part.toLowerCase();
  if (n.includes('jetson orin')) return Math.max(70, Math.round(p.tris * 0.055));
  if (n.includes('oak-d2')) {
    if (role === 'oak-body') return p.tris > 50000 ? 8500 : 5500;
    if (role === 'chrome') return 360; // screws
    return null;
  }
  if (n.includes('pixhawk')) return p.tris > 15000 ? 4200 : p.tris > 6000 ? 2200 : null;
  return null;
}

/* ───────────────────────────── mesh processing ───────────────────────────── */

/** Weld by (position, normal) so smooth surfaces share vertices and hard edges stay split. */
function weld(pos, nrm, idx) {
  const map = new Map();
  const outP = [];
  const outN = [];
  const remap = new Uint32Array(pos.length / 3);
  const qp = 1e6; // 1 micron
  const qn = 60;
  for (let i = 0; i < remap.length; i++) {
    const key = `${Math.round(pos[3 * i] * qp)},${Math.round(pos[3 * i + 1] * qp)},${Math.round(pos[3 * i + 2] * qp)},${Math.round(nrm[3 * i] * qn)},${Math.round(nrm[3 * i + 1] * qn)},${Math.round(nrm[3 * i + 2] * qn)}`;
    let v = map.get(key);
    if (v === undefined) {
      v = outP.length / 3;
      map.set(key, v);
      outP.push(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]);
      outN.push(nrm[3 * i], nrm[3 * i + 1], nrm[3 * i + 2]);
    }
    remap[i] = v;
  }
  const outI = [];
  for (let t = 0; t < idx.length; t += 3) {
    const a = remap[idx[t]];
    const b = remap[idx[t + 1]];
    const c = remap[idx[t + 2]];
    if (a !== b && b !== c && a !== c) outI.push(a, b, c);
  }
  return { pos: new Float32Array(outP), nrm: new Float32Array(outN), idx: new Uint32Array(outI) };
}

/** Hard-surface look for decimated parts: unweld and give every triangle its own face normal. */
function flatten(m) {
  const n = m.idx.length;
  const pos = new Float32Array(n * 3);
  const nrm = new Float32Array(n * 3);
  const idx = new Uint32Array(n);
  for (let t = 0; t < n; t += 3) {
    const a = m.idx[t] * 3;
    const b = m.idx[t + 1] * 3;
    const c = m.idx[t + 2] * 3;
    const ux = m.pos[b] - m.pos[a];
    const uy = m.pos[b + 1] - m.pos[a + 1];
    const uz = m.pos[b + 2] - m.pos[a + 2];
    const vx = m.pos[c] - m.pos[a];
    const vy = m.pos[c + 1] - m.pos[a + 1];
    const vz = m.pos[c + 2] - m.pos[a + 2];
    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1;
    nx /= l;
    ny /= l;
    nz /= l;
    [a, b, c].forEach((src, k) => {
      const o = (t + k) * 3;
      pos[o] = m.pos[src];
      pos[o + 1] = m.pos[src + 1];
      pos[o + 2] = m.pos[src + 2];
      nrm[o] = nx;
      nrm[o + 1] = ny;
      nrm[o + 2] = nz;
      idx[t + k] = t + k;
    });
  }
  return { pos, nrm, idx };
}

function decimate(m, targetTris) {
  if (!targetTris || m.idx.length / 3 <= targetTris) return m;
  const target = targetTris * 3;
  // position-only simplification: connectors / screws / pins are full of hard edges that make the
  // attribute-aware variant refuse to collapse. Normals ride along on the surviving vertices.
  let best = m.idx;
  let err = 0.0002; // metres (absolute)
  for (let k = 0; k < 8; k++) {
    const [res] = MeshoptSimplifier.simplify(m.idx, m.pos, 3, target, err, ['ErrorAbsolute']);
    best = res;
    if (res.length <= target * 1.3) break;
    err *= 2.4;
  }
  if (best.length > target * 1.3) {
    // clutter that no error bound will fold: let the triangle budget alone decide
    const [res] = MeshoptSimplifier.simplifySloppy(m.idx, m.pos, 3, null, target, 1.0);
    best = res;
  }
  // compact vertex buffers (compactMesh rewrites the indices in place and returns old->new)
  const idx = Uint32Array.from(best);
  const [remap, unique] = MeshoptSimplifier.compactMesh(idx);
  const pos = new Float32Array(unique * 3);
  const nrm = new Float32Array(unique * 3);
  for (let i = 0; i < remap.length; i++) {
    const r = remap[i];
    if (r === 0xffffffff || r >= unique) continue;
    pos.set(m.pos.subarray(3 * i, 3 * i + 3), 3 * r);
    nrm.set(m.nrm.subarray(3 * i, 3 * i + 3), 3 * r);
  }
  for (let i = 0; i < idx.length; i++) if (idx[i] >= unique) throw new Error('compactMesh produced an out-of-range index');
  return flatten({ pos, nrm, idx });
}

/* ───────────────────────────── build ───────────────────────────── */

const groups = new Map(); // key part|role -> { part, role, chunks[] }
let trisIn = 0;
const t0 = performance.now();
for (const p of meta.parts) {
  trisIn += p.tris;
  const role = roleOf(p);
  const P = f32(p.pos);
  const N = f32(p.nrm);
  const I = u32(p.idx);
  // CAD -> three
  const pos = new Float32Array(P.length);
  const nrm = new Float32Array(N.length);
  for (let i = 0; i < P.length; i += 3) {
    const [x, y, z] = toThree(P[i], P[i + 1], P[i + 2]);
    pos[i] = x;
    pos[i + 1] = y;
    pos[i + 2] = z;
    nrm[i] = -N[i];
    nrm[i + 1] = N[i + 2];
    nrm[i + 2] = N[i + 1];
  }
  let m = weld(pos, nrm, I);
  m = decimate(m, budgetOf(p, role));
  const key = `${p.part}|${role}`;
  if (!groups.has(key)) groups.set(key, { part: p.part, role, chunks: [] });
  groups.get(key).chunks.push(m);
}

const doc = new Document();
doc.getRoot().getAsset().generator = 'navless pitch: step-to-glb.mjs';
const buffer = doc.createBuffer();
const scene = doc.createScene('navless-uav');
const root = doc.createNode('navless-uav').setExtras({
  source: 'sky-beta.step (Autodesk Fusion export, 2026-11-02)',
  units: 'metres',
  note: 'Y up, nose along -Z. Props are not in the CAD: spawn them at prop_* empties.',
});
scene.addChild(root);

/** Preview PBR so the GLB looks right in any viewer; the site restyles by role. */
const MAT = {
  plate: { c: [0.92, 0.93, 0.95], m: 1, r: 0.08 },
  arm: { c: [0.1, 0.1, 0.11], m: 1, r: 0.16 },
  chrome: { c: [0.95, 0.96, 0.98], m: 1, r: 0.05 },
  'motor-bell': { c: [0.08, 0.08, 0.09], m: 1, r: 0.22 },
  jetson: { c: [0.04, 0.045, 0.05], m: 0.4, r: 0.5 },
  heatsink: { c: [0.8, 0.82, 0.85], m: 1, r: 0.18 },
  port: { c: [0.55, 0.57, 0.6], m: 1, r: 0.3 },
  pcb: { c: [0.02, 0.03, 0.035], m: 0.2, r: 0.45 },
  pixhawk: { c: [0.03, 0.03, 0.035], m: 0.5, r: 0.4 },
  'oak-body': { c: [0.03, 0.03, 0.035], m: 0.6, r: 0.35 },
  lens: { c: [0.0, 0.0, 0.0], m: 0, r: 0.05, e: [0.1, 0.9, 1.0] },
  glass: { c: [0.0, 0.0, 0.0], m: 0.2, r: 0.02 },
  led: { c: [0.0, 0.0, 0.0], m: 0, r: 0.4, e: [1.0, 0.1, 0.2] },
  dark: { c: [0.02, 0.02, 0.025], m: 0.3, r: 0.6 },
  battery: { c: [0.015, 0.015, 0.018], m: 0.2, r: 0.55 },
  'battery-label': { c: [0.0, 0.0, 0.0], m: 0, r: 0.5, e: [1.0, 0.08, 0.2] },
};
const mats = {};
for (const [role, v] of Object.entries(MAT)) {
  const mat = doc
    .createMaterial(role)
    .setBaseColorFactor([...v.c, 1])
    .setMetallicFactor(v.m)
    .setRoughnessFactor(v.r);
  if (v.e) mat.setEmissiveFactor(v.e);
  mats[role] = mat;
}

const slug = (s) => s.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '');
let trisOut = 0;
const rows = [];
for (const { part, role, chunks } of groups.values()) {
  // concatenate the group's chunks into one primitive
  let nv = 0;
  let ni = 0;
  for (const c of chunks) {
    nv += c.pos.length / 3;
    ni += c.idx.length;
  }
  const P = new Float32Array(nv * 3);
  const N = new Float32Array(nv * 3);
  const I = new Uint32Array(ni);
  let vo = 0;
  let io = 0;
  for (const c of chunks) {
    P.set(c.pos, vo * 3);
    N.set(c.nrm, vo * 3);
    for (let k = 0; k < c.idx.length; k++) I[io + k] = c.idx[k] + vo;
    vo += c.pos.length / 3;
    io += c.idx.length;
  }
  const prim = doc
    .createPrimitive()
    .setAttribute('POSITION', doc.createAccessor().setType('VEC3').setArray(P).setBuffer(buffer))
    .setAttribute('NORMAL', doc.createAccessor().setType('VEC3').setArray(N).setBuffer(buffer))
    .setIndices(doc.createAccessor().setType('SCALAR').setArray(nv > 65535 ? I : Uint16Array.from(I)).setBuffer(buffer))
    .setMaterial(mats[role] ?? mats.dark);
  const name = `${slug(part)}__${role}`;
  const mesh = doc.createMesh(name).addPrimitive(prim);
  const node = doc.createNode(name).setMesh(mesh).setExtras({ role, part });
  root.addChild(node);
  trisOut += I.length / 3;
  rows.push([part, role, chunks.length, ni / 3]);
}

/* ───────────────────────────── empties: props + HUD anchors ───────────────────────────── */

const motors = meta.parts.filter((p) => p.part.toLowerCase().includes('motor') && p.solid === 0);
const PROP_RADIUS = 0.0775; // 6.1 in props: the repo's drone model (overall width 0.475 m = 2 x (0.16 + 0.0775))
const hubZ = 0.0735; // motor shaft top (CAD z)
const propDefs = [];
for (const m of motors) {
  const [cx, cy] = centerMM(m).map((v) => v / 1000);
  const [x, y, z] = toThree(cx, cy, hubZ);
  const left = /left/i.test(m.part);
  const front = /front/i.test(m.part);
  // standard quad-X: diagonal pairs share direction. CAD "left" is -X, which our rotation mirrors in x, so key off position instead.
  const dir = (x > 0) === (z < 0) ? 1 : -1;
  const tag = `${front ? 'F' : 'R'}${left ? 'L' : 'R'}`;
  propDefs.push({ tag, x, y, z, dir });
}
for (const d of propDefs) {
  root.addChild(doc.createNode(`prop_${d.tag}`).setTranslation([d.x, d.y, d.z]).setExtras({ dir: d.dir, radius: PROP_RADIUS }));
}

const anchor = (name, p, extras = {}) => {
  const [x, y, z] = toThree(p[0] / 1000, p[1] / 1000, p[2] / 1000);
  root.addChild(doc.createNode(name).setTranslation([x, y, z]).setExtras({ anchor: true, ...extras }));
};
const byPart = (s) => meta.parts.filter((p) => p.part.toLowerCase().includes(s));
const bboxCenter = (ps) => {
  const mn = [Infinity, Infinity, Infinity];
  const mx = [-Infinity, -Infinity, -Infinity];
  for (const p of ps)
    for (let a = 0; a < 3; a++) {
      mn[a] = Math.min(mn[a], p.bbox[a]);
      mx[a] = Math.max(mx[a], p.bbox[3 + a]);
    }
  return [0, 1, 2].map((a) => ((mn[a] + mx[a]) / 2) * 1000);
};
const oak = bboxCenter(byPart('oak-d2'));
anchor('anchor_camera', [oak[0], bboxCenter(byPart('oak-d2').filter((p) => roleOf(p) === 'glass'))[1] - 6, oak[2]], { label: 'OAK-D2 stereo' });
anchor('anchor_jetson', bboxCenter(byPart('jetson')), { label: 'Jetson Orin Nano' });
anchor('anchor_pixhawk', bboxCenter(byPart('pixhawk 6c')), { label: 'Pixhawk 6C' });
anchor('anchor_battery', bboxCenter(byPart('ovonic')), { label: '4S 5000 mAh LiPo' });
anchor('anchor_esc', bboxCenter(byPart('aero selfie')), { label: '45A 4-in-1 ESC' });
for (const d of propDefs) {
  const [x, y, z] = [d.x, d.y, d.z];
  root.addChild(doc.createNode(`anchor_motor_${d.tag}`).setTranslation([x, y - 0.02, z]).setExtras({ anchor: true, label: 'X2807 1500KV' }));
}

/* ───────────────────────────── write ───────────────────────────── */
fs.mkdirSync(path.dirname(outFile), { recursive: true });
const io = new NodeIO();
const glb = await io.writeBinary(doc);
fs.writeFileSync(outFile, glb);

rows.sort((a, b) => b[3] - a[3]);
console.log('\npart / role                                          solids      tris');
for (const [part, role, n, tris] of rows) console.log(`${(part + ' / ' + role).padEnd(52)} ${String(n).padStart(5)} ${String(tris).padStart(9)}`);
console.log(`\ntriangles ${trisIn.toLocaleString()} -> ${trisOut.toLocaleString()}   props ${propDefs.map((d) => d.tag + (d.dir > 0 ? '(cw)' : '(ccw)')).join(' ')}`);
console.log(`wrote ${outFile}  ${(glb.length / 1048576).toFixed(2)} MB  in ${((performance.now() - t0) / 1000).toFixed(1)} s`);
