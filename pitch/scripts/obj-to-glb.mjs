#!/usr/bin/env node
/**
 * obj-to-glb.mjs: the AS-BUILT DroneHacks course (dronehacks-course-asbuilt.obj on git) -> web GLB + scan cloud.
 *
 *   node scripts/obj-to-glb.mjs                         (reads drone19/flight via `git show`, read-only)
 *   node scripts/obj-to-glb.mjs --obj path/to/your.obj  (inject your own map; see README "Asset slots")
 *   COURSE_REF=main node scripts/obj-to-glb.mjs
 *
 * Writes
 *   public/models/dronehacks-course.glb   one node per OBJ group, extras { kind, wall }
 *   public/models/course-scan.bin         LiDAR-STYLE resample of the map (see header below)
 *
 * OBJ frame: ENU metres, Z up, origin = floor centre.   GLB frame: Y up (three.js): (x, y, z) -> (x, z, -y).
 *
 * Dropped on purpose
 *   M1 / M2 / M3 maze dividers  empty groups: the organisers removed them on 2026-11-07
 *   Guideline (zone 1)          the OLD maze's floor tape, flagged DO NOT FOLLOW in course_asbuilt.json
 * Decimated
 *   Boulder 1..5                14.6k triangles each in the source; 500 here
 *
 * course-scan.bin layout (little endian):  u32 magic 0x4e4c5343 ('NLSC') | u32 count | f32[count*3] xyz (three axes) | u8[count] kind
 * kinds: 0 floor, 1 net, 2 wall, 3 tube, 4 boulder, 5 pad/markings, 6 other.
 * NOTE: the raw Scaniverse scan (596k points) is not in git, only the walls fitted to it. This cloud is a resample of the as-built
 * mesh with phone-LiDAR-like noise (sigma 6 mm) so the site can tell the scan -> map -> plan story. It is labelled that way in the UI.
 */
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { Document, NodeIO } from '@gltf-transform/core';
import { MeshoptSimplifier } from 'meshoptimizer';

const here = path.dirname(fileURLToPath(import.meta.url));
const pitch = path.resolve(here, '..');
const repo = path.resolve(pitch, '..');
const REF = process.env.COURSE_REF || 'drone19/flight';

const args = Object.fromEntries(
  process.argv.slice(2).reduce((acc, a, i, all) => {
    if (a.startsWith('--')) acc.push([a.slice(2), all[i + 1] && !all[i + 1].startsWith('--') ? all[i + 1] : 'true']);
    return acc;
  }, []),
);

const text = args.obj
  ? fs.readFileSync(path.resolve(args.obj), 'utf8')
  : execFileSync('git', ['-C', repo, 'show', `${REF}:dronehacks-course-asbuilt.obj`], { maxBuffer: 256 * 1024 * 1024 }).toString('utf8');

await MeshoptSimplifier.ready;

/* ───────────────────────────── parse ───────────────────────────── */
const V = [];
const VN = [];
const groups = new Map(); // name -> { corners: [{v,n,mat}] triples }
let cur = null;
let mat = '';
for (const raw of text.split('\n')) {
  const l = raw.trimEnd();
  if (l.startsWith('v ')) {
    const [, x, y, z] = l.split(/\s+/);
    V.push([+x, +y, +z]);
  } else if (l.startsWith('vn ')) {
    const [, x, y, z] = l.split(/\s+/);
    VN.push([+x, +y, +z]);
  } else if (l.startsWith('g ')) {
    const name = l.slice(2).trim();
    if (!groups.has(name)) groups.set(name, { tris: [] });
    cur = groups.get(name);
  } else if (l.startsWith('usemtl ')) {
    mat = l.split(/\s+/)[1];
  } else if (l.startsWith('f ')) {
    if (!cur) continue;
    const c = l
      .split(/\s+/)
      .slice(1)
      .map((tok) => {
        const [v, , n] = tok.split('/');
        const vi = +v;
        const ni = n ? +n : 0;
        return { v: vi > 0 ? vi - 1 : V.length + vi, n: ni ? (ni > 0 ? ni - 1 : VN.length + ni) : -1, mat };
      });
    for (let k = 1; k < c.length - 1; k++) cur.tris.push([c[0], c[k], c[k + 1]]);
  }
}

/* ───────────────────────────── classify ───────────────────────────── */
const SKIP = [/guideline/i];
function classify(name) {
  const n = name.toLowerCase();
  if (SKIP.some((r) => r.test(n))) return null;
  if (/^floor/.test(n)) return { kind: 'floor' };
  if (/net (side|roof)/.test(n)) return { kind: 'net', roof: /roof/.test(n) };
  if (/barrier/.test(n)) return { kind: 'wall', sub: 'barrier', wall: 'Barrier' + (n.match(/barrier (\d)/)?.[1] ?? '') };
  if (/maze wall north/.test(n)) return { kind: 'wall', sub: 'maze', wall: 'MN' };
  if (/long diagonal/.test(n)) return { kind: 'wall', sub: 'maze', wall: 'MD' };
  if (/maze wall to tube/.test(n)) return { kind: 'wall', sub: 'maze', wall: 'MT' };
  if (/maze divider/.test(n)) return { kind: 'wall', sub: 'maze', wall: name.slice(0, 2) };
  if (/c2a/.test(n)) return { kind: 'wall', sub: 'window', wall: 'C2a' };
  if (/c2b/.test(n)) return { kind: 'wall', sub: 'chicane', wall: 'C2b' };
  if (/c1 chicane/.test(n)) return { kind: 'wall', sub: 'chicane', wall: 'C1' };
  if (/a1 approach/.test(n)) return { kind: 'wall', sub: 'approach', wall: 'A1' };
  if (/tube oval/.test(n)) return { kind: 'tube' };
  if (/tube cradle/.test(n)) return { kind: 'cradle' };
  if (/boulder/.test(n)) return { kind: 'boulder' };
  if (/landing pad ring/.test(n)) return { kind: 'pad', sub: 'ring' };
  if (/landing pad cross/.test(n)) return { kind: 'pad', sub: 'cross' };
  if (/landing pad 5 ft/.test(n)) return { kind: 'pad', sub: 'plate' };
  if (/launch mark/.test(n)) return { kind: 'pad', sub: 'launch' };
  if (/approach ring/.test(n)) return { kind: 'pad', sub: 'line' };
  return { kind: 'other' };
}

/** ENU -> three */
const toThree = ([x, y, z]) => [x, z, -y];
const LIFT = 0.003; // lift floor markings 3 mm so they never z-fight the floor

/* ───────────────────────────── build meshes per group ───────────────────────────── */
function buildMesh(group, kind) {
  const map = new Map();
  const P = [];
  const N = [];
  const I = [];
  for (const tri of group.tris) {
    // geometric normal for faces without vn
    const [a, b, c] = tri.map((t) => V[t.v]);
    const e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let gn = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
    const gl = Math.hypot(...gn) || 1;
    gn = gn.map((v) => v / gl);
    for (const t of tri) {
      const key = `${t.v}/${t.n}`;
      let idx = map.get(key);
      if (idx === undefined) {
        idx = P.length / 3;
        map.set(key, idx);
        const p = [...V[t.v]];
        if (kind === 'pad' || kind === 'floor') {
          if (Math.abs(p[2]) < 0.002 && kind === 'pad') p[2] += LIFT;
        }
        const n = t.n >= 0 ? VN[t.n] : gn;
        P.push(...toThree(p));
        N.push(...toThree(n));
      }
      I.push(idx);
    }
  }
  return { pos: new Float32Array(P), nrm: new Float32Array(N), idx: new Uint32Array(I) };
}

function decimate(m, targetTris) {
  if (m.idx.length / 3 <= targetTris) return m;
  const target = targetTris * 3;
  // weld first: the OBJ shares vertices only per (v, vn) pair
  const [res] = MeshoptSimplifier.simplify(m.idx, m.pos, 3, target, 0.01, ['ErrorAbsolute']);
  let best = res;
  if (best.length > target * 1.3) best = MeshoptSimplifier.simplifySloppy(m.idx, m.pos, 3, null, target, 1.0)[0];
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
  return { pos, nrm, idx };
}

const doc = new Document();
doc.getRoot().getAsset().generator = 'navless pitch: obj-to-glb.mjs';
const buffer = doc.createBuffer();
const scene = doc.createScene('dronehacks-course');
const root = doc.createNode('dronehacks-course').setExtras({
  source: `${args.obj ?? `git ${REF}:dronehacks-course-asbuilt.obj`}`,
  frame: 'three.js Y up; from ENU (x east, y north, z up) via (x, z, -y); origin = floor centre; metres',
});
scene.addChild(root);
const baseMat = doc.createMaterial('course').setBaseColorFactor([0.5, 0.5, 0.55, 1]).setMetallicFactor(0).setRoughnessFactor(0.8);

const slug = (s) => s.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '');
const kept = []; // for the scan cloud
let trisIn = 0;
let trisOut = 0;
const report = [];
for (const [name, g] of groups) {
  if (!g.tris.length) continue; // M1/M2/M3 are empty groups
  const cls = classify(name);
  if (!cls) {
    report.push(`skip   ${name}`);
    continue;
  }
  trisIn += g.tris.length;
  let m = buildMesh(g, cls.kind);
  if (cls.kind === 'boulder') m = decimate(m, 500);
  trisOut += m.idx.length / 3;
  const prim = doc
    .createPrimitive()
    .setAttribute('POSITION', doc.createAccessor().setType('VEC3').setArray(m.pos).setBuffer(buffer))
    .setAttribute('NORMAL', doc.createAccessor().setType('VEC3').setArray(m.nrm).setBuffer(buffer))
    .setIndices(doc.createAccessor().setType('SCALAR').setArray(m.pos.length / 3 > 65535 ? m.idx : Uint16Array.from(m.idx)).setBuffer(buffer))
    .setMaterial(baseMat);
  const mesh = doc.createMesh(slug(name)).addPrimitive(prim);
  root.addChild(doc.createNode(slug(name)).setMesh(mesh).setExtras({ group: name, ...cls }));
  kept.push({ cls, m });
  report.push(`keep   ${name.padEnd(42)} ${String(g.tris.length).padStart(6)} -> ${String(m.idx.length / 3).padStart(5)} tris   ${cls.kind}${cls.wall ? ' ' + cls.wall : ''}`);
}

const out = path.join(pitch, 'public', 'models');
fs.mkdirSync(out, { recursive: true });
const glb = await new NodeIO().writeBinary(doc);
fs.writeFileSync(path.join(out, 'dronehacks-course.glb'), glb);

/* ───────────────────────────── scan-style point cloud ───────────────────────────── */
function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const rnd = mulberry32(20261107);
const gauss = () => Math.sqrt(-2 * Math.log(rnd() + 1e-12)) * Math.cos(2 * Math.PI * rnd());
const KIND_ID = { floor: 0, net: 1, wall: 2, tube: 3, boulder: 4, pad: 5 };
const QUOTA = { floor: 5200, net: 5200, wall: 30000, tube: 8000, boulder: 4000, pad: 3400, cradle: 500, other: 300 };

function sampleKind(kind) {
  const meshes = kept.filter((k) => k.cls.kind === kind && !k.cls.roof);
  if (!meshes.length) return [];
  const tris = [];
  let total = 0;
  for (const { m } of meshes)
    for (let t = 0; t < m.idx.length; t += 3) {
      const a = m.idx[t] * 3;
      const b = m.idx[t + 1] * 3;
      const c = m.idx[t + 2] * 3;
      const e1 = [m.pos[b] - m.pos[a], m.pos[b + 1] - m.pos[a + 1], m.pos[b + 2] - m.pos[a + 2]];
      const e2 = [m.pos[c] - m.pos[a], m.pos[c + 1] - m.pos[a + 1], m.pos[c + 2] - m.pos[a + 2]];
      const area = 0.5 * Math.hypot(e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]);
      if (area <= 1e-9) continue;
      total += area;
      tris.push({ m, a, b, c, cum: total });
    }
  const n = QUOTA[kind] ?? 300;
  const pts = [];
  for (let i = 0; i < n; i++) {
    const r = rnd() * total;
    let lo = 0;
    let hi = tris.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (tris[mid].cum < r) lo = mid + 1;
      else hi = mid;
    }
    const { m, a, b, c } = tris[lo];
    let u = rnd();
    let v = rnd();
    if (u + v > 1) {
      u = 1 - u;
      v = 1 - v;
    }
    const w = 1 - u - v;
    const sg = 0.006; // phone LiDAR: about 6 mm
    pts.push([
      m.pos[a] * w + m.pos[b] * u + m.pos[c] * v + gauss() * sg,
      m.pos[a + 1] * w + m.pos[b + 1] * u + m.pos[c + 1] * v + gauss() * sg,
      m.pos[a + 2] * w + m.pos[b + 2] * u + m.pos[c + 2] * v + gauss() * sg,
      KIND_ID[kind] ?? 6,
    ]);
  }
  return pts;
}
const cloud = [];
for (const k of ['floor', 'net', 'wall', 'tube', 'boulder', 'pad', 'cradle', 'other']) cloud.push(...sampleKind(k));
const count = cloud.length;
const bin = Buffer.alloc(8 + count * 12 + count);
bin.writeUInt32LE(0x4e4c5343, 0);
bin.writeUInt32LE(count, 4);
cloud.forEach((p, i) => {
  bin.writeFloatLE(p[0], 8 + i * 12);
  bin.writeFloatLE(p[1], 8 + i * 12 + 4);
  bin.writeFloatLE(p[2], 8 + i * 12 + 8);
  bin.writeUInt8(p[3], 8 + count * 12 + i);
});
fs.writeFileSync(path.join(out, 'course-scan.bin'), bin);

console.log(report.join('\n'));
console.log(`\ntriangles ${trisIn.toLocaleString()} -> ${trisOut.toLocaleString()}   dronehacks-course.glb ${(glb.length / 1024).toFixed(0)} KB   course-scan.bin ${count.toLocaleString()} pts ${(bin.length / 1024).toFixed(0)} KB`);
