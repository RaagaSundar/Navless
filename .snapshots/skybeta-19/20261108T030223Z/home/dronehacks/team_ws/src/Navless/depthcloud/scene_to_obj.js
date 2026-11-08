#!/usr/bin/env node
// Build a stand-in course .obj from tape measurements: a box-shaped room plus tables, chairs and boxes.
//
//   node depthcloud/scene_to_obj.js depthcloud/test_room.json depthcloud/test_room.obj [--up z|y] [--svg plan.svg]
//
// Frame (before --up): origin at a floor corner of the room, metres, Z up.
//   x runs along the room's "x" wall, y along the "y" wall (y is to your LEFT when you stand in the
//   origin corner facing +x), z up. Output is always metres, whatever `units` the JSON uses.
// --up y   writes the common OBJ Y-up convention instead: (x, y, z) -> (x, z, -y).
// --svg    also writes a top-down plan, to check the layout against the real room.
//
// Scene JSON (all lengths in `units`):
//   { "units": "ft" | "m" | "in",
//     "room": { "x": 34, "y": 30, "height": 9 },
//     "defaults": { "table": {...}, "chair": {...} },        // optional, overrides the sizes below
//     "items": [
//       { "type": "table", "name": "T1", "center": [cx, cy], "yaw_deg": 0 },   // long side along x at yaw 0
//       { "type": "table_grid", "name": "T", "rows": 4, "cols": 3, "first_center": [cx, cy],
//         "pitch": [dx, dy], "yaw_deg": 0 },     // rows step along y, columns along x (centre to centre)
//       { "type": "chair", "name": "C1", "center": [cx, cy], "yaw_deg": 90 },  // seat faces +x at yaw 0
//       { "type": "box",   "name": "crate", "center": [cx, cy], "size": [sx, sy, sz], "yaw_deg": 0, "z0": 0 }
//     ] }
//   Any table/chair can override its own size keys (e.g. "length": 8).
//   Tables are modelled as their top slab only: the legs are open frames, and solid leg panels would
//   add surfaces that don't exist. Chairs are a seat slab plus a backrest slab.
//
// Room faces point INTO the room and object faces point OUT, so normals face the camera everywhere.
import fs from "node:fs";

const UNIT = { m: 1, ft: 0.3048, in: 0.0254 };
// in feet: 6 ft x 2.5 ft folding/training table, 30 in high; stacking chair
const DEFAULTS_FT = {
  table: { length: 6, width: 2.5, height: 2.5, top: 0.1 },
  chair: { width: 1.7, depth: 1.7, seat_height: 1.5, back_height: 2.8, thickness: 0.15 },
};

function boxFaces(c, s, yawDeg, z0, inward) {
  const [cx, cy] = c, [sx, sy, sz] = s, a = (yawDeg || 0) * Math.PI / 180;
  const co = Math.cos(a), si = Math.sin(a), v = [];
  for (let i = 0; i < 8; i++) {
    const lx = (i & 1 ? 0.5 : -0.5) * sx, ly = (i & 2 ? 0.5 : -0.5) * sy;
    v.push([cx + co * lx - si * ly, cy + si * lx + co * ly, z0 + (i & 4 ? sz : 0)]);
  }
  // outward-facing, counter-clockwise when seen from outside
  let f = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
  if (inward) f = f.map(q => q.slice().reverse());
  return { v, f };
}

function normal(p0, p1, p2) {
  const a = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]], b = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

// Expand one scene item (scene units) into named boxes in metres: [name, kind, {v, f}]
function expand(item, i, k, defaults) {
  const type = item.type || "box", name = item.name || `${type}_${i + 1}`;
  if (type === "table_grid") {
    // rows step along y, columns along x; first_center is the table nearest the origin.
    // Either pitch entry may be a list of spacings (cols - 1 or rows - 1 of them) for uneven spacing.
    const out = [], [x0, y0] = item.first_center;
    const offsets = (p, n, what) => {
      const gaps = Array.isArray(p) ? p : Array(Math.max(n - 1, 0)).fill(p);
      if (gaps.length !== n - 1) throw new Error(`${item.name}: need ${n - 1} ${what} spacings, got ${gaps.length}`);
      return gaps.reduce((acc, g) => [...acc, acc[acc.length - 1] + g], [0]);
    };
    const xs = offsets(item.pitch[0], item.cols, "column"), ys = offsets(item.pitch[1], item.rows, "row");
    ys.forEach((oy, r) => xs.forEach((ox, c) =>
      out.push(...expand({ ...item, type: "table", name: `${item.name || "T"}_r${r + 1}c${c + 1}`,
        center: [x0 + ox, y0 + oy] }, 0, k, defaults))));
    return out;
  }
  const yaw = item.yaw_deg || 0, c = item.center.map(t => t * k);
  if (type === "box") {
    return [[name, "box", boxFaces(c, item.size.map(t => t * k), yaw, (item.z0 || 0) * k, false)]];
  }
  const d = { ...defaults[type], ...item };
  if (type === "table") {
    return [[name, "table", boxFaces(c, [d.length * k, d.width * k, d.top * k], yaw, (d.height - d.top) * k, false)]];
  }
  if (type === "chair") {
    const a = yaw * Math.PI / 180, back = (d.depth - d.thickness) / 2 * k;      // backrest sits behind the seat
    const bc = [c[0] - Math.cos(a) * back, c[1] - Math.sin(a) * back];
    return [
      [`${name}_seat`, "chair", boxFaces(c, [d.depth * k, d.width * k, d.thickness * k], yaw, (d.seat_height - d.thickness) * k, false)],
      [`${name}_back`, "chair", boxFaces(bc, [d.thickness * k, d.width * k, (d.back_height - d.seat_height) * k], yaw, d.seat_height * k, false)],
    ];
  }
  throw new Error(`item ${name}: unknown type "${type}" (use table, table_grid, chair or box)`);
}

function svgPlan(parts, X, Y, k, units) {
  const s = 600 / Math.max(X, Y), pad = 30, W = X * s + 2 * pad, H = Y * s + 2 * pad;
  // plan view: +x right, +y UP (as seen from above), origin bottom-left
  const P = (x, y) => `${(pad + x * s).toFixed(1)},${(pad + (Y - y) * s).toFixed(1)}`;
  const colour = { table: "#c8875a", chair: "#5a8ac8", box: "#7ab36b" };
  const out = [`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W.toFixed(0)} ${H.toFixed(0)}" font-family="sans-serif" font-size="10">`,
    `<rect width="100%" height="100%" fill="#fff"/>`];
  const step = 5 * k;                                                           // grid every 5 scene units
  for (let g = 0; g <= X + 1e-6; g += step)
    out.push(`<line x1="${(pad + g * s).toFixed(1)}" y1="${pad}" x2="${(pad + g * s).toFixed(1)}" y2="${(pad + Y * s).toFixed(1)}" stroke="#eee"/>`,
      `<text x="${(pad + g * s).toFixed(1)}" y="${(pad + Y * s + 14).toFixed(1)}" text-anchor="middle" fill="#888">${Math.round(g / k)}</text>`);
  for (let g = 0; g <= Y + 1e-6; g += step)
    out.push(`<line x1="${pad}" y1="${(pad + (Y - g) * s).toFixed(1)}" x2="${(pad + X * s).toFixed(1)}" y2="${(pad + (Y - g) * s).toFixed(1)}" stroke="#eee"/>`,
      `<text x="${pad - 6}" y="${(pad + (Y - g) * s + 3).toFixed(1)}" text-anchor="end" fill="#888">${Math.round(g / k)}</text>`);
  out.push(`<rect x="${pad}" y="${pad}" width="${(X * s).toFixed(1)}" height="${(Y * s).toFixed(1)}" fill="none" stroke="#333" stroke-width="2"/>`,
    `<circle cx="${pad}" cy="${(pad + Y * s).toFixed(1)}" r="4" fill="#d33"/>`,
    `<text x="${pad + 6}" y="${(pad + Y * s - 6).toFixed(1)}" fill="#d33">origin</text>`,
    `<text x="${(pad + X * s / 2).toFixed(1)}" y="${(H - 4).toFixed(1)}" text-anchor="middle">x (${units}) →</text>`);
  for (const [name, kind, { v }] of parts) {
    if (kind === "room") continue;
    const foot = [0, 1, 3, 2].map(i => P(v[i][0], v[i][1])).join(" ");
    out.push(`<polygon points="${foot}" fill="${colour[kind]}" fill-opacity="0.75" stroke="#333" stroke-width="0.7"/>`);
    if (kind !== "chair" || name.endsWith("_seat")) {
      const cx = (v[0][0] + v[3][0]) / 2, cy = (v[0][1] + v[3][1]) / 2, [tx, ty] = P(cx, cy).split(",");
      out.push(`<text x="${tx}" y="${(+ty + 3).toFixed(1)}" text-anchor="middle" fill="#111">${name.replace(/_seat$/, "")}</text>`);
    }
  }
  out.push("</svg>");
  return out.join("\n");
}

function main() {
  const args = process.argv.slice(2), opt = {};
  for (const flag of ["--up", "--svg"]) {
    const i = args.indexOf(flag);
    if (i >= 0) opt[flag] = args.splice(i, 2)[1];
  }
  const up = opt["--up"] || "z";
  if (args.length !== 2 || !["z", "y"].includes(up)) {
    console.error("usage: node scene_to_obj.js scene.json out.obj [--up z|y] [--svg plan.svg]");
    process.exit(2);
  }
  const scene = JSON.parse(fs.readFileSync(args[0], "utf8"));
  const units = scene.units || "m", k = UNIT[units];
  if (!k) throw new Error(`unknown units "${scene.units}" (use m, ft or in)`);
  const defaults = {};
  for (const t of Object.keys(DEFAULTS_FT)) {                                   // defaults are in feet; convert to scene units
    defaults[t] = Object.fromEntries(Object.entries(DEFAULTS_FT[t]).map(([n, val]) => [n, val * UNIT.ft / k]));
    Object.assign(defaults[t], (scene.defaults || {})[t]);
  }
  const r = scene.room, X = r.x * k, Y = r.y * k, H = r.height * k;

  const parts = [["room", "room", boxFaces([X / 2, Y / 2], [X, Y, H], 0, 0, true)]];
  const items = [...(scene.items || []), ...(scene.boxes || [])];               // `boxes` kept for older scene files
  items.forEach((it, i) => parts.push(...expand(it, i, k, defaults)));

  // sanity checks in the Z-up frame: room normals point at the room centre, object normals away from the object
  const centre = [X / 2, Y / 2, H / 2], names = new Set();
  for (const [name, kind, { v, f }] of parts) {
    if (names.has(name)) throw new Error(`duplicate name ${name}`);
    names.add(name);
    const bc = v.reduce((acc, p) => acc.map((t, j) => t + p[j] / 8), [0, 0, 0]);
    for (const q of f) {
      const n = normal(v[q[0]], v[q[1]], v[q[2]]);
      const fc = q.reduce((acc, i) => acc.map((t, j) => t + v[i][j] / 4), [0, 0, 0]);
      const ref = kind === "room" ? centre.map((t, j) => t - fc[j]) : fc.map((t, j) => t - bc[j]);
      if (n[0] * ref[0] + n[1] * ref[1] + n[2] * ref[2] <= 0) throw new Error(`bad winding on ${name}`);
    }
    if (kind !== "room" && v.some(p => p[0] < -1e-6 || p[1] < -1e-6 || p[2] < -1e-6 || p[0] > X + 1e-6 || p[1] > Y + 1e-6 || p[2] > H + 1e-6))
      console.warn(`warning: ${name} sticks out of the room`);
  }

  const conv = up === "y" ? p => [p[0], p[2], -p[1]] : p => p;
  const out = [`# stand-in course from ${args[0]} (metres, ${up.toUpperCase()}-up, origin = room floor corner)`];
  let base = 1, nf = 0;
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const [name, , { v, f }] of parts) {
    out.push(`o ${name}`);
    for (const p of v.map(conv)) {
      out.push(`v ${p.map(t => t.toFixed(4)).join(" ")}`);
      p.forEach((t, j) => { lo[j] = Math.min(lo[j], t); hi[j] = Math.max(hi[j], t); });
    }
    for (const q of f) out.push(`f ${q.map(i => i + base).join(" ")}`);
    base += v.length; nf += f.length;
  }
  fs.writeFileSync(args[1], out.join("\n") + "\n");
  const count = kind => parts.filter(([n, kd]) => kd === kind && (kind !== "chair" || n.endsWith("_seat"))).length;
  console.log(`wrote ${args[1]}: room + ${count("table")} tables, ${count("chair")} chairs, ${count("box")} boxes; ${nf} faces, ${up.toUpperCase()}-up`);
  console.log(`bounding box (m): x ${lo[0].toFixed(3)}..${hi[0].toFixed(3)}  y ${lo[1].toFixed(3)}..${hi[1].toFixed(3)}  z ${lo[2].toFixed(3)}..${hi[2].toFixed(3)}`);
  if (opt["--svg"]) {
    fs.writeFileSync(opt["--svg"], svgPlan(parts, X, Y, k, units));
    console.log(`wrote ${opt["--svg"]} (top-down plan, origin bottom-left)`);
  }
}

main();
