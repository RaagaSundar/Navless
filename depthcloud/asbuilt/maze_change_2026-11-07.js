// Replace the maze dividers (M1, M2, M3) in the as-built course .obj with the three new walls, then
// render a top view (SVG) straight from the resulting .obj.   node new_maze.js in.obj out.obj out.svg
const fs = require('fs');
const [, , SRC, DST, SVG] = process.argv;
const IN = 0.0254, NET = 3.048, H = 68 * IN, T = 4 * IN;
const BAR_N = 0.4572 + 2 * IN;                       // barrier line's north face
const NEW = {
  "Maze wall north (new 2026-11-07)":         [[-2.547, 2.985], [-0.830, 2.985]],                // along the north net
  "Maze wall long diagonal (new 2026-11-07)": [[-1.996, BAR_N], [-0.169, NET - 1.0]],            // tip 1 m from the net
  "Maze wall to tube (new 2026-11-07)":       [[-0.182, BAR_N], [1.535, 1.352]],
};
const DROP = new Set(['M1 maze divider', 'M2 maze divider', 'M3 maze divider']);

const lines = fs.readFileSync(SRC, 'utf8').split(/\r?\n/);
let nv = 0, group = '', out = [], dropped = 0;
for (const l of lines) {
  const t = l.split(/\s+/);
  if (t[0] === 'v') nv++;
  else if (t[0] === 'g') group = t.slice(1).join(' ');
  else if (t[0] === 'f' && DROP.has(group)) { dropped++; continue; }
  out.push(l);
}
out.push('', '# --- maze changed on 2026-11-07: M1, M2, M3 removed, three new walls (from Shining\'s sketch;',
         '#     north wall against the net, long diagonal tip 1 m from the north net) ---');
let idx = nv + 1;
for (const [name, [[x0, y0], [x1, y1]]] of Object.entries(NEW)) {
  const L = Math.hypot(x1 - x0, y1 - y0), nx = -(y1 - y0) / L * T / 2, ny = (x1 - x0) / L * T / 2;
  const c = [[x0 - nx, y0 - ny], [x1 - nx, y1 - ny], [x1 + nx, y1 + ny], [x0 + nx, y0 + ny]];
  out.push(`g ${name}`);
  for (const z of [0, H]) for (const [x, y] of c) out.push(`v ${x.toFixed(5)} ${y.toFixed(5)} ${z.toFixed(5)}`);
  const b = idx, u = idx + 4;
  for (const q of [[b, b + 1, b + 2, b + 3], [u + 3, u + 2, u + 1, u], [b, b + 3, u + 3, u], [b + 1, u + 1, u + 2, b + 2],
                   [b, u, u + 1, b + 1], [b + 3, b + 2, u + 2, u + 3]]) out.push('f ' + q.join(' '));
  idx += 8;
  console.log(`${name}: (${x0.toFixed(3)}, ${y0.toFixed(3)}) -> (${x1.toFixed(3)}, ${y1.toFixed(3)}) m, length ${L.toFixed(2)} m, bearing ${((Math.atan2(x1 - x0, y1 - y0) * 180 / Math.PI + 360) % 360).toFixed(1)} deg`);
}
fs.writeFileSync(DST, out.join('\n') + '\n');
console.log(`dropped ${dropped} faces of M1/M2/M3; wrote ${DST}`);

// ---- top view from the NEW .obj: every face of a solid, non-floor group, projected to x/y ----
const V = [], faces = [];
group = '';
for (const l of out) {
  const t = l.trim().split(/\s+/);
  if (t[0] === 'v') V.push([+t[1], +t[2], +t[3]]);
  else if (t[0] === 'g') group = t.slice(1).join(' ');
  else if (t[0] === 'f') faces.push([group, t.slice(1).map(s => +s.split('/')[0] - 1)]);
}
const skip = /^(Net|Floor|Guideline|Approach ring|Landing pad|Launch)/;
const S = 110, PAD = 30, half = 3.1, W = Math.round(2 * half * S + 2 * PAD);
const X = x => (PAD + (x + half) * S).toFixed(1), Y = y => (PAD + (half - y) * S).toFixed(1);
let svg = [`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${W + 70}" font-family="sans-serif" font-size="13">`,
  `<rect width="100%" height="100%" fill="#fff"/>`,
  `<rect x="${X(-NET)}" y="${Y(NET)}" width="${2 * NET * S}" height="${2 * NET * S}" fill="#fbf7f2" stroke="#999" stroke-dasharray="6 4"/>`];
for (let k = -3; k <= 3; k++) svg.push(`<line x1="${X(k)}" y1="${Y(-NET)}" x2="${X(k)}" y2="${Y(NET)}" stroke="#eee"/><line x1="${X(-NET)}" y1="${Y(k)}" x2="${X(NET)}" y2="${Y(k)}" stroke="#eee"/>`);
for (const [g, f] of faces) {
  if (skip.test(g)) continue;
  const pts = f.map(i => V[i]);
  const isNew = g.includes('new 2026-11-07');
  const fill = isNew ? '#1a9b4b' : /Tube/.test(g) ? '#b9c3d6' : /Boulder/.test(g) ? '#c9b8a6' : '#6b6f7a';
  svg.push(`<polygon points="${pts.map(p => X(p[0]) + ',' + Y(p[1])).join(' ')}" fill="${fill}" stroke="${fill}" stroke-width="0.6"/>`);
}
const lm = [-2.5146, 0.9144];
svg.push(`<circle cx="${X(lm[0])}" cy="${Y(lm[1])}" r="6" fill="#e0a000"/><text x="${+X(lm[0]) + 9}" y="${+Y(lm[1]) + 4}">launch</text>`);
const tip = NEW['Maze wall long diagonal (new 2026-11-07)'][1];
svg.push(`<line x1="${X(tip[0])}" y1="${Y(tip[1])}" x2="${X(tip[0])}" y2="${Y(NET)}" stroke="#7b3fb5" stroke-width="2" stroke-dasharray="5 3"/>`,
         `<text x="${+X(tip[0]) + 7}" y="${(+Y(tip[1]) + +Y(NET)) / 2}" fill="#7b3fb5">1 m</text>`);
svg.push(`<text x="${PAD}" y="${W + 25}">Top view of the updated as-built .obj, north up, grid 1 m. Green = the three new maze walls; grey = unchanged walls;</text>`,
         `<text x="${PAD}" y="${W + 45}">blue-grey = tube; tan = boulders. M1, M2, M3 removed.</text>`, '</svg>');
fs.writeFileSync(SVG, svg.join('\n'));
console.log('wrote', SVG);
