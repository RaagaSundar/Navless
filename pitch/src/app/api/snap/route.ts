import { existsSync } from 'node:fs';
import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';

/**
 * DEV ONLY. The scene's snapshot hook (components/three/DevSnap.tsx) posts a JPEG of the WebGL canvas here so it can be
 * inspected at full resolution. Writes pitch/scratch/<name>.jpg (gitignored). Returns 404 in production builds.
 */
export async function POST(req: Request) {
  if (process.env.NODE_ENV === 'production') return new Response('not found', { status: 404 });
  const cwd = process.cwd();
  const root = existsSync(path.join(cwd, 'pitch', 'package.json')) ? path.join(cwd, 'pitch') : cwd;
  const dir = path.join(root, 'scratch');
  await mkdir(dir, { recursive: true });
  const name = (new URL(req.url).searchParams.get('name') ?? 'snap').replace(/[^a-z0-9_-]/gi, '');
  const buf = Buffer.from(await req.arrayBuffer());
  await writeFile(path.join(dir, `${name}.jpg`), buf);
  return Response.json({ ok: true, bytes: buf.length, file: `scratch/${name}.jpg` });
}
