import type { NextConfig } from 'next';
import path from 'node:path';

const root = path.resolve(__dirname);

const nextConfig: NextConfig = {
  reactStrictMode: true,
  // three ships ESM that Next should compile with the app so R3F/drei share a single copy.
  transpilePackages: ['three'],
  // The Navless repo root also has a package.json/lockfile (the Slack MCP server). Pin the workspace root
  // to this folder so Next never walks up and picks the wrong lockfile.
  turbopack: { root },
  outputFileTracingRoot: root,
  poweredByHeader: false,
  images: { unoptimized: true },
  async headers() {
    // CAD + course assets never change between deploys of a given build: let the browser keep them (production only,
    // so regenerating a model in dev is never masked by the cache; the ?v= query in config/assets.ts busts it in prod).
    if (process.env.NODE_ENV !== 'production') return [];
    return [
      {
        source: '/models/:path*',
        headers: [{ key: 'Cache-Control', value: 'public, max-age=31536000, immutable' }],
      },
      {
        source: '/fonts/:path*',
        headers: [{ key: 'Cache-Control', value: 'public, max-age=31536000, immutable' }],
      },
    ];
  },
};

export default nextConfig;
