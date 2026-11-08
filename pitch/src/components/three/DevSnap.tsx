'use client';

import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useRef } from 'react';
import * as THREE from 'three';

/**
 * DEV ONLY (mounted only when NODE_ENV !== 'production'): `window.__snap('name')` saves the next fully post-processed frame of
 * the canvas to pitch/scratch/name.jpg via /api/snap. It runs at priority 2000, after the EffectComposer's render, in the same
 * task, so the drawing buffer is still valid when the blob is taken.
 */
export default function DevSnap() {
  const gl = useThree((s) => s.gl);
  const advance = useThree((s) => s.advance);
  const camera = useThree((s) => s.camera);
  const want = useRef<string | null>(null);
  useEffect(() => {
    (window as unknown as { __snap: (n: string) => void }).__snap = (n: string) => {
      want.current = n;
    };
    // `window.__advance(n)` runs n frames of the R3F loop by hand. A hidden / unfocused window pauses requestAnimationFrame
    // (so the demo clock and every useFrame stop); this keeps QA scripts deterministic when that happens.
    (window as unknown as { __advance: (n?: number) => void }).__advance = (n = 1) => {
      for (let i = 0; i < n; i++) advance(performance.now() / 1000, true);
    };
    // `window.__project(x, y, z)` -> normalised device coords of a three.js world point (QA: is the camera looking where we think?)
    (window as unknown as { __project: (x: number, y: number, z: number) => number[] }).__project = (x, y, z) => {
      const v = new THREE.Vector3(x, y, z).project(camera);
      return [Number(v.x.toFixed(3)), Number(v.y.toFixed(3)), Number(v.z.toFixed(3))];
    };
  }, [advance, camera]);
  useFrame(() => {
    const name = want.current;
    if (!name) return;
    want.current = null;
    gl.domElement.toBlob(
      (b) => {
        if (b) void fetch(`/api/snap?name=${encodeURIComponent(name)}`, { method: 'POST', body: b });
      },
      'image/jpeg',
      0.88,
    );
  }, 2000);
  return null;
}
