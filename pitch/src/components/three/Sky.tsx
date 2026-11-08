'use client';

import { useThree } from '@react-three/fiber';
import { useLayoutEffect } from 'react';
import * as THREE from 'three';
import { PAL } from '@/lib/palette';

/**
 * The dusk backdrop. A vertical gradient painted once into a canvas texture and drawn behind everything in screen space, with a
 * low, wide sun glow on the horizon. It is static: nothing in it moves or pulses, so it can never flicker.
 *
 * A hair of fixed (not animated) dither keeps the dark gradient from banding on 8-bit displays.
 */
function makeSky(): THREE.CanvasTexture {
  const W = 1024;
  const H = 1024;
  const c = document.createElement('canvas');
  c.width = W;
  c.height = H;
  const g = c.getContext('2d')!;

  const v = g.createLinearGradient(0, 0, 0, H);
  v.addColorStop(0.0, PAL.duskTop);
  v.addColorStop(0.46, '#190d0b');
  v.addColorStop(0.7, PAL.duskMid);
  v.addColorStop(0.86, '#5f2e15');
  v.addColorStop(0.94, PAL.duskGlow);
  v.addColorStop(1.0, '#1d0e09');
  g.fillStyle = v;
  g.fillRect(0, 0, W, H);

  // the low sun: a wide soft glow sitting on the horizon
  const r = g.createRadialGradient(W * 0.5, H * 0.9, 0, W * 0.5, H * 0.9, W * 0.62);
  r.addColorStop(0, 'rgba(255,176,98,0.30)');
  r.addColorStop(0.38, 'rgba(255,128,62,0.14)');
  r.addColorStop(1, 'rgba(255,128,62,0)');
  g.fillStyle = r;
  g.fillRect(0, 0, W, H);

  // fixed dither (deterministic, so it never changes between frames or loads)
  const img = g.getImageData(0, 0, W, H);
  let s = 1337;
  for (let i = 0; i < img.data.length; i += 4) {
    s = (s * 1664525 + 1013904223) >>> 0;
    const n = ((s >>> 24) / 255 - 0.5) * 3;
    img.data[i] = Math.max(0, Math.min(255, img.data[i] + n));
    img.data[i + 1] = Math.max(0, Math.min(255, img.data[i + 1] + n));
    img.data[i + 2] = Math.max(0, Math.min(255, img.data[i + 2] + n));
  }
  g.putImageData(img, 0, 0);

  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.minFilter = THREE.LinearFilter;
  t.generateMipmaps = false;
  return t;
}

export default function Sky() {
  const scene = useThree((s) => s.scene);
  useLayoutEffect(() => {
    const t = makeSky();
    scene.background = t;
    scene.fog = new THREE.Fog(PAL.umber, 24, 72);
    return () => {
      scene.background = null;
      scene.fog = null;
      t.dispose();
    };
  }, [scene]);
  return null;
}
