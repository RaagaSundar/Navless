'use client';

import { Bloom, ChromaticAberration, EffectComposer, ToneMapping, Vignette } from '@react-three/postprocessing';
import { ToneMappingMode } from 'postprocessing';
import { useMemo } from 'react';
import * as THREE from 'three';
import { useDeck } from '@/lib/store';
import { Grade } from './Grade';

/**
 * Post stack, kept deliberately quiet.
 *   Bloom   a soft glow on the things that are meant to glow (the route, the plan, sun glints on the chrome). The threshold is
 *           high, so walls and ground stay crisp instead of washing out.
 *   CA      chromatic aberration, edge-weighted and fixed at a hair. It does not react to scroll or impacts.
 *   Grade   golden-hour split tone: cool shadows, warm highlights.
 *   Vignette, ACES to finish.
 * No noise or grain pass: anything that changes every frame reads as flicker. `quality` is walked down by the
 * PerformanceMonitor in SceneManager; the cheap tiers drop MSAA and the aberration pass.
 */
export default function Effects() {
  const quality = useDeck((s) => s.quality);
  const offset = useMemo(() => new THREE.Vector2(0.0005, 0.0004), []);

  const high = quality === 'high';
  return (
    <EffectComposer multisampling={high ? 4 : 0} enableNormalPass={false}>
      <Bloom intensity={high ? 0.62 : 0.5} luminanceThreshold={0.62} luminanceSmoothing={0.3} mipmapBlur radius={high ? 0.78 : 0.65} />
      {quality !== 'low' ? <ChromaticAberration offset={offset} radialModulation modulationOffset={0.35} /> : <></>}
      <Grade />
      <Vignette offset={0.32} darkness={0.62} eskil={false} />
      <ToneMapping mode={ToneMappingMode.ACES_FILMIC} />
    </EffectComposer>
  );
}
