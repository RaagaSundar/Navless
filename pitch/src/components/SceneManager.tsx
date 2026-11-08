'use client';

import { PerformanceMonitor, Preload } from '@react-three/drei';
import { Canvas } from '@react-three/fiber';
import { Suspense } from 'react';
import { useDeck } from '@/lib/store';
import AutoDirector from './AutoDirector';
import DroneModel from './DroneModel';
import InteractiveJudgeOverride from './InteractiveJudgeOverride';
import SceneEnvironment from './SceneEnvironment';
import CameraRig from './three/CameraRig';
import Effects from './three/Effects';
import FeatureCloud from './three/FeatureCloud';
import HeroType from './three/HeroType';
import DevSnap from './three/DevSnap';
import Dust from './three/Dust';
import Sky from './three/Sky';

/**
 * SceneManager: the one persistent WebGL canvas behind the whole deck.
 *
 * Order matters. AutoDirector (priority -4) and CameraRig (priority -3) sit outside <Suspense> and run first each
 * frame, so the shared `view` is always fresh, even while a model is still streaming in. Everything inside <Suspense>
 * reads `view`. <PerformanceMonitor> walks the quality tier down (DPR, MSAA, aberration) if the GPU can't hold frame rate.
 */
const DPR = { high: [1, 1.75], medium: [1, 1.25], low: [0.75, 1] } as const;

export default function SceneManager() {
  const quality = useDeck((s) => s.quality);
  const setQuality = useDeck((s) => s.setQuality);
  const locked = useDeck((s) => s.qualityLocked);

  return (
    <Canvas
      flat // tone mapping is done once, in <Effects />
      dpr={[...DPR[quality]] as [number, number]}
      frameloop="always"
      gl={{ antialias: false, alpha: false, stencil: false, powerPreference: 'high-performance' }}
      camera={{ fov: 30, near: 0.05, far: 140, position: [0, 0.35, 10.8] }}
      style={{ position: 'fixed', inset: 0 }}
      onCreated={({ gl }) => gl.setClearColor('#0d0807', 1)}
    >
      <Sky />
      {locked ? null : (
        <PerformanceMonitor
          flipflops={3}
          onDecline={() => setQuality(useDeck.getState().quality === 'high' ? 'medium' : 'low')}
          onFallback={() => setQuality('low')}
        />
      )}
      <AutoDirector />
      <CameraRig />
      {process.env.NODE_ENV !== 'production' ? <DevSnap /> : null}
      <Suspense fallback={null}>
        <SceneEnvironment />
        <HeroType />
        <FeatureCloud />
        <Dust />
        <DroneModel />
        <InteractiveJudgeOverride />
        <Effects />
        <Preload all />
      </Suspense>
    </Canvas>
  );
}
