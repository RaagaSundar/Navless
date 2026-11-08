'use client';

import { useFrame, useThree } from '@react-three/fiber';
import { useLayoutEffect, useRef } from 'react';
import * as THREE from 'three';
import { updateView, view } from '@/lib/choreography';
import { damp } from '@/lib/math';

/**
 * CameraRig runs FIRST every frame (priority -3) and does two jobs:
 *   1. advances the shared `view` (scroll -> choreography) so everything after it reads a consistent frame
 *   2. eases the real camera toward the choreography target (lerped position / look-at / FOV) and adds impact shake
 * In OVERRIDE MODE the projection is nudged left with a view offset so the maze sits clear of the terminal panel.
 */
/**
 * three's FOV is vertical, so a portrait window sees a sliver of the scene (the drone and the type get cropped at the
 * sides). Below an aspect of ~1.2 widen it, gently: 768x1024 ends up near 40 deg, a 390x844 phone near 53 deg.
 */
const effFov = (fov: number, aspect: number) => {
  if (aspect >= 1.2) return fov;
  const k = Math.min(1.9, Math.pow(1.2 / Math.max(aspect, 0.3), 0.65));
  return (2 * Math.atan(Math.tan((fov * Math.PI) / 360) * k) * 180) / Math.PI;
};

export default function CameraRig() {
  const basePos = useRef(new THREE.Vector3());
  const look = useRef(new THREE.Vector3());
  const lastShift = useRef(0);
  const camera = useThree((s) => s.camera) as THREE.PerspectiveCamera;

  useLayoutEffect(() => {
    updateView(1 / 60, 0);
    basePos.current.copy(view.camPos);
    look.current.copy(view.camLook);
    camera.position.copy(view.camPos);
    camera.fov = effFov(view.camFov, camera.aspect);
    camera.lookAt(view.camLook);
    camera.updateProjectionMatrix();
  }, [camera]);

  useFrame((state, dtRaw) => {
    const dt = Math.min(dtRaw, 0.05);
    updateView(dt, state.clock.elapsedTime);

    const w = state.size.width;
    const h = state.size.height;
    const targetFov = effFov(view.camFov, w / h);

    const snapping = view.snap > 0;
    const lp = view.directed ? 14 : 5.5; // on the director's rails the path itself is already smooth: just follow it closely
    const ll = view.directed ? 18 : 7;
    const k = snapping ? 1 : 1 - Math.exp(-lp * dt);
    basePos.current.lerp(view.camPos, k);
    look.current.lerp(view.camLook, snapping ? 1 : 1 - Math.exp(-ll * dt));
    if (snapping) {
      camera.fov = targetFov;
      camera.updateProjectionMatrix();
      view.snap--;
    }

    const t = state.clock.elapsedTime;
    const s = view.shake;
    camera.position.copy(basePos.current);
    if (s > 0.002) {
      // a soft settle, not a shake: one slow, tiny sway that dies away (the old 9-12 Hz jitter is gone)
      camera.position.y += Math.sin(t * 7) * 0.012 * s;
    }

    const fov = damp(camera.fov, targetFov, 6, dt);
    let dirty = Math.abs(fov - camera.fov) > 1e-4;
    camera.fov = fov;

    // push the scene left in override so the terminal doesn't cover the maze (wide screens only)
    const shift = w > 980 ? view.o * w * 0.1 : 0;
    if (Math.abs(shift - lastShift.current) > 0.25) {
      lastShift.current = shift;
      if (shift > 0.25) camera.setViewOffset(w, h, shift, 0, w, h);
      else camera.clearViewOffset();
      dirty = true;
    }
    if (dirty) camera.updateProjectionMatrix();

    camera.lookAt(look.current);
    camera.updateMatrixWorld();
  }, -3);

  return null;
}
