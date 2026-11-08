'use client';

import { extend, useFrame, useThree, type ThreeElement } from '@react-three/fiber';
import { MeshLineGeometry, MeshLineMaterial } from 'meshline';
import { forwardRef, useImperativeHandle, useLayoutEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';

extend({ MeshLineGeometry, MeshLineMaterial });

declare module '@react-three/fiber' {
  interface ThreeElements {
    meshLineGeometry: ThreeElement<typeof MeshLineGeometry>;
    meshLineMaterial: ThreeElement<typeof MeshLineMaterial>;
  }
}

/** Imperative handle: drive `visibility` (progressive draw) and `dashOffset` (flow) from useFrame with no React renders. */
export interface GlowLineHandle {
  readonly material: MeshLineMaterial;
  readonly mesh: THREE.Mesh;
}

interface GlowLineProps {
  points: THREE.Vector3[];
  /** screen-space width in px (size attenuation is off, so it holds at any camera distance) */
  width?: number;
  from: THREE.ColorRepresentation;
  to: THREE.ColorRepresentation;
  /** HDR multiplier. Anything over ~1 clears the bloom threshold, which is what makes the line glow. */
  boost?: number;
  opacity?: number;
  additive?: boolean;
  renderOrder?: number;
  /** dash pattern in fractions of the line length: { array: 0.03, ratio: 0.5 } */
  dashed?: { array: number; ratio: number } | null;
  visible?: boolean;
}

/**
 * GlowLine: a MeshLine ribbon (meshline's MeshLineGeometry + MeshLineMaterial) with a two-colour gradient along its
 * length. This is the line the route, the A* plan and the invalidated old plan are drawn with.
 */
export const GlowLine = forwardRef<GlowLineHandle, GlowLineProps>(function GlowLine(
  { points, width = 4, from, to, boost = 2, opacity = 1, additive = false, renderOrder = 0, dashed = null, visible = true },
  ref,
) {
  const size = useThree((s) => s.size);
  const mesh = useRef<THREE.Mesh>(null!);
  const mat = useRef<MeshLineMaterial>(null!);
  const widthPx = useRef(width);
  widthPx.current = width;
  const geometry = useMemo(() => new MeshLineGeometry(), []);
  const gradient = useMemo(() => [new THREE.Color(from).multiplyScalar(boost), new THREE.Color(to).multiplyScalar(boost)], [from, to, boost]);
  const resolution = useMemo(() => new THREE.Vector2(1, 1), []);
  resolution.set(size.width, size.height);
  // meshline's constructor wants a parameters object; keep it referentially stable so R3F builds the material once
  const args = useMemo(() => [{ resolution }] as ConstructorParameters<typeof MeshLineMaterial>, [resolution]);

  useLayoutEffect(() => {
    geometry.setPoints(points);
  }, [geometry, points]);
  // With sizeAttenuation off, meshline's on-screen width is lineWidth / (2 * P00), P00 being the projection's x scale.
  // Solve for it every frame so `width` really is CSS pixels at any FOV (the override camera is 18 deg, the hero 30).
  useFrame((state) => {
    const m = mat.current;
    if (m) m.lineWidth = 2 * state.camera.projectionMatrix.elements[0] * widthPx.current;
  }, -1);
  useLayoutEffect(() => () => geometry.dispose(), [geometry]);
  useImperativeHandle(
    ref,
    () => ({
      get material() {
        return mat.current;
      },
      get mesh() {
        return mesh.current;
      },
    }),
    [],
  );

  return (
    <mesh ref={mesh} geometry={geometry} frustumCulled={false} renderOrder={renderOrder} visible={visible}>
      <meshLineMaterial
        ref={mat}
        args={args}
        transparent
        depthWrite={false}
        toneMapped={false}
        lineWidth={width}
        resolution={resolution}
        sizeAttenuation={0}
        useGradient={1}
        gradient={gradient}
        opacity={opacity}
        blending={additive ? THREE.AdditiveBlending : THREE.NormalBlending}
        // meshline's `dashArray` setter flips `useDash` on for ANY non-zero value, so a solid line must get exactly 0.
        // Props apply in order, so `useDash` goes last and the result never depends on the dashArray default.
        dashArray={dashed?.array ?? 0}
        dashRatio={dashed?.ratio ?? 0.5}
        useDash={dashed ? 1 : 0}
      />
    </mesh>
  );
});
