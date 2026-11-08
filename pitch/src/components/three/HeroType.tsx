'use client';

import { Text } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useRef } from 'react';
import type * as THREE from 'three';
import { ASSETS } from '@/config/assets';
import { view } from '@/lib/choreography';
import { easeOutExpo, remap, smoothstep } from '@/lib/math';
import { PAL } from '@/lib/palette';

/**
 * Brutalist hero type, set in WebGL so it sits BEHIND the drone with real depth (the drone occludes it), picks up bloom,
 * and parallaxes with the camera. Each line slides in from off-screen, stretched, and tightens as it lands.
 * Scaled by the visible width at its depth so it behaves like vw units at any aspect ratio.
 */

const Z = -2.8;
// A tight stack: the last line has to clear the PLAY DEMO button that sits under the hero (components/hud/DemoControls.tsx).
const LINES = [
  { text: 'NAVLESS:', size: 0.62, y: 1.92, side: -1, style: 'outline' as const, color: PAL.ink },
  { text: 'ZERO GPS.', size: 1.72, y: 0.4, side: 1, style: 'solid' as const, color: PAL.ink },
  { text: 'PURE AUTONOMY.', size: 1.0, y: -1.1, side: -1, style: 'solid' as const, color: PAL.ember },
];
const DESIGN_W = 12.4; // visible world width the sizes above were designed for

type TroikaText = THREE.Mesh & { fillOpacity: number; strokeOpacity: number };

export default function HeroType() {
  const group = useRef<THREE.Group>(null!);
  const lines = useRef<(THREE.Group | null)[]>([]);
  const texts = useRef<(TroikaText | null)[]>([]);

  useFrame((state) => {
    const g = group.current;
    if (!g) return;
    const alive = view.textOut < 0.999 && view.textIn > 0;
    g.visible = alive;
    if (!alive) return;

    const cam = state.camera as THREE.PerspectiveCamera;
    const dist = Math.abs(cam.position.z - Z) + 0.0001;
    const visW = 2 * dist * Math.tan((cam.fov * Math.PI) / 360) * cam.aspect;
    const fit = Math.min(1.5, Math.max(0.42, visW / DESIGN_W));
    g.scale.setScalar(fit);

    for (let k = 0; k < LINES.length; k++) {
      const L = LINES[k];
      const lp = remap(view.textIn, k * 0.16, 0.62 + k * 0.16);
      const e = easeOutExpo(lp);
      const grp = lines.current[k];
      const tx = texts.current[k];
      if (!grp || !tx) continue;
      const off = visW / fit; // slide distance in local units
      grp.position.x = L.side * (1 - e) * off * 0.95;
      grp.position.y = L.y + view.textOut * (0.9 + k * 0.5);
      grp.scale.x = 1 + (1 - e) * 0.35; // stretched on entry, snaps to true width
      const a = smoothstep(0, 0.45, lp) * (1 - view.textOut);
      if (L.style === 'outline') {
        tx.strokeOpacity = a;
        tx.fillOpacity = 0;
      } else {
        tx.fillOpacity = a;
      }
    }
  });

  return (
    <group ref={group} position={[0, 0, Z]} visible={false}>
      {LINES.map((L, k) => (
        <group
          key={L.text}
          ref={(el) => {
            lines.current[k] = el;
          }}
          position={[0, L.y, 0]}
        >
          <Text
            ref={(el: unknown) => {
              texts.current[k] = el as TroikaText | null;
            }}
            font={ASSETS.fontDisplay}
            fontSize={L.size}
            letterSpacing={-0.045}
            lineHeight={0.86}
            anchorX="center"
            anchorY="middle"
            color={L.color}
            fillOpacity={0}
            strokeWidth={L.style === 'outline' ? '3%' : 0}
            strokeColor={L.color}
            strokeOpacity={0}
            sdfGlyphSize={128}
            characters="NAVLESS:ZERO GPS.PURE AUTONOMY."
            material-toneMapped={false}
            frustumCulled={false}
          >
            {L.text}
          </Text>
        </group>
      ))}
    </group>
  );
}
