import * as THREE from 'three';
import type { DroneUniforms } from './droneMaterials';

/**
 * The Sky Beta CAD ships without propellers, so we build them: a tri-blade 6.1 in rotor (the radius the repo's drone
 * model uses) plus a motion-blur disc that fades in with spin speed. Hung on the GLB's `prop_*` empties.
 */

let bladeGeo: THREE.BufferGeometry | null = null;
function blade(radius: number) {
  if (bladeGeo) return bladeGeo;
  const s = new THREE.Shape();
  const R = radius;
  s.moveTo(0.01, -0.006);
  s.bezierCurveTo(0.03, -0.014, 0.055, -0.015, 0.075, -0.0095);
  s.bezierCurveTo(R * 0.93, -0.005, R * 0.99, -0.002, R, 0.0);
  s.bezierCurveTo(R * 0.99, 0.003, R * 0.92, 0.006, 0.07, 0.0085);
  s.bezierCurveTo(0.05, 0.011, 0.03, 0.01, 0.01, 0.006);
  s.closePath();
  const g = new THREE.ExtrudeGeometry(s, { depth: 0.0016, bevelEnabled: false, curveSegments: 14 });
  g.rotateX(-Math.PI / 2); // lie flat in XZ
  g.translate(0, -0.0008, 0);
  g.computeVertexNormals();
  bladeGeo = g;
  return g;
}

export interface Rotor {
  group: THREE.Group;
  disc: THREE.Mesh<THREE.RingGeometry, THREE.ShaderMaterial>;
  dir: number;
}

export function buildRotor(radius: number, dir: number, bladeMat: THREE.Material, hubMat: THREE.Material, u: DroneUniforms): Rotor {
  const group = new THREE.Group();
  const bg = blade(radius);
  for (let i = 0; i < 3; i++) {
    const holder = new THREE.Group();
    holder.rotation.y = (i * Math.PI * 2) / 3;
    const b = new THREE.Mesh(bg, bladeMat);
    b.rotation.x = 0.24 * dir; // pitch
    holder.add(b);
    group.add(holder);
  }
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.0105, 0.0125, 0.009, 24), hubMat);
  hub.position.y = 0.002;
  group.add(hub);
  const nut = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.006, 6), hubMat);
  nut.position.y = 0.009;
  group.add(nut);

  const discMat = new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    side: THREE.DoubleSide,
    toneMapped: false,
    uniforms: { uOmega: { value: 0 }, uTime: u.uTime, uColor: u.uAccent, uR: { value: radius } },
    vertexShader: /* glsl */ `
      varying vec2 vP;
      void main(){ vP = position.xy; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }
    `,
    fragmentShader: /* glsl */ `
      uniform float uOmega; uniform float uTime; uniform vec3 uColor; uniform float uR;
      varying vec2 vP;
      void main(){
        float r = length(vP) / uR;
        float a = atan(vP.y, vP.x);
        float swirl = 0.5 + 0.5 * sin(a * 3.0 - uTime * 5.0 + r * 7.0);
        float tip = smoothstep(0.30, 0.96, r) * (1.0 - smoothstep(0.97, 1.0, r));
        float alpha = uOmega * (0.02 + 0.07 * tip * (0.6 + 0.4 * swirl));
        vec3 tint = mix(vec3(1.0), uColor, 0.45);
        gl_FragColor = vec4(tint * (0.7 + 1.0 * tip), alpha);
      }
    `,
  });
  const disc = new THREE.Mesh(new THREE.RingGeometry(0.012, radius * 1.01, 72, 1), discMat);
  disc.rotation.x = -Math.PI / 2;
  disc.position.y = 0.001;
  disc.renderOrder = 4;
  group.add(disc);
  return { group, disc, dir };
}
