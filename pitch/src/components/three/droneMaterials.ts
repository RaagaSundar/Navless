import * as THREE from 'three';

/**
 * Dark liquid-chrome material system for the drone.
 *
 * The CAD's own materials are discarded. Every mesh is re-skinned by ROLE (written into the GLB by scripts/step-to-glb.mjs).
 * Structural roles share the same physical recipe: metalness 1.0, roughness 0.1, clearcoat 1.0. Only the base tint differs,
 * which is what separates dark mirror chrome (arms, motor bells) from lighter polished steel (plates, standoffs).
 *
 * Every material gets the same small shader patch:
 *   1. DISSOLVE-IN   a scan front sweeps outward from the Jetson; behind it the CAD is solid, at the front it glows gold-white.
 *   2. LIQUID RIPPLE slow animated noise on the shading normal: the chrome reads as mercury, not machined steel.
 *   3. FRESNEL RIM   a golden edge light, the low sun catching the silhouette (a touch hotter in override).
 * All patched materials share ONE uniforms object, so one write per frame updates the whole drone.
 */

export interface DroneUniforms {
  uTime: { value: number };
  uReveal: { value: number };
  /** where the dissolve starts, in drone space (the Jetson board) and how far it has to travel to reach the prop tips */
  uRevealCenter: { value: THREE.Vector3 };
  uRevealRadius: { value: number };
  /** inverse of the drone's world matrix: lets every mesh (props included) reveal in drone space */
  uDroneInv: { value: THREE.Matrix4 };
  uScanColor: { value: THREE.Color };
  uAccent: { value: THREE.Color };
  uRipple: { value: number };
  uRim: { value: number };
}

export function createDroneUniforms(): DroneUniforms {
  return {
    uTime: { value: 0 },
    uReveal: { value: 0 },
    // the CAD scans in outward from the Jetson Orin Nano board (centre of the stack, in GLB space)
    uRevealCenter: { value: new THREE.Vector3(0, -0.039, -0.029) },
    uRevealRadius: { value: 0.3 },
    uDroneInv: { value: new THREE.Matrix4() },
    uScanColor: { value: new THREE.Color('#ffc98a') },
    uAccent: { value: new THREE.Color('#ffb868') },
    uRipple: { value: 0.045 },
    uRim: { value: 0.12 },
  };
}

const NOISE = /* glsl */ `
  float hash31(vec3 p){ p = fract(p * 0.3183099 + vec3(.1,.2,.3)); p *= 17.0; return fract(p.x * p.y * p.z * (p.x + p.y + p.z)); }
  float vnoise(vec3 x){
    vec3 i = floor(x), f = fract(x); f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(hash31(i), hash31(i + vec3(1,0,0)), f.x), mix(hash31(i + vec3(0,1,0)), hash31(i + vec3(1,1,0)), f.x), f.y),
               mix(mix(hash31(i + vec3(0,0,1)), hash31(i + vec3(1,0,1)), f.x), mix(hash31(i + vec3(0,1,1)), hash31(i + vec3(1,1,1)), f.x), f.y), f.z);
  }
`;

type Patchable = THREE.MeshStandardMaterial | THREE.MeshPhysicalMaterial;

export function patchMaterial(mat: Patchable, u: DroneUniforms, key: string, opts: { ripple?: number; rim?: number } = {}) {
  const ripple = (opts.ripple ?? 1).toFixed(3);
  const rim = (opts.rim ?? 1).toFixed(3);
  mat.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, u);
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vObjPos;\nuniform mat4 uDroneInv;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\nvObjPos = (uDroneInv * modelMatrix * vec4(position, 1.0)).xyz;');
    shader.fragmentShader = shader.fragmentShader
      .replace(
        '#include <common>',
        `#include <common>
         varying vec3 vObjPos;
         uniform float uTime; uniform float uReveal; uniform vec3 uRevealCenter; uniform float uRevealRadius;
         uniform vec3 uScanColor; uniform vec3 uAccent; uniform float uRipple; uniform float uRim;
         ${NOISE}`,
      )
      .replace(
        '#include <clipping_planes_fragment>',
        `#include <clipping_planes_fragment>
         float revealH = length(vObjPos - uRevealCenter) / uRevealRadius;
         float revealEdge = uReveal * 1.12 - revealH;
         if (revealEdge < 0.0) discard;`,
      )
      .replace(
        '#include <normal_fragment_maps>',
        `#include <normal_fragment_maps>
         {
           vec3 q = vObjPos * 46.0 + vec3(uTime * 0.18, uTime * 0.11, -uTime * 0.14);
           vec3 n = vec3(vnoise(q), vnoise(q + 17.3), vnoise(q + 41.7)) - 0.5;
           normal = normalize(normal + n * uRipple * ${ripple});
         }`,
      )
      .replace(
        '#include <emissivemap_fragment>',
        `#include <emissivemap_fragment>
         float scanGlow = 1.0 - smoothstep(0.0, 0.085, revealEdge);
         totalEmissiveRadiance += uScanColor * scanGlow * 3.2;
         float fres = pow(1.0 - saturate(dot(normalize(vViewPosition), normal)), 5.0);
         totalEmissiveRadiance += uAccent * fres * uRim * ${rim};`,
      );
  };
  mat.customProgramCacheKey = () => `navless-drone-${key}-${ripple}-${rim}`;
  return mat;
}

export type RoleMaterials = Record<string, THREE.Material>;

/** The recipe the brief asks for, applied to every structural role. */
export const LIQUID_CHROME = {
  metalness: 1.0,
  roughness: 0.1,
  clearcoat: 1.0,
  clearcoatRoughness: 0.04,
} as const;

/** role -> material. Roles are written into the GLB by scripts/step-to-glb.mjs (node.extras.role). */
export function createRoleMaterials(u: DroneUniforms): RoleMaterials {
  const phys = (key: string, p: THREE.MeshPhysicalMaterialParameters, o?: { ripple?: number; rim?: number }) =>
    patchMaterial(new THREE.MeshPhysicalMaterial(p), u, key, o);
  const std = (key: string, p: THREE.MeshStandardMaterialParameters, o?: { ripple?: number; rim?: number }) =>
    patchMaterial(new THREE.MeshStandardMaterial(p), u, key, o);
  const chrome = (key: string, color: string, env: number, o?: { ripple?: number; rim?: number }, over: THREE.MeshPhysicalMaterialParameters = {}) =>
    phys(key, { ...LIQUID_CHROME, color, envMapIntensity: env, ...over }, o);

  const matte = std('matte', { color: '#100c0b', metalness: 0.35, roughness: 0.55, envMapIntensity: 0.7 }, { ripple: 0.2, rim: 0.7 });

  // Tints are warm gunmetal: the chrome picks its colour from the golden-hour environment, the tint only keeps it from
  // reading as cold blue-grey steel next to it.
  return {
    // ── dark liquid chrome (metalness 1.0 / roughness 0.1 / clearcoat 1.0)
    plate: chrome('plate', '#6d6560', 2.1, { ripple: 1.6 }),
    arm: chrome('arm', '#14100f', 2.1, { ripple: 1.2, rim: 1.4 }),
    chrome: chrome('chrome', '#d6cbc2', 2.1),
    'motor-bell': chrome('motor', '#1f1b1a', 2.1, { rim: 1.3 }),
    heatsink: chrome('heatsink', '#b5aca4', 1.8, undefined, { roughness: 0.18 }),
    port: chrome('port', '#a39a92', 1.4, undefined, { roughness: 0.26 }),
    // ── electronics
    jetson: matte,
    dark: matte,
    battery: std('battery', { color: '#0b0908', metalness: 0.15, roughness: 0.5, envMapIntensity: 0.5 }, { ripple: 0, rim: 0.5 }),
    pcb: std('pcb', { color: '#0a0706', metalness: 0.35, roughness: 0.38, envMapIntensity: 0.9, emissive: new THREE.Color('#4a2410'), emissiveIntensity: 0.5 }, { ripple: 0, rim: 0.6 }),
    pixhawk: std('pixhawk', { color: '#0e0b0a', metalness: 0.6, roughness: 0.35, envMapIntensity: 1.0 }, { ripple: 0.3, rim: 1.0 }),
    'oak-body': chrome('oak', '#15110f', 1.2, { rim: 1.2 }, { metalness: 0.85, roughness: 0.28, clearcoat: 0.5, clearcoatRoughness: 0.15 }),
    // ── glass + emissives (these are what the bloom pass catches). The lens glows ice-blue: cool = what the drone perceives.
    lens: patchMaterial(
      new THREE.MeshStandardMaterial({ color: '#000000', emissive: new THREE.Color('#9cc6dd'), emissiveIntensity: 2.6, roughness: 0.05, metalness: 0, toneMapped: false }),
      u,
      'lens',
      { ripple: 0, rim: 0 },
    ),
    glass: phys('glass', { color: '#050304', metalness: 0.3, roughness: 0.02, envMapIntensity: 2.0, clearcoat: 1, clearcoatRoughness: 0.01 }, { ripple: 0, rim: 1.4 }),
    led: patchMaterial(
      new THREE.MeshStandardMaterial({ color: '#000000', emissive: new THREE.Color('#ff5a3c'), emissiveIntensity: 2.4, toneMapped: false }),
      u,
      'led',
      { ripple: 0, rim: 0 },
    ),
    'battery-label': patchMaterial(
      new THREE.MeshStandardMaterial({ color: '#000000', emissive: new THREE.Color('#e0592a'), emissiveIntensity: 1.6, roughness: 0.6, toneMapped: false }),
      u,
      'label',
      { ripple: 0, rim: 0 },
    ),
  };
}
