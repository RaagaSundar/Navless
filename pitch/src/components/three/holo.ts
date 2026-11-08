import * as THREE from 'three';

/**
 * Hologram material kit for the maze. Everything shares one set of uniforms (`holo`): write the theme colour once and the
 * floor grid and the scan points re-theme together (copper-gold in the tour, a touch redder in OVERRIDE). The walls, edges,
 * net and pad stay sand: the world is warm and quiet so the route and the plan are the brightest things in the frame.
 * Per-wall state (`uRise`, `uAlpha`) lives on each material. Nothing here pulses or flickers: the only motion is the slow
 * rise of the walls and a very slow scan-line drift.
 */
export const holo = {
  uTime: { value: 0 },
  /** the theme colour: copper-gold in the tour, redder in OVERRIDE. Floor grid + scan points follow it. */
  uAccent: { value: new THREE.Color('#d99a55') },
  /** the maze itself: sand */
  uWorld: { value: new THREE.Color('#f0d2a4') },
  uOverride: { value: 0 },
};

/** translucent glass wall: fresnel body, faint rising scan-lines, a soft top edge, a glowing cut front while it rises */
export function wallMaterial(height: number): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    side: THREE.DoubleSide,
    toneMapped: false,
    uniforms: {
      uTime: holo.uTime,
      uAccent: holo.uWorld,
      uRise: { value: 0 },
      uH: { value: height },
      uAlpha: { value: 1 },
      uBase: { value: new THREE.Color('#120907') },
    },
    vertexShader: /* glsl */ `
      varying vec3 vW; varying vec3 vN; varying vec3 vV;
      void main(){
        vec4 wp = modelMatrix * vec4(position, 1.0);
        vW = wp.xyz; vN = normalize(mat3(modelMatrix) * normal); vV = cameraPosition - wp.xyz;
        gl_Position = projectionMatrix * viewMatrix * wp;
      }
    `,
    fragmentShader: /* glsl */ `
      uniform float uTime; uniform float uRise; uniform float uH; uniform float uAlpha; uniform vec3 uAccent; uniform vec3 uBase;
      varying vec3 vW; varying vec3 vN; varying vec3 vV;
      void main(){
        float hh = vW.y / uH;
        if (hh > uRise) discard;
        float front = 1.0 - smoothstep(0.0, 0.07, uRise - hh);
        float fres = pow(1.0 - abs(dot(normalize(vN), normalize(vV))), 2.0);
        float scan = 0.5 + 0.5 * sin(vW.y * 26.0 - uTime * 0.7);
        float top = smoothstep(0.78, 1.0, hh);
        vec3 col = uBase + uAccent * (0.07 * fres + 0.025 * scan * (0.3 + hh) + top * 0.1);
        col += uAccent * front * 0.9;
        float a = uAlpha * (0.05 + 0.28 * fres + 0.03 * scan + front * 0.55 + top * 0.04);
        gl_FragColor = vec4(col, clamp(a, 0.0, 1.0));
      }
    `,
  });
}

/** crisp edges (also used with wireframe:true for the tunnel and boulders) */
export function lineMaterial(height: number, color: THREE.ColorRepresentation | 'accent', opts: { wireframe?: boolean; alpha?: number } = {}): THREE.ShaderMaterial {
  const c = new THREE.Color(color === 'accent' ? '#ffffff' : color);
  return new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    wireframe: opts.wireframe ?? false,
    toneMapped: false,
    uniforms: {
      uAccent: holo.uWorld,
      uRise: { value: 0 },
      uH: { value: height },
      uColor: { value: c },
      uUseAccent: { value: color === 'accent' ? 1 : 0 },
      uAlpha: { value: opts.alpha ?? 1 },
    },
    vertexShader: /* glsl */ `
      varying vec3 vW;
      void main(){ vec4 wp = modelMatrix * vec4(position, 1.0); vW = wp.xyz; gl_Position = projectionMatrix * viewMatrix * wp; }
    `,
    fragmentShader: /* glsl */ `
      uniform float uRise; uniform float uH; uniform vec3 uColor; uniform vec3 uAccent; uniform float uUseAccent; uniform float uAlpha;
      varying vec3 vW;
      void main(){
        float hh = vW.y / uH;
        if (hh > uRise + 0.002) discard;
        float front = 1.0 - smoothstep(0.0, 0.05, uRise - hh);
        float top = smoothstep(0.9, 1.0, hh);
        vec3 base = mix(uColor, uAccent * 1.15, uUseAccent);
        vec3 col = base * (0.36 + 0.2 * top + 0.9 * front);
        gl_FragColor = vec4(col, uAlpha * (0.42 + 0.2 * top + 0.6 * front));
      }
    `,
  });
}

/** the reactive floor: 10 cm + 50 cm grid, a ring that travels outward from the launch mark as the map reveals */
export function floorMaterial(): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    toneMapped: false,
    uniforms: {
      uAccent: holo.uAccent,
      uReveal: { value: 0 },
      uOrigin: { value: new THREE.Vector2(-2.5, 0.9) },
      uGrid: { value: 1 },
    },
    vertexShader: /* glsl */ `
      varying vec2 vXZ;
      void main(){ vec4 wp = modelMatrix * vec4(position, 1.0); vXZ = wp.xz; gl_Position = projectionMatrix * viewMatrix * wp; }
    `,
    fragmentShader: /* glsl */ `
      uniform float uReveal; uniform vec2 uOrigin; uniform vec3 uAccent; uniform float uGrid;
      varying vec2 vXZ;
      float gridLine(vec2 p, float s){
        vec2 q = p / s;
        vec2 g = abs(fract(q - 0.5) - 0.5) / max(fwidth(q), vec2(1e-4));
        return 1.0 - min(min(g.x, g.y), 1.0);
      }
      void main(){
        vec2 p = vXZ;
        float d = length(p - uOrigin);
        float R = uReveal * 18.0;
        float wave = smoothstep(d - 2.2, d, R);
        float ring = exp(-pow((d - R) * 1.8, 2.0)) * step(0.001, uReveal) * (1.0 - smoothstep(0.85, 1.0, uReveal));
        float l1 = gridLine(p, 0.1);
        float l5 = gridLine(p, 0.5);
        float room = step(abs(p.x), 3.052) * step(abs(p.y), 3.052);
        float edge = (1.0 - smoothstep(0.0, 0.03, abs(abs(p.x) - 3.048))) * step(abs(p.y), 3.07)
                   + (1.0 - smoothstep(0.0, 0.03, abs(abs(p.y) - 3.048))) * step(abs(p.x), 3.07);
        float fade = exp(-length(p) * 0.115);
        float a = (l1 * 0.04 * room + l5 * (0.13 + 0.17 * room) + edge * 0.65) * wave * fade * uGrid;
        a += ring * 0.4 * fade;
        vec3 col = uAccent * (1.0 + ring * 1.0 + edge * 0.5);
        gl_FragColor = vec4(col, clamp(a, 0.0, 1.0));
      }
    `,
  });
}

/** floor markings (pad rings, launch mark): emissive flat colour with a radial reveal (a very slow, very small swell at most) */
export function flatMaterial(color: THREE.ColorRepresentation | 'accent', opts: { alpha?: number; pulse?: number; boost?: number } = {}): THREE.ShaderMaterial {
  return new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    toneMapped: false,
    side: THREE.DoubleSide,
    blending: THREE.AdditiveBlending,
    uniforms: {
      uTime: holo.uTime,
      uAccent: holo.uWorld,
      uColor: { value: new THREE.Color(color === 'accent' ? '#ffffff' : color) },
      uUseAccent: { value: color === 'accent' ? 1 : 0 },
      uAlpha: { value: opts.alpha ?? 0.7 },
      uPulse: { value: opts.pulse ?? 0.25 },
      uBoost: { value: opts.boost ?? 1.4 },
      uReveal: { value: 0 },
      uOrigin: { value: new THREE.Vector2(-2.5, 0.9) },
      uPhase: { value: 2.0 },
    },
    vertexShader: /* glsl */ `
      varying vec2 vXZ;
      void main(){ vec4 wp = modelMatrix * vec4(position, 1.0); vXZ = wp.xz; gl_Position = projectionMatrix * viewMatrix * wp; }
    `,
    fragmentShader: /* glsl */ `
      uniform float uTime; uniform vec3 uColor; uniform vec3 uAccent; uniform float uUseAccent; uniform float uAlpha; uniform float uPulse; uniform float uBoost;
      uniform float uReveal; uniform vec2 uOrigin; uniform float uPhase;
      varying vec2 vXZ;
      void main(){
        float d = length(vXZ - uOrigin);
        float wave = smoothstep(d - 2.2, d, uReveal * 18.0);
        if (wave < 0.01) discard;
        vec3 base = mix(uColor, uAccent, uUseAccent);
        float p = 1.0 + 0.2 * uPulse * sin(uTime * 0.8 + uPhase);
        gl_FragColor = vec4(base * uBoost * p, uAlpha * wave);
      }
    `,
  });
}

/** the 20 x 20 ft net as a faint 1 ft mesh (the roof is left off so the top-down view stays clean) */
export function buildNet(half: number, height: number): { object: THREE.LineSegments; material: THREE.ShaderMaterial } {
  const step = 0.3048;
  const pos: number[] = [];
  const side = (ax: 'x' | 'z', fixed: number) => {
    for (let t = -half; t <= half + 1e-6; t += step) {
      if (ax === 'x') pos.push(fixed, 0, t, fixed, height, t);
      else pos.push(t, 0, fixed, t, height, fixed);
    }
    for (let y = 0; y <= height + 1e-6; y += step) {
      if (ax === 'x') pos.push(fixed, y, -half, fixed, y, half);
      else pos.push(-half, y, fixed, half, y, fixed);
    }
  };
  side('x', -half);
  side('x', half);
  side('z', -half);
  side('z', half);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  const m = new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    toneMapped: false,
    uniforms: { uAccent: holo.uWorld, uAlpha: { value: 0 }, uH: { value: height } },
    vertexShader: /* glsl */ `
      varying vec3 vW;
      void main(){ vec4 wp = modelMatrix * vec4(position, 1.0); vW = wp.xyz; gl_Position = projectionMatrix * viewMatrix * wp; }
    `,
    fragmentShader: /* glsl */ `
      uniform vec3 uAccent; uniform float uAlpha; uniform float uH;
      varying vec3 vW;
      void main(){
        float hh = vW.y / uH;
        float a = uAlpha * (0.04 + 0.08 * (1.0 - hh));
        gl_FragColor = vec4(mix(vec3(1.0, 0.92, 0.82), uAccent, 0.5), a);
      }
    `,
  });
  return { object: new THREE.LineSegments(g, m), material: m };
}
