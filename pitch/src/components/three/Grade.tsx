'use client';

import { wrapEffect } from '@react-three/postprocessing';
import { Effect } from 'postprocessing';
import { Color, Uniform } from 'three';

/**
 * Golden-hour grade: a gentle split tone applied in linear light, before tone mapping.
 * Shadows lean cool (dusty blue), highlights lean warm (amber), mid-tones are left alone. It is the same trick a low sun plays
 * on a real scene: warm where the light lands, cool where it does not. Static, so it can never flicker.
 */
const FRAG = /* glsl */ `
  uniform vec3 uShadow;
  uniform vec3 uHighlight;
  uniform float uAmount;
  void mainImage(const in vec4 inputColor, const in vec2 uv, out vec4 outputColor) {
    float l = dot(inputColor.rgb, vec3(0.2126, 0.7152, 0.0722));
    vec3 tint = mix(uShadow, uHighlight, smoothstep(0.02, 0.6, l));
    outputColor = vec4(mix(inputColor.rgb, inputColor.rgb * tint, uAmount), inputColor.a);
  }
`;

class GradeEffectImpl extends Effect {
  constructor({ shadow = '#c9d6ee', highlight = '#fff0dc', amount = 0.55 }: { shadow?: string; highlight?: string; amount?: number } = {}) {
    super('GradeEffect', FRAG, {
      uniforms: new Map<string, Uniform>([
        ['uShadow', new Uniform(new Color(shadow))],
        ['uHighlight', new Uniform(new Color(highlight))],
        ['uAmount', new Uniform(amount)],
      ]),
    });
  }
}

export const Grade = wrapEffect(GradeEffectImpl);
