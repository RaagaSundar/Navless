'use client';

import { useTransform, type MotionValue } from 'framer-motion';
import { progress } from '@/lib/motion';
import { smoothstep } from '@/lib/math';

/** 0 -> 1 -> 0 window over [a, b] of the scroll track. No React renders: it is a MotionValue. */
export function useWindow(a: number, b: number, fade = 0.018): MotionValue<number> {
  return useTransform(progress, (p) => smoothstep(a, a + fade, p) * (1 - smoothstep(b - fade, b, p)));
}

/** 0 -> 1 ramp over [a, b] */
export function useRamp(a: number, b: number): MotionValue<number> {
  return useTransform(progress, (p) => smoothstep(a, b, p));
}

/** 1 -> 0 ramp over [a, b] */
export function useFadeOut(a: number, b: number): MotionValue<number> {
  return useTransform(progress, (p) => 1 - smoothstep(a, b, p));
}
