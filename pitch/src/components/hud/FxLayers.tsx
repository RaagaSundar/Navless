'use client';

/**
 * Full-screen finishing layer: a soft vignette, and that is all. (There used to be animated film grain, scanlines and a red
 * edge glow in override: all gone, because anything that moves across the whole frame reads as flicker.)
 */
export default function FxLayers() {
  return <div className="fx-vignette pointer-events-none fixed inset-0 z-20" />;
}
