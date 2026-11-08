'use client';

import { useFrame } from '@react-three/fiber';
import { view } from '@/lib/choreography';
import { enterOverride, stopDemo } from '@/lib/deckControls';
import { DEMO_DURATION, OVERRIDE_AT, RELEASE_AT, cameraAt, demo, demoTime, pOfT, resetDemoFlags, revealOfT } from '@/lib/director';
import { clamp } from '@/lib/math';
import { progress } from '@/lib/motion';
import { log, useDeck } from '@/lib/store';

/**
 * AutoDirector: takes the R3F camera for the 2-minute demo.
 *
 * It runs at priority -4, i.e. BEFORE CameraRig (-3), so each frame it
 *   1. advances the demo clock,
 *   2. writes the scroll `progress` for that time (the HTML text windows, drone anchors, maze reveal and ghost flight all
 *      read progress, so they stay locked to the same clock as the camera),
 *   3. samples the CatmullRomCurve3 camera + look-at paths (see lib/director.ts) into `view.camPos / camLook / camFov`,
 *   4. fires the scripted OVERRIDE beats: enter at 1:40, two blocker drops, hand the grid to the judge at 1:58.
 * CameraRig then eases the real camera onto that pose. Esc aborts, Space pauses.
 */
export default function AutoDirector() {
  useFrame((_, dtRaw) => {
    if (!demo.active) return;
    const dt = Math.min(dtRaw, 0.1);

    if (demo.seek !== null) {
      demo.t = clamp(demo.seek, 0, DEMO_DURATION);
      demo.seek = null;
      view.snap = 3;
      if (demo.t < OVERRIDE_AT) resetDemoFlags();
    } else if (!demo.paused) {
      demo.t = Math.min(DEMO_DURATION, demo.t + dt);
    }
    const t = demo.t;
    demoTime.set(t);

    // timeline -> everything that follows scroll
    progress.set(pOfT(t));
    view.directed = true;
    view.revealOverride = revealOfT(t);
    // camera on rails
    view.camFov = cameraAt(t, view.camPos, view.camLook);

    if (demo.paused) return;
    // scripted override beats
    const f = demo.flags;
    const st = useDeck.getState();
    if (t >= OVERRIDE_AT && !f.override) {
      f.override = true;
      enterOverride();
    }
    if (t >= OVERRIDE_AT + 4 && !f.drop1) {
      f.drop1 = true;
      log('EVENT', 'AUTO: injecting obstacle #1 on the active path', 'warn');
      st.dropOnPath();
    }
    if (t >= OVERRIDE_AT + 12 && !f.drop2) {
      f.drop2 = true;
      log('EVENT', 'AUTO: injecting obstacle #2', 'warn');
      st.dropOnPath();
    }
    if (t >= RELEASE_AT && !f.release) {
      f.release = true;
      stopDemo();
      log('SYS', 'CONTROL RELEASED. inject your own obstacle: click the button, then the grid.', 'ok');
      useDeck.getState().setArmed(true);
    }
  }, -4);

  return null;
}
