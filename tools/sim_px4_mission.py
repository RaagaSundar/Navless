#!/usr/bin/env python3
"""Offline closed-loop run of px4_bridge_node's plain-PX4 mission (no ROS, no lab).

  python3 tools/sim_px4_mission.py [--seed N] [--no-fly-home] [--max-s 240]

Physics + noisy/drifting VIO: tests/test_ros2_gazebo_sim.QuadrotorPhysicsAndVIOSim.
PX4 here is a toy: Offboard needs >= 0.5 s of fresh setpoints, ARM arms, NAV_LAND descends at
0.5 m/s and auto-disarms 1 s after touchdown. It checks the mission logic and planner over the real
course mesh; it does NOT model cuVSLAM losing features, so a pass here is necessary, not sufficient.
Exit 0 = DONE with no wall contact.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from planner.config import PlannerConfig  # noqa: E402
from planner.fsm_router import GlobalCourseRouter  # noqa: E402
from planner.grid_map import OccupancyGrid3D  # noqa: E402
from planner.px4_bridge_node import (  # noqa: E402
    ARMING_STATE_ARMED,
    NAVIGATION_STATE_AUTO_LAND,
    NAVIGATION_STATE_OFFBOARD,
    VEHICLE_CMD_COMPONENT_ARM_DISARM,
    VEHICLE_CMD_DO_SET_MODE,
    VEHICLE_CMD_NAV_LAND,
    PX4TrajectoryTrackerCore,
)
from tests.test_ros2_gazebo_sim import QuadrotorPhysicsAndVIOSim  # noqa: E402

DISARMED, POSCTL = 1, 2
SPAWN_ENU = (-2.515, 1.114, 0.067)  # FlyX lab spawn (body origin standing on the floor)
CONTACT_M = 0.20  # body centre closer than this to raw occupied voxels counts as contact (7" props)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--no-fly-home", action="store_true")
    ap.add_argument("--max-s", type=float, default=240.0)
    default_prior = (
        ROOT / "dronehacks-course-asbuilt.obj"
        if (ROOT / "dronehacks-course-asbuilt.obj").exists()
        else ROOT / "gazebo_sim/models/dh_course/meshes/course_visual.obj"
    )
    ap.add_argument("--prior", type=Path, default=default_prior)
    ap.add_argument("--speed", type=float, default=None, help="maze/chicane cruise speed override, as px4_bridge_node --speed")
    ap.add_argument("--kp-pos", type=float, default=0.95, help="PX4 position gain MPC_XY_P (the old stand-in used 3.4)")
    ap.add_argument("--fast-mode", action="store_true", help="as px4_bridge_node --fast-mode")
    a = ap.parse_args()

    cfg = PlannerConfig.from_json(ROOT / "planner/config/course_config.json")
    cfg.enable_fly_home = not a.no_fly_home
    cfg.fast_mode = a.fast_mode
    if a.speed:
        from dataclasses import replace
        cfg.maze_profile = replace(cfg.maze_profile, v_max=a.speed)
        cfg.tunnel_profile = replace(cfg.tunnel_profile, v_max=min(a.speed, cfg.tunnel_profile.v_max))
        cfg.chicane_profile = replace(cfg.chicane_profile, v_max=a.speed)
    grid = OccupancyGrid3D(bounds_min_m=cfg.global_map_bounds_min_m, bounds_max_m=cfg.global_map_bounds_max_m,
                           resolution_m=cfg.voxel_resolution_m, inflation_radius_m=cfg.obstacle_inflation_m,
                           local_window_m=cfg.local_window_size_m)
    grid.load_prior_from_obj(a.prior)
    occ = np.argwhere(grid.raw_grid)
    occ_m = (occ + 0.5) * grid.resolution + grid.bounds_min

    router = GlobalCourseRouter(cfg, grid)
    tracker = PX4TrajectoryTrackerCore(router, launch_origin_enu=SPAWN_ENU, prearm_settle_s=2.0)
    phys = QuadrotorPhysicsAndVIOSim(SPAWN_ENU, SPAWN_ENU, seed=a.seed)
    phys.kp_pos = a.kp_pos  # PX4 MPC_XY_P: 0.95 default; the FlyX lab data fits it (0.48 m lead -> ~0.41 m/s)

    dt, t = 1.0 / 30.0, 0.0
    armed, nav, landed_since = False, POSCTL, None
    last_sp_t, stream_since, sp = -1e9, None, None
    log_key, min_clear, worst = None, 9.9, None
    while t < a.max_s:
        p_ned, v_ned, hdg = phys.observe_noisy_vio_ned(dt)
        z = float(phys.true_pos_enu[2])
        on_ground = z <= SPAWN_ENU[2] + 0.03 and float(np.linalg.norm(phys.true_vel_enu)) < 0.15
        tracker.update_local_position(t, *p_ned, *v_ned, hdg, True, True)
        tracker.update_vehicle_status(t, ARMING_STATE_ARMED if armed else DISARMED, nav)
        tracker.update_landed(t, on_ground)
        tracker.update_estimator_health(True)

        cmd = tracker.step_px4_mission(t)
        key = (cmd.state, router.stage.value)
        if key != log_key:
            log_key = key
            e, n, u = phys.true_pos_enu
            print(f"t={t:6.1f}s  {cmd.state:9s} {router.stage.value:20s} true=({e:5.2f},{n:5.2f},{u:4.2f})  {cmd.reason}")
        if cmd.setpoint is not None:
            sp = cmd.setpoint
            if t - last_sp_t > 0.5:
                stream_since = t
            last_sp_t = t
        for c, p1, _p2 in cmd.commands:  # toy PX4 command handling
            if c == VEHICLE_CMD_DO_SET_MODE and stream_since is not None and t - stream_since >= 0.5:
                nav = NAVIGATION_STATE_OFFBOARD
            elif c == VEHICLE_CMD_COMPONENT_ARM_DISARM:
                armed = p1 > 0.5
            elif c == VEHICLE_CMD_NAV_LAND and armed:
                nav = NAVIGATION_STATE_AUTO_LAND
        if nav == NAVIGATION_STATE_OFFBOARD and t - last_sp_t > 0.5:  # offboard loss failsafe
            nav = NAVIGATION_STATE_AUTO_LAND

        if not armed:
            phys.true_vel_enu[:] = 0.0
        elif nav == NAVIGATION_STATE_AUTO_LAND:  # p_ned is already PX4's own (raw) frame
            phys.step_physics(dt, t, (p_ned[0], p_ned[1], p_ned[2] + 0.5), (0.0, 0.0, 0.5), hdg, 0.6)
            if on_ground:
                landed_since = t if landed_since is None else landed_since
                if t - landed_since > 1.0:
                    armed, nav, landed_since = False, POSCTL, None
            else:
                landed_since = None
        elif nav == NAVIGATION_STATE_OFFBOARD and sp is not None:
            vel = tuple(0.0 if not math.isfinite(v) else v for v in sp.velocity_ned)
            phys.step_physics(dt, t, tracker.to_px4_frame(sp.position_ned), vel, sp.yaw_ned, 1.2)
        else:
            phys.step_physics(dt, t, tuple(p_ned), (0.0, 0.0, 0.0), hdg, 1.2)

        if phys.true_pos_enu[2] <= SPAWN_ENU[2] and phys.true_vel_enu[2] < 0.0:  # the floor stops it
            phys.true_pos_enu[2] = SPAWN_ENU[2]
            phys.true_vel_enu[:] = 0.0
        if armed and z > 0.25:
            d = float(np.min(np.linalg.norm(occ_m - phys.true_pos_enu, axis=1)))
            if d < min_clear:
                min_clear, worst = d, (round(t, 1), *np.round(phys.true_pos_enu, 2))
        if cmd.state in ("DONE", "ABORT"):
            break
        t += dt

    print(f"END state={tracker.px4_state} after {t:.1f}s  min clearance {min_clear:.3f} m at t,e,n,u={worst}")
    ok = tracker.px4_state == "DONE" and min_clear >= CONTACT_M
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
