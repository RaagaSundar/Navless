"""Coordinate transformations between ROS 2 ENU/FLU and PX4 NED/FRD frames.

In the DroneHacks Lab (`AI_CONTEXT.md`):
- PX4 `VehicleLocalPosition` and `TrajectorySetpoint` use NED:
  x = North (or local forward reference), y = East (or right), z = Down (metres).
  Climbing is negative z in NED.
- Standard ROS 2 / EGO-Planner / 3D layout models use ENU / FLU:
  +z = Up (altitude above floor in metres).
- This module supports both:
  1) Standard FLU (x=Forward/North, y=Left/West, z=Up) <-> FRD/NED (x=Forward/North, y=-Left=Right/East, z=-Up=Down),
     matching Step 4 of the architecture spec: `sp.position = {pos.x, -pos.y, -pos.z}`, `yaw_ned = -yaw_flu`.
  2) Geographic ENU (x=East, y=North, z=Up) <-> Geographic NED (x=North, y=East, z=-Up), `yaw_ned = -yaw_enu + pi/2`.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


def wrap_pi(angle_rad: float) -> float:
    """Wrap angle in radians to [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def clamp_altitude_enu(
    pos_enu: np.ndarray | Tuple[float, float, float],
    z_min: float = 0.50,
    z_max: float = 1.10,
) -> np.ndarray:
    """Enforce vertical Z-ceiling clamp in ENU/FLU (+Z Up) meters."""
    if z_min > z_max:
        raise ValueError(f"z_min ({z_min}) cannot exceed z_max ({z_max})")
    out = np.asarray(pos_enu, dtype=np.float64).copy()
    out[2] = float(np.clip(out[2], z_min, z_max))
    return out


def enu_to_ned_pos(
    pos: np.ndarray | Tuple[float, float, float],
    *,
    convention: str = "enu_to_ned",
    origin_enu: Optional[Tuple[float, float, float]] = None,
) -> Tuple[float, float, float]:
    """Convert [x, y, z] from ROS ENU (+X East, +Y North, +Z Up) to PX4 NED (+X North, +Y East, +Z Down).

    Args:
        pos: 3D position [x, y, z] with z = altitude up (m).
        convention:
            - "enu_to_ned" (default, standard compass East-North-Up to North-East-Down):
              x_ned = y_north, y_ned = x_east, z_ned = -z_up
            - "flu_to_frd" (body Forward-Left-Up to Forward-Right-Down):
              x_ned = x, y_ned = -y, z_ned = -z
        origin_enu: Optional launch pad origin (x0, y0, z0) in course ENU frame so that
            PX4 local NED (0, 0, 0) is anchored at the launch mark where VIO initializes.
    """
    x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
    if origin_enu is not None:
        x -= float(origin_enu[0])
        y -= float(origin_enu[1])
        z -= float(origin_enu[2])
    if convention == "enu_to_ned":
        return (y, x, -z)
    if convention == "flu_to_frd":
        return (x, -y, -z)
    raise ValueError(f"Unsupported frame convention: {convention}")


def ned_to_enu_pos(
    pos_ned: np.ndarray | Tuple[float, float, float],
    *,
    convention: str = "enu_to_ned",
    origin_enu: Optional[Tuple[float, float, float]] = None,
) -> np.ndarray:
    """Convert PX4 NED [x_north, y_east, z_down] back to course ENU frame (+X East, +Y North, +Z Up)."""
    x_n, y_e, z_d = float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])
    if convention == "enu_to_ned":
        enu = np.array([y_e, x_n, -z_d], dtype=np.float64)
    elif convention == "flu_to_frd":
        enu = np.array([x_n, -y_e, -z_d], dtype=np.float64)
    else:
        raise ValueError(f"Unsupported frame convention: {convention}")
    if origin_enu is not None:
        enu[0] += float(origin_enu[0])
        enu[1] += float(origin_enu[1])
        enu[2] += float(origin_enu[2])
    return enu


def enu_to_ned_vel(
    vel: np.ndarray | Tuple[float, float, float],
    *,
    convention: str = "enu_to_ned",
) -> Tuple[float, float, float]:
    """Convert velocity vector [vx, vy, vz] from ENU (+Z Up) to PX4 NED."""
    return enu_to_ned_pos(vel, convention=convention, origin_enu=None)


def ned_to_enu_vel(
    vel_ned: np.ndarray | Tuple[float, float, float],
    *,
    convention: str = "enu_to_ned",
) -> np.ndarray:
    """Convert velocity vector from PX4 NED to ENU (+Z Up)."""
    return ned_to_enu_pos(vel_ned, convention=convention, origin_enu=None)


def enu_yaw_to_ned_yaw(
    yaw_rad: float,
    *,
    convention: str = "enu_to_ned",
) -> float:
    """Convert planner ENU yaw (0=East, +pi/2=North) to PX4 NED yaw (0=North, +pi/2=East) in [-pi, pi)."""
    if convention == "enu_to_ned":
        return wrap_pi(-float(yaw_rad) + 0.5 * math.pi)
    if convention == "flu_to_frd":
        return wrap_pi(-float(yaw_rad))
    raise ValueError(f"Unsupported frame convention: {convention}")
