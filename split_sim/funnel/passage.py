"""Metric estimates and horizontal setpoints in the launch odometry frame."""
import numpy as np


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    if not np.isclose(x*x+y*y+z*z+w*w, 1., atol=.005):
        raise ValueError('invalid orientation')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def midpoint(aim, projection, optical_to_body, body_to_launch):
    # Known 1 m aperture viewed approximately front-on. Optical z is depth.
    fx, cx = projection[0], projection[2]
    span = aim['right_px'] - aim['left_px'] + 1
    z = fx / span
    if not .55 < z < 6 or abs(aim['bearing_right_deg']) > 15:
        return None
    optical = np.array([(aim['center_px']-cx)*z/fx, 0., z])
    t = optical_to_body.translation
    body = rotation(optical_to_body.rotation) @ optical + [t.x,t.y,t.z]
    p = body_to_launch.position
    point = rotation(body_to_launch.orientation) @ body + [p.x,p.y,p.z]
    return point[:2] if np.all(np.isfinite(point)) else None


def advance(previous, position, target, dt):
    # Limit both setpoint travel per tick and how far it can lead the quad.
    delta = target - previous
    candidate = previous + delta * min(1., .25*dt/max(1e-9, np.linalg.norm(delta)))
    lead = candidate - position
    return position + lead * min(1., .16/max(1e-9, np.linalg.norm(lead)))
