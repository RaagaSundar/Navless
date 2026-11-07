"""Known launch-fixture transforms, never estimated from simulator truth.

Bearing is degrees clockwise from geographic north of initial body +x.
Startup odom is FLU (x forward, y left, z up). This assumes a level,
stationary start in the documented fixture; it is not a north sensor.
"""
import math


def launch_odom_to_ned(xyz, bearing_deg=0.0):
    if not math.isfinite(bearing_deg):
        raise ValueError('Launch bearing must be explicitly finite')
    b=math.radians(bearing_deg); c,s=math.cos(b),math.sin(b)
    x,y,z=xyz
    return [c*x+s*y, s*x-c*y, -z]


def ned_to_launch_odom(xyz, bearing_deg=0.0):
    # This orthogonal matrix is symmetric, hence its own inverse.
    return launch_odom_to_ned(xyz,bearing_deg)


def multiply(a,b):
    w,x,y,z=a; W,X,Y,Z=b
    return (w*W-x*X-y*Y-z*Z,w*X+x*W+y*Z-z*Y,
            w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W)


def launch_quaternion_to_ned(q_wxyz,bearing_deg=0.0):
    if not math.isfinite(bearing_deg):
        raise ValueError('Launch bearing must be explicitly finite')
    half=math.radians(bearing_deg)*.5
    rotation=multiply((math.cos(half),0,0,math.sin(half)),(0,1,0,0))
    return list(multiply(multiply(rotation,q_wxyz),(0,1,0,0)))


def launch_covariance_diagonals(cov,bearing_deg=0.0):
    """Rotate world fixed-axis covariance; preserve xy correlations."""
    if not math.isfinite(bearing_deg):
        raise ValueError('Launch bearing must be explicitly finite')
    b=math.radians(bearing_deg); c,s=math.cos(b),math.sin(b)
    def block(k):
        xx,yy,xy=cov[k*6+k],cov[(k+1)*6+k+1],cov[k*6+k+1]
        return [c*c*xx+s*s*yy+2*c*s*xy,
                s*s*xx+c*c*yy-2*c*s*xy,cov[(k+2)*6+k+2]]
    return block(0),block(3)
