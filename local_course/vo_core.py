"""Calibration and camera-to-body math for measured OpenCV stereo VO.

No truth, PX4, ROS clock or command dependencies. Covariance is direct camera
position plus fixed-axis orientation error in the initial optical world.
"""
import math
import numpy as np


def camera_calibration(left,right):
    """Validate rectified pinhole CameraInfo pair and derive measured baseline."""
    if (left.width,left.height)!=(right.width,right.height) or left.width<=0 or left.height<=0:
        raise ValueError('camera_dimensions')
    if any(abs(float(v))>1e-10 for m in (left,right) for v in m.d):
        raise ValueError('requires_rectified_zero_distortion_images')
    matrices=[np.asarray(m.p,dtype=float).reshape(3,4) for m in (left,right)]
    a,b=matrices
    if not all(np.isfinite(x).all() for x in matrices) or a[0,0]<=0 or a[1,1]<=0:
        raise ValueError('camera_projection')
    if abs(a[0,1])+abs(a[1,0])>1e-10:
        raise ValueError('unsupported_skew_intrinsics')
    if not np.allclose(a[:,:3],b[:,:3],rtol=1e-6,atol=1e-6):
        raise ValueError('unrectified_intrinsics')
    if not np.allclose(a[2],[0,0,1,0]) or not np.allclose(b[2],[0,0,1,0]):
        raise ValueError('camera_projection_bottom_row')
    if abs(a[1,3])+abs(b[1,3])>1e-8:
        raise ValueError('nonhorizontal_stereo')
    baseline=float(a[0,3]/a[0,0]-b[0,3]/b[0,0])
    if not .001<baseline<1.0:
        raise ValueError('stereo_baseline')
    return a[:,:3].copy(),baseline


def transform_matrix(translation,quaternion_xyzw):
    x,y,z,w=map(float,quaternion_xyzw)
    if not np.isfinite([x,y,z,w,*translation]).all() or abs(x*x+y*y+z*z+w*w-1)>1e-4:
        raise ValueError('camera_extrinsic')
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    out=np.eye(4);out[:3,:3]=r;out[:3,3]=translation
    return out


def skew(v):
    x,y,z=v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def camera_to_body(pose,covariance,body_camera):
    """Camera-origin pose to initial-body odom, with lever-arm uncertainty."""
    pose=np.asarray(pose,float);cov=np.asarray(covariance,float)
    if pose.shape!=(4,4) or cov.shape!=(6,6) or not np.isfinite(pose).all() or not np.isfinite(cov).all():
        raise ValueError('pose_covariance_shape_or_finiteness')
    if not np.allclose(pose[3],[0,0,0,1]) or not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-5) or abs(np.linalg.det(pose[:3,:3])-1)>1e-5:
        raise ValueError('pose_rotation')
    if not np.allclose(cov,cov.T,atol=1e-9) or np.linalg.eigvalsh(cov).min()<-1e-10 or np.any(np.diag(cov)<=0):
        raise ValueError('invalid_covariance')
    inverse=np.linalg.inv(body_camera)
    out=body_camera@pose@inverse
    r=body_camera[:3,:3]
    lever=pose[:3,:3]@inverse[:3,3]
    jac=np.zeros((6,6));jac[:3,:3]=r;jac[:3,3:]=-r@skew(lever);jac[3:,3:]=r
    c=jac@cov@jac.T
    return out,c


def quaternion_xyzw(rotation):
    # Stable standard matrix-to-quaternion conversion including 180deg cases.
    r=np.asarray(rotation,float)
    q=np.zeros(4)
    tr=float(np.trace(r))
    if tr>0:
        s=math.sqrt(tr+1)*2
        q[:]=[(r[2,1]-r[1,2])/s,(r[0,2]-r[2,0])/s,(r[1,0]-r[0,1])/s,.25*s]
    else:
        i=int(np.argmax(np.diag(r)));j=(i+1)%3;k=(i+2)%3
        s=math.sqrt(1+r[i,i]-r[j,j]-r[k,k])*2
        q[i]=.25*s;q[j]=(r[j,i]+r[i,j])/s;q[k]=(r[k,i]+r[i,k])/s;q[3]=(r[k,j]-r[j,k])/s
    return q/np.linalg.norm(q)


class StartupReadiness:
    """Debounce clock/stereo startup only, before any VO origin exists.

    This is timestamp health, not a physical stationarity measurement. The team
    must still establish the reviewed stationary fixture. Once ready, callers
    must preserve the estimator origin through failure; never restart this gate
    to silently rebase an aircraft in motion.
    """
    def __init__(self, min_pairs=5, min_span_s=.4):
        self.min_pairs=min_pairs;self.min_span=min_span_s
        self.count=0;self.first=None;self.previous=None

    def observe(self, stamp_s, now_s):
        if not np.isfinite([stamp_s,now_s]).all() or stamp_s<=0 or not -.05<=now_s-stamp_s<=.25:
            self.count=0;self.first=self.previous=None
            return False
        if self.previous is not None:
            stamp_dt=stamp_s-self.previous[0];clock_dt=now_s-self.previous[1]
            if not 0<stamp_dt<=.25 or not 0<clock_dt<=.5:
                self.count=0;self.first=None
        if self.first is None:self.first=(stamp_s,now_s)
        self.previous=(stamp_s,now_s);self.count+=1
        return (self.count>=self.min_pairs and stamp_s-self.first[0]>=self.min_span-1e-9
                and now_s-self.first[1]>=self.min_span-1e-9)
