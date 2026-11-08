"""Portable camera-only stereo VO. No ROS, simulator, filesystem, or truth inputs.

Covariance is conditional linearized image-residual covariance, not hardware
qualification: independent isotropic pixel noise, fixed calibration, per-step pixel observations are assumed. Accumulation uses n*sum(C_i), a
Cauchy upper bound allowing arbitrary cross-increment correlations. Calibration
and correspondence systematic errors are absent.
No finite covariance can describe undetected mismatches; independent monitoring
and empirical consistency qualification remain necessary.
"""
import time
import cv2
import numpy as np

def _flow(a,b,p):
    p=np.asarray(p,np.float32).reshape(-1,1,2)
    opts=dict(winSize=(21,21),maxLevel=3,criteria=(3,40,.001))
    q,s,_=cv2.calcOpticalFlowPyrLK(a,b,p,None,**opts)
    back,s2,_=cv2.calcOpticalFlowPyrLK(b,a,q,None,**opts)
    return q.reshape(-1,2),(s.ravel()>0)&(s2.ravel()>0)&(np.linalg.norm(back.reshape(-1,2)-p.reshape(-1,2),axis=1)<.5)

def _flow_initialized(a,b,p,initial):
    p=np.asarray(p,np.float32).reshape(-1,1,2);initial=np.asarray(initial,np.float32).reshape(-1,1,2)
    opts=dict(winSize=(21,21),maxLevel=3,criteria=(3,40,.001),flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    q,s,_=cv2.calcOpticalFlowPyrLK(a,b,p,initial.copy(),**opts)
    back,s2,_=cv2.calcOpticalFlowPyrLK(b,a,q,p.copy(),**opts)
    return q.reshape(-1,2),(s.ravel()>0)&(s2.ravel()>0)&(np.linalg.norm(back.reshape(-1,2)-p.reshape(-1,2),axis=1)<.5)

def _projection_jacobian(X,K):
    x,y,z=X
    return np.array([[K[0,0]/z,0,-K[0,0]*x/z**2],[0,K[1,1]/z,-K[1,1]*y/z**2]])

def _motion_covariance(X,p,pr,q,qr,K,baseline,rv,tv):
    """Schur-eliminate uncertain stereo landmarks from four-view pixel system.

    Parameters are Rodrigues rotation then translation of previous→current camera.
    Uses actual four-view residual SSE / remaining degrees of freedom; no chosen
    variance floor. Reject singular/zero-residual systems instead of inventing noise.
    """
    R=cv2.Rodrigues(rv)[0];n=len(X);offset=np.array([baseline,0.,0.])
    cur=(R@X.T).T+tv.ravel();points=[X,X-offset,cur,cur-offset]
    A=np.zeros((n,8,6));B=np.zeros((n,8,3));res=np.zeros((n,8))
    for j,(pts,obs) in enumerate(zip(points,[p,pr,q,qr])):
        proj=(K@pts.T).T;proj=proj[:,:2]/proj[:,2:];res[:,2*j:2*j+2]=proj-obs
        J=np.zeros((n,2,3));z=pts[:,2]
        J[:,0,0]=K[0,0]/z;J[:,1,1]=K[1,1]/z
        J[:,0,2]=-K[0,0]*pts[:,0]/z**2;J[:,1,2]=-K[1,1]*pts[:,1]/z**2
        B[:,2*j:2*j+2]=J if j<2 else J@R
        if j>=2:
            _,jac=cv2.projectPoints(X,rv,tv.ravel()-(offset if j==3 else 0),K,None)
            A[:,2*j:2*j+2]=jac[:,:6].reshape(n,2,6)
    At=np.swapaxes(A,1,2);Bt=np.swapaxes(B,1,2)
    AB=At@B;BB=Bt@B
    H=np.sum(At@A-AB@np.linalg.pinv(BB)@np.swapaxes(AB,1,2),axis=0)
    sse=float(np.sum(res**2))
    H=(H+H.T)/2;eig=np.linalg.eigvalsh(H)
    if eig[0]<=0 or eig[-1]/eig[0]>1e10 or sse<=0:
        raise ValueError('unobservable covariance information')
    variance=sse/(5*len(X)-6)
    return np.linalg.inv(H)*variance,dict(pixel_variance=variance,information_condition=float(eig[-1]/eig[0]),covariance_points=len(X))

class StereoVO:
    """Strict sequential tracker. Any failed step latches LOST until reset().

    reset() destroys the coordinate origin. The caller must explicitly coordinate
    that discontinuity with downstream consumers; automatic relocalization absent.
    T_origin_camera maps current optical coordinates into first optical coordinates.
    covariance_6x6 is [position, small-angle rotation] in the origin optical basis.
    """
    def __init__(self,K,baseline_m,max_dt_s=.25,recovery_horizon_s=0.,feature_quality=.0001,stereo_matcher=None,max_disparity=100.,right_check_inliers=False,prediction_initialized_right=False,temporal_matcher=None,feature_limit=3000):
        self.K=np.asarray(K,float).reshape(3,3).copy();self.baseline=float(baseline_m);self.max_dt=float(max_dt_s);self.recovery_horizon=float(recovery_horizon_s);self.feature_quality=float(feature_quality);self.stereo_matcher=stereo_matcher or _flow;self.max_disparity=float(max_disparity);self.right_check_inliers=bool(right_check_inliers);self.prediction_initialized_right=bool(prediction_initialized_right);self.temporal_matcher=temporal_matcher or _flow;self.feature_limit=int(feature_limit)
        if not 20<=self.feature_limit<=3000 or not np.isfinite(self.max_disparity) or self.max_disparity<=1.5 or not np.isfinite(self.K).all() or not np.isfinite([self.baseline,self.max_dt,self.recovery_horizon,self.feature_quality]).all() or not 0<self.feature_quality<=1 or self.recovery_horizon<0 or min(self.K[0,0],self.K[1,1],self.baseline,self.max_dt)<=0:raise ValueError('invalid calibration/timing')
        self.reset()
    def reset(self):
        self.prev=None;self.stamp=None;self.T=np.eye(4);self.cov=np.zeros((6,6));self.lost=False;self.steps=0
    def process(self,stamp_s,left_gray,right_gray):
        started=time.perf_counter()
        try:return self._process(stamp_s,left_gray,right_gray)
        except (cv2.error,np.linalg.LinAlgError,ValueError) as exc:
            self.lost=True
            return dict(valid=False,reason='computation_rejected:'+type(exc).__name__,stamp_s=stamp_s,T_origin_camera=None,covariance_6x6=None,counters={},elapsed_s=time.perf_counter()-started)
    def _process(self,stamp_s,left_gray,right_gray):
        started=time.perf_counter();stamp=float(stamp_s)
        result=dict(valid=False,reason='uninitialized',stamp_s=stamp,T_origin_camera=None,covariance_6x6=None,counters={})
        def finish(reason):
            result['reason']=reason;result['elapsed_s']=time.perf_counter()-started;return result
        recovering=self.lost and self.prev is not None and self.recovery_horizon>0 and 0<stamp-self.stamp<=self.recovery_horizon
        if self.lost and not recovering:return finish('lost_reset_required')
        if not isinstance(left_gray,np.ndarray) or not isinstance(right_gray,np.ndarray) or not np.isfinite(stamp) or left_gray.dtype!=np.uint8 or right_gray.dtype!=np.uint8 or left_gray.ndim!=2 or left_gray.shape!=right_gray.shape:
            self.lost=True;return finish('invalid_input')
        if self.prev is None:
            self.prev=(left_gray.copy(),right_gray.copy());self.stamp=stamp;return finish('initialized_no_motion')
        if stamp<=self.stamp or (stamp-self.stamp>self.max_dt and not recovering):
            self.lost=True;return finish('timestamp_gap_or_regression')
        a,b=self.prev
        if a.shape!=left_gray.shape:self.lost=True;return finish('image_size_changed')
        p=cv2.goodFeaturesToTrack(a,3000,self.feature_quality,5)
        if p is None:self.lost=True;return finish('no_features')
        p=p.reshape(-1,2)
        if len(p)>self.feature_limit:p=p[np.linspace(0,len(p)-1,self.feature_limit).astype(int)]
        pr,ok=self.stereo_matcher(a,b,p);disp=p[:,0]-pr[:,0]
        ok &= (np.abs(p[:,1]-pr[:,1])<.4)&(disp>1.5)&(disp<self.max_disparity)
        q,tok=self.temporal_matcher(a,left_gray,p);ok &=tok
        p=p[ok];pr=pr[ok];q=q[ok];depth=self.K[0,0]*self.baseline/disp[ok]
        X=np.column_stack(((p[:,0]-self.K[0,2])*depth/self.K[0,0],(p[:,1]-self.K[1,2])*depth/self.K[1,1],depth))
        result['counters']['stereo_temporal_points']=len(X)
        if len(X)<20:self.lost=True;return finish('insufficient_stereo_tracks')
        passed,rv,tv,ins=cv2.solvePnPRansac(X,q,self.K,None,iterationsCount=300,reprojectionError=1.,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
        if not passed or ins is None or len(ins)<20:self.lost=True;return finish('pnp_rejected')
        idx=ins.ravel();rv,tv=cv2.solvePnPRefineLM(X[idx],q[idx],self.K,None,rv,tv)
        R=cv2.Rodrigues(rv)[0]
        Xt=(R@X.T).T+tv.ravel();Xt[:,0]-=self.baseline
        proj=(self.K@Xt.T).T;proj=proj[:,:2]/proj[:,2:];qr,rok=_flow_initialized(left_gray,right_gray,q,proj) if self.prediction_initialized_right else self.stereo_matcher(left_gray,right_gray,q)
        err=np.linalg.norm(proj-qr,axis=1);cheirality=(Xt[:,2]>0)&np.isfinite(proj).all(axis=1);good=rok&cheirality&(err<1.)
        overall_med=float(np.median(err[rok])) if rok.any() else float('inf')
        inlier_mask=np.zeros(len(X),bool);inlier_mask[idx]=True
        checked=rok & inlier_mask if self.right_check_inliers else rok
        if self.right_check_inliers:good &= inlier_mask
        med=float(np.median(err[checked])) if checked.any() else float('inf')
        result['counters'].update(inliers=len(ins),right_verified=int(good.sum()),right_error_median=med,right_error_overall_median=overall_med)
        if good.sum()<15 or med>=2:self.lost=True;return finish('independent_right_reprojection_rejected')
        # Covariance uses only points passing BOTH PnP and independent right checks.
        use=np.intersect1d(idx,np.flatnonzero(good))
        if len(use)<8:self.lost=True;return finish('insufficient_covariance_points')
        try:cov,info=_motion_covariance(X[use],p[use],pr[use],q[use],qr[use],self.K,self.baseline,rv,tv)
        except ValueError as exc:self.lost=True;return finish(str(exc))
        result['counters'].update(info)
        step=np.eye(4);step[:3,:3]=R;step[:3,3]=tv.ravel();newT=self.T@np.linalg.inv(step)
        # Numerical derivative maps incremental Rodrigues/translation uncertainty
        # into origin-frame position and rotation tangent; covariance accumulates
        # with the independent-increment sum retained internally; output n*sum(C_i)
        # bounds arbitrary increment correlations by Cauchy-Schwarz.
        G=np.zeros((6,6));params=np.r_[rv.ravel(),tv.ravel()]
        for j in range(6):
            pert=params.copy();pert[j]+=1e-6;S=np.eye(4);S[:3,:3]=cv2.Rodrigues(pert[:3])[0];S[:3,3]=pert[3:]
            P=self.T@np.linalg.inv(S);G[:3,j]=(P[:3,3]-newT[:3,3])/1e-6
            dR=P[:3,:3]@newT[:3,:3].T;G[3:,j]=np.array([dR[2,1]-dR[1,2],dR[0,2]-dR[2,0],dR[1,0]-dR[0,1]])/(2e-6)
        self.cov+=G@cov@G.T;self.cov=(self.cov+self.cov.T)/2
        self.steps+=1;self.lost=False
        self.T=newT;self.prev=(left_gray.copy(),right_gray.copy());self.stamp=stamp
        result.update(valid=True,T_origin_camera=self.T.copy(),covariance_6x6=self.steps*self.cov.copy(),covariance_independent_6x6=self.cov.copy())
        return finish('relocalized_same_origin' if recovering else 'tracking')
