"""Portable advisory visual reliability probe. Sensors/calibration only; no truth/ROS.

Not a navigation estimator or a flight-qualified future-view guarantee. Qualified
means a bounded set of current rigid, nonperiodic four-view correspondences passed
these checks. Dominant static scene and calibrated stereo/IMU remain assumptions.
"""
from dataclasses import dataclass,asdict
import hashlib,json,math,time
import cv2
import numpy as np

@dataclass(frozen=True)
class QualityConfig:
    grid_cols:int=8
    grid_rows:int=5
    proposals_per_cell:int=5
    patch_radius:int=10
    self_search_radius:int=32
    self_exclusion_radius:int=8
    repeated_patch_ncc:float=.92
    patch_rms_min:float=2.
    min_tracks:int=30
    min_cells:int=6
    max_rotation_error_deg:float=1.5
    max_imu_gap_s:float=.012
    max_frame_gap_s:float=.08


def flow(a,b,p,initial=None):
    p=np.asarray(p,np.float32).reshape(-1,1,2)
    options=dict(winSize=(21,21),maxLevel=3,criteria=(3,40,.001))
    if initial is None:q,s,_=cv2.calcOpticalFlowPyrLK(a,b,p,None,**options)
    else:q,s,_=cv2.calcOpticalFlowPyrLK(a,b,p,np.asarray(initial,np.float32).reshape(-1,1,2).copy(),flags=cv2.OPTFLOW_USE_INITIAL_FLOW,**options)
    back,s2,_=cv2.calcOpticalFlowPyrLK(b,a,q,None,**options)
    return q.reshape(-1,2),(s[:,0]>0)&(s2[:,0]>0)&(np.linalg.norm(back.reshape(-1,2)-p.reshape(-1,2),axis=1)<.5)


def patch_quality(image,point,c):
    h,w=image.shape;x,y=np.rint(point).astype(int);r=c.patch_radius
    if x-r<0 or y-r<0 or x+r>=w or y+r>=h:return 'unknown',None
    patch=image[y-r:y+r+1,x-r:x+r+1]
    if float(patch.std())<c.patch_rms_min:return 'low_contrast',None
    radius=c.self_search_radius;x0=max(r,x-radius);x1=min(w-r-1,x+radius);y0=max(r,y-radius);y1=min(h-r-1,y+radius)
    roi=image[y0-r:y1+r+1,x0-r:x1+r+1]
    cc=cv2.matchTemplate(roi,patch,cv2.TM_CCOEFF_NORMED)
    peaks=cc>=cv2.dilate(cc,np.ones((3,3),np.uint8))-1e-6
    yy,xx=np.indices(cc.shape);peaks&=((xx-(x-x0))**2+(yy-(y-y0))**2)>c.self_exclusion_radius**2
    peaks[:2]=False;peaks[-2:]=False;peaks[:,:2]=False;peaks[:,-2:]=False
    alternate=float(cc[peaks].max()) if peaks.any() else -1.
    return ('periodic_or_ambiguous' if alternate>=c.repeated_patch_ncc else 'unique'),alternate


def imu_rotation(samples,start,end,body_from_camera,max_gap):
    samples=np.asarray(samples,float)
    if samples.ndim!=2 or samples.shape[1]!=4 or len(samples)<2 or not np.isfinite(samples).all():return None,None
    samples=samples[np.argsort(samples[:,0])];t=samples[:,0]
    if t[0]>start or t[-1]<end or np.any(np.diff(t)<=0):return None,None
    inside=t[(t>start)&(t<end)];times=np.r_[start,inside,end]
    gap=float(np.diff(times).max())
    if gap>max_gap:return None,gap
    gyro=np.column_stack([np.interp(times,t,samples[:,j]) for j in [1,2,3]])
    orientation=np.eye(3)
    for i,dt in enumerate(np.diff(times)):
        orientation=orientation@cv2.Rodrigues((gyro[i]+gyro[i+1])*.5*dt)[0]
    r=body_from_camera[:3,:3]
    return r.T@orientation.T@r,gap


class VisualQuality:
    def __init__(self,K,baseline_m,body_from_camera,config=QualityConfig()):
        self.K=np.asarray(K,float).reshape(3,3);self.baseline=float(baseline_m);self.extrinsic=np.asarray(body_from_camera,float).reshape(4,4);self.config=config
        if (not np.isfinite(self.K).all() or not np.isfinite(self.extrinsic).all() or not .001<self.baseline<1 or min(self.K[0,0],self.K[1,1])<=0 or not np.allclose(self.K[2],[0,0,1]) or not np.allclose(self.extrinsic[3],[0,0,0,1]) or not np.allclose(self.extrinsic[:3,:3].T@self.extrinsic[:3,:3],np.eye(3),atol=1e-5) or abs(np.linalg.det(self.extrinsic[:3,:3])-1)>1e-5):raise ValueError('invalid calibration')
    def evaluate(self,previous_stamp_s,stamp_s,previous_left,previous_right,left,right,gyro_samples):
        started=time.perf_counter()
        try:
            answer=self._evaluate(previous_stamp_s,stamp_s,previous_left,previous_right,left,right,gyro_samples)
            digest=hashlib.sha256(np.asarray([previous_stamp_s,stamp_s],dtype='<f8').tobytes())
            for image in [previous_left,previous_right,left,right]:
                if isinstance(image,np.ndarray):digest.update(image.tobytes())
            digest.update(np.asarray(gyro_samples,dtype='<f8').tobytes());digest.update(self.K.astype('<f8').tobytes());digest.update(self.extrinsic.astype('<f8').tobytes())
            answer['evidence_id']='stereo-imu-quality:'+digest.hexdigest()
            answer['scope']='two-frame current observed support only; future view unqualified'
            if isinstance(left,np.ndarray) and left.ndim==2:answer['calibration']['image_size']=[left.shape[1],left.shape[0]]
            answer['measured']['periodic_proposal_fraction']=answer['measured']['periodic_rejected']/max(1,answer['measured']['proposed_tracks'])
            return answer
        except (cv2.error,np.linalg.LinAlgError,ValueError) as exc:
            return {'stamp_s':float(stamp_s) if np.isfinite(stamp_s) else None,'previous_stamp_s':float(previous_stamp_s) if np.isfinite(previous_stamp_s) else None,'state':'unknown','reasons':['computation_rejected:'+type(exc).__name__],'measured':{'proposed_tracks':0,'periodic_rejected':0,'low_contrast_rejected':0,'rigid_nonperiodic_tracks':0,'occupied_grid_cells':0,'total_grid_cells':self.config.grid_cols*self.config.grid_rows,'pnp_inliers':0,'four_view_verified':0,'gyro_rotation_error_deg':None,'imu_max_gap_s':None,'processing_ms':(time.perf_counter()-started)*1000},'landmarks_current_camera':[],'calibration':{'K':self.K.tolist(),'baseline_m':self.baseline,'body_from_left_optical':self.extrinsic.tolist()},'config':asdict(self.config)}

    def _evaluate(self,previous_stamp_s,stamp_s,previous_left,previous_right,left,right,gyro_samples):
        started=time.perf_counter();c=self.config
        result={'stamp_s':float(stamp_s),'previous_stamp_s':float(previous_stamp_s),'state':'unknown','reasons':[],'measured':{'proposed_tracks':0,'periodic_rejected':0,'low_contrast_rejected':0,'rigid_nonperiodic_tracks':0,'occupied_grid_cells':0,'total_grid_cells':c.grid_cols*c.grid_rows,'pnp_inliers':0,'four_view_verified':0,'gyro_rotation_error_deg':None,'imu_max_gap_s':None},'landmarks_current_camera':[],'calibration':{'K':self.K.tolist(),'baseline_m':self.baseline,'body_from_left_optical':self.extrinsic.tolist()},'assumptions':['dominant static rigid scene is not independently proven','calibrated rectified stereo and synchronized calibrated body IMU'],'config':asdict(c)}
        m=result['measured']
        def finish(reason,state='unknown'):
            result['state']=state;result['reasons'].append(reason);m['processing_ms']=(time.perf_counter()-started)*1000;return result
        images=[previous_left,previous_right,left,right]
        if any(not isinstance(x,np.ndarray) or x.dtype!=np.uint8 or x.ndim!=2 or x.shape!=left.shape for x in images):return finish('invalid_stereo_images')
        dt=stamp_s-previous_stamp_s
        if not 0<dt<=c.max_frame_gap_s:return finish('frame_gap_or_regression')
        expected,maxgap=imu_rotation(gyro_samples,previous_stamp_s,stamp_s,self.extrinsic,c.max_imu_gap_s);m['imu_max_gap_s']=maxgap
        if expected is None:return finish('missing_or_gapped_imu')
        h,w=left.shape;points=[]
        for gy in range(c.grid_rows):
            for gx in range(c.grid_cols):
                x0=gx*w//c.grid_cols;x1=(gx+1)*w//c.grid_cols;y0=gy*h//c.grid_rows;y1=(gy+1)*h//c.grid_rows
                found=cv2.goodFeaturesToTrack(previous_left[y0:y1,x0:x1],c.proposals_per_cell,.001,5)
                if found is not None:points.extend(found.reshape(-1,2)+np.array([x0,y0]))
        m['proposed_tracks']=len(points);kept=[]
        for p in points:
            state,_=patch_quality(previous_left,p,c)
            if state=='unique':kept.append(p)
            elif state=='periodic_or_ambiguous':m['periodic_rejected']+=1
            elif state=='low_contrast':m['low_contrast_rejected']+=1
        if len(kept)<c.min_tracks:return finish('insufficient_unique_patch_proposals')
        p=np.asarray(kept,np.float32);pr,ok=flow(previous_left,previous_right,p);q,tok=flow(previous_left,left,p);d=p[:,0]-pr[:,0];ok&=tok&(abs(p[:,1]-pr[:,1])<.4)&(d>1.5)&(d<256)
        ids=np.flatnonzero(ok);ids=np.array([i for i in ids if patch_quality(left,q[i],c)[0]=='unique'],dtype=int)
        if len(ids)<c.min_tracks:return finish('insufficient_unique_stereo_temporal_tracks')
        p,pr,q,d=p[ids],pr[ids],q[ids],d[ids];z=self.K[0,0]*self.baseline/d;X=np.column_stack([(p[:,0]-self.K[0,2])*z/self.K[0,0],(p[:,1]-self.K[1,2])*z/self.K[1,1],z]);m['stereo_disparity_p10_p50_p90_px']=np.quantile(d,[.1,.5,.9]).tolist()
        passed,rv,tv,ins=cv2.solvePnPRansac(X,q,self.K,None,iterationsCount=200,reprojectionError=1.,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
        if not passed or ins is None or len(ins)<c.min_tracks:return finish('insufficient_rigid_model_inliers')
        idx=ins[:,0];rv,tv=cv2.solvePnPRefineLM(X[idx],q[idx],self.K,None,rv,tv);R=cv2.Rodrigues(rv)[0];m['pnp_inliers']=len(idx)
        if not np.isfinite(R).all() or not np.isfinite(tv).all():return finish('nonfinite_rigid_model')
        angle=math.degrees(math.acos(float(np.clip((np.trace(R@expected.T)-1)*.5,-1,1))));m['gyro_rotation_error_deg']=angle
        if angle>c.max_rotation_error_deg:return finish('independent_gyro_rotation_disagreement','rejected')
        cur=(R@X.T).T+tv.reshape(1,3);rpoints=cur-np.array([self.baseline,0,0]);proj=(self.K@rpoints.T).T;proj=proj[:,:2]/proj[:,2:];qr,rok=flow(left,right,q,initial=proj);error=np.linalg.norm(qr-proj,axis=1);good=np.zeros(len(X),bool);good[idx]=True;good&=rok&(error<1.)&(cur[:,2]>0)
        ids=np.flatnonzero(good);m['four_view_verified']=len(ids);m['rigid_nonperiodic_tracks']=len(ids)
        cells={(min(c.grid_cols-1,int(q[i,0]*c.grid_cols/w)),min(c.grid_rows-1,int(q[i,1]*c.grid_rows/h))) for i in ids if 0<=q[i,0]<w and 0<=q[i,1]<h};m['occupied_grid_cells']=len(cells)
        if len(ids)<c.min_tracks or len(cells)<c.min_cells:return finish('insufficient_four_view_spatial_support')
        result['landmarks_current_camera']=[{'xyz_m':cur[i].tolist(),'uv_px':q[i].tolist(),'track_index':int(i)} for i in ids]
        return finish('current_rigid_nonperiodic_four_view_support','qualified')
