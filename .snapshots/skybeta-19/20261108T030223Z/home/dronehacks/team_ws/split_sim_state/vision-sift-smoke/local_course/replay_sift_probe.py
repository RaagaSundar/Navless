"""CPU stereo SIFT-descriptor matching at proposed corners, then subpixel LK."""
import cv2,numpy as np
import hashlib
from collections import OrderedDict
_CACHE=OrderedDict()
def _descriptors(image,points):
    key=(hashlib.blake2b(image.tobytes(),digest_size=16).digest(),hashlib.blake2b(points.tobytes(),digest_size=16).digest())
    if key in _CACHE:
        _CACHE.move_to_end(key);return _CACHE[key]
    sift=cv2.SIFT_create(contrastThreshold=.001)
    _,descriptors=sift.compute(image,[cv2.KeyPoint(float(x),float(y),16) for x,y in points])
    _CACHE[key]=descriptors
    while len(_CACHE)>4:_CACHE.popitem(last=False)
    return descriptors
from replay_core import _flow_initialized

def descriptor_flow(a,b,p,stereo=True,max_points=None,use_flann=False):
    p=np.asarray(p,np.float32).reshape(-1,2);out=p.copy();ok=np.zeros(len(p),bool)
    right=cv2.goodFeaturesToTrack(b,3000,.0001,5)
    if right is None:return out,ok
    right=right.reshape(-1,2);mapping=np.arange(len(p))
    if max_points is not None:
        mapping=np.linspace(0,len(p)-1,min(max_points,len(p))).astype(int);p=p[mapping];right=right[np.linspace(0,len(right)-1,min(max_points,len(right))).astype(int)]
    da=_descriptors(a,p);db=_descriptors(b,right)
    if da is None or db is None or len(db)<2:return out,ok
    bf=cv2.FlannBasedMatcher(dict(algorithm=1,trees=4),dict(checks=64)) if use_flann else cv2.BFMatcher();fw=bf.knnMatch(da,db,k=2);bw=bf.knnMatch(db,da,k=2)
    reverse={m.queryIdx:m.trainIdx for pair in bw if len(pair)==2 for m,n in [pair] if m.distance<.75*n.distance}
    inds=[];guesses=[]
    for pair in fw:
        if len(pair)!=2:continue
        m,n=pair
        if m.distance>=.75*n.distance or reverse.get(m.trainIdx)!=m.queryIdx:continue
        u=p[m.queryIdx];v=right[m.trainIdx]
        if stereo and (abs(u[1]-v[1])>2 or not 1.5<u[0]-v[0]<256):continue
        inds.append(m.queryIdx);guesses.append(v)
    if not inds:return out,ok
    q,passed=_flow_initialized(a,b,p[inds],np.array(guesses));out[mapping[inds]]=q;ok[mapping[inds]]=passed
    return out,ok
def sift_stereo(a,b,p):return descriptor_flow(a,b,p,True)
def sift_temporal(a,b,p):return descriptor_flow(a,b,p,False)

def sift_sparse_stereo(a,b,p):return descriptor_flow(a,b,p,True,800)
def sift_sparse_temporal(a,b,p):return descriptor_flow(a,b,p,False,800)

def sift_flann_stereo(a,b,p):return descriptor_flow(a,b,p,True,1000,True)
def sift_flann_temporal(a,b,p):return descriptor_flow(a,b,p,False,1000,True)

if __name__=='__main__':
    for t in [73.,88.3]:
        l=cv2.imread(f'local_course/replay-evidence/east-left-{t:.1f}.png',0);r=cv2.imread(f'local_course/replay-evidence/east-right-{t:.1f}.png',0);p=cv2.goodFeaturesToTrack(l,3000,.0001,5).reshape(-1,2);q,ok=sift_stereo(l,r,p);d=p[:,0]-q[:,0];ok&=(np.abs(p[:,1]-q[:,1])<.4)&(d>1.5)&(d<100);print(t,ok.sum(),np.quantile(d[ok],[0,.1,.5,.9,1]) if ok.any() else None)
