"""Reuse measured outbound registration; never re-zero an estimator on the pad."""
import math
RESET_NAMES=('heading_reset_counter','xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter')

def retained_frame_class(base,seed,monitor):
 class RetainedFrame(base):
  def __init__(self,bearing):
   super().__init__(bearing)
   if abs(bearing-seed['course_bearing_deg'])>1e-8:raise ValueError('retained geographic frame differs')
   self.bearing=float(seed['control_bearing_deg']);self.reset=tuple(seed['reset_counters'][k] for k in RESET_NAMES[:3]);self.rechecked=False;self.recheck_first=None;self.recheck_last=None;self.recheck_count=0
  def observe(self,now,t_vo,q,p,flags,armed,landed,aligned_heading):
   if any(int(getattr(p,k))!=seed['reset_counters'][k] for k in RESET_NAMES):return 'reject','PX4 reset counter differs from pinned outbound registration'
   source_ready,reason=monitor(now)
   if not source_ready:return ('reject' if self.rechecked or armed else 'wait'),reason
   if not self.rechecked:
    if armed or not landed:return 'reject','retained-frame recheck requires landed/disarmed'
    good=(flags is not None and 0<=now-flags.timestamp/1e6<=1.5 and flags.cs_tilt_align and flags.cs_ev_pos and flags.cs_ev_yaw and not flags.cs_ev_yaw_fault and not flags.cs_inertial_dead_reckoning)
    if not good or t_vo is None or not 0<=now-t_vo<=.5 or aligned_heading is None:return 'wait','waiting for fresh acquisition-aligned retained-frame recheck'
    from split_frame import yaw,wrap
    if math.hypot(p.vx,p.vy,p.vz)>.15:return 'wait','waiting stationary for retained-frame recheck'
    # The current body may point anywhere. Compare the actual invariant bridge
    # registration angle to the PINNED outbound value; do not demand yaw=0.
    angle=wrap(aligned_heading+yaw(q))
    if abs(wrap(angle-math.radians(self.bearing)))>math.radians(3):return 'reject','live heading/VO disagrees with retained outbound control frame'
    if self.recheck_last==t_vo:return 'wait','distinct retained-frame samples required'
    if self.recheck_last is None or not 0<t_vo-self.recheck_last<=.3:self.recheck_first=t_vo;self.recheck_count=0
    self.recheck_last=t_vo;self.recheck_count+=1
    if t_vo-self.recheck_first<2 or self.recheck_count<10:return 'wait','qualifying retained frame for2seconds'
    self.rechecked=True
   return super().observe(now,t_vo,q,p,flags,armed,landed,aligned_heading)
 return RetainedFrame

def retained_datum_class(base,seed):
 class RetainedDatum(base):
  def freeze(self,now):
   if self.offset is not None:return True
   current=self.candidate(now)
   if current is None:return False
   old=tuple(seed['datum_offset_ned_m'])
   if math.dist(current[:2],old[:2])>.15 or abs(current[2]-old[2])>.15:return False
   # Freeze the original measured translation, NEVER this new candidate.
   # Candidate is used only as fresh5second stationary consistency evidence.
   self.offset=old;return True
 return RetainedDatum

class CounterMonitor:
 def __init__(self,seed):self.seed=seed;self.attitude=self.vision=None
 def attitude_cb(self,msg):self.attitude=(msg.timestamp/1e6,int(msg.quat_reset_counter))
 def vision_cb(self,msg):self.vision=(msg.timestamp/1e6,int(msg.reset_counter),msg.timestamp_sample/1e6)
 def __call__(self,now):
  if self.attitude is None or self.vision is None:return False,'waiting for actual attitude and EV bridge reset counters'
  if not 0<=now-self.attitude[0]<=.5 or not 0<=now-self.vision[0]<=.5 or not 0<=now-self.vision[2]<=.5:return False,'stale retained-frame reset-counter observations'
  if self.attitude[1]!=self.seed['attitude_reset_counter'] or self.vision[1]!=self.seed['vision_reset_counter']:return False,'attitude or external-vision reset after outbound registration'
  return True,'fresh reset counters equal outbound'
