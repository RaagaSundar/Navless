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
 """Bounded causal queues; source callbacks can precede the /clock callback.

 Never replace an eligible observation with a future one. Every eligible
 counter event is checked, even if a later consistent sample exists. Queue
 overflow or invalid timestamps latch rejection rather than dropping evidence.
 """
 def __init__(self,seed,max_pending=512,logger=print):
  from collections import deque
  self.seed=seed;self.attitude=self.vision=None
  self.pending_attitude=deque();self.pending_vision=deque()
  self.max_pending=max_pending;self.latched=None;self.last_diagnostics={}
  self.logger=logger;self.last_log_category=None
 def _add(self,kind,publish,acquire,counter):
  if self.latched is not None:return
  if not all(math.isfinite(v) and v>0 for v in (publish,acquire)):
   self.latched=f'invalid native {kind} publication/acquisition stamp';return
  queue=self.pending_attitude if kind=='attitude' else self.pending_vision
  if len(queue)>=self.max_pending:
   self.latched=f'{kind} pending causal queue overflow ({self.max_pending}); counter evidence not discarded';return
  queue.append((publish,counter,acquire))
 def attitude_cb(self,msg):
  # Older message schemas without timestamp_sample retain their native
  # publication timestamp as the only available attitude timing evidence.
  self._add('attitude',float(msg.timestamp)/1e6,float(getattr(msg,'timestamp_sample',msg.timestamp))/1e6,int(msg.quat_reset_counter))
 def vision_cb(self,msg):
  self._add('bridge',float(msg.timestamp)/1e6,float(msg.timestamp_sample)/1e6,int(msg.reset_counter))
 def _consume(self,kind,now):
  queue=self.pending_attitude if kind=='attitude' else self.pending_vision
  future=[];eligible=[]
  while queue:
   row=queue.popleft()
   (eligible if row[0]<=now and row[2]<=now else future).append(row)
  queue.extend(future)
  expected=self.seed['attitude_reset_counter' if kind=='attitude' else 'vision_reset_counter']
  for row in sorted(eligible,key=lambda r:(r[2],r[0])):
   if row[1]!=expected:
    self.latched=(f'{kind} reset event after outbound registration: expected={expected} observed={row[1]} '
                  f'publish_s={row[0]:.9f} acquisition_s={row[2]:.9f}');return
  if eligible:
   latest=max(eligible,key=lambda r:(r[2],r[0]));old=self.attitude if kind=='attitude' else self.vision
   if old is None or (latest[2],latest[0])>(old[2],old[0]):
    if kind=='attitude':self.attitude=latest
    else:self.vision=latest
 def _reason(self,now,category,text):
  def age(row,index):return None if row is None else round(now-row[index],9)
  self.last_diagnostics={'sim_now_s':now,'attitude_publish_age_s':age(self.attitude,0),
   'attitude_acquisition_age_s':age(self.attitude,2),'bridge_publish_age_s':age(self.vision,0),
   'bridge_acquisition_age_s':age(self.vision,2),'pending_attitude':len(self.pending_attitude),
   'pending_bridge':len(self.pending_vision),
   'next_attitude_publish_s':min((r[0] for r in self.pending_attitude),default=None),
   'next_bridge_publish_s':min((r[0] for r in self.pending_vision),default=None)}
  details=' '.join(f'{key}={value}' for key,value in self.last_diagnostics.items())
  reason=text+'; '+details
  if self.logger is not None and category!=self.last_log_category:
   self.logger('retained counter monitor: '+reason);self.last_log_category=category
  return reason
 def __call__(self,now):
  if not math.isfinite(now):
   self.latched='nonfinite simulation clock'
  if self.latched is None:
   self._consume('attitude',now)
   if self.latched is None:self._consume('bridge',now)
  if self.latched is not None:return False,self._reason(now,'latched',self.latched)
  if self.attitude is None or self.vision is None:
   return False,self._reason(now,'waiting','waiting for eligible actual attitude and EV bridge reset counters')
  if any(not 0<=now-row[index]<=.5 for row in (self.attitude,self.vision) for index in (0,2)):
   return False,self._reason(now,'stale','stale retained-frame reset-counter observations')
  return True,self._reason(now,'ready','fresh eligible reset counters equal outbound')
