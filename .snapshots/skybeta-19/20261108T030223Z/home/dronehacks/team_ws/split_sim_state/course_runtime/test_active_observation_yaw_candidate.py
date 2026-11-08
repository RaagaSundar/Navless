import json,math,unittest
from pathlib import Path
from active_observation_yaw_candidate import FiniteYawObservation,measured_fixed_surface_standoff
from anticipatory_tangent_yaw_candidate import wrap

C=json.loads(Path(__file__).with_name('active-observation-config.json').read_text())
P=[*C['launch_center_enu_xy'],.95]

def inputs(t):
    health=dict(px4_valid=True,slam_valid=True,gyro_valid=True,integrity_valid=True,authority_valid=True,range_valid=True,pose_stamp_s=t,yaw_stamp_s=t,gyro_stamp_s=t,status_stamp_s=t,range_stamp_s=t,speed_m_s=0.,gyro_yaw_rate_deg_s=0.)
    quality=dict(state='qualified',stamp_s=round(math.floor((t+1e-8)/.2)*.2,6))
    sector=dict(state='supported_measured_front_sector',evidence_id='unit-only-not-real-clearance',stamp_s=t,minimum_measured_ray_distance_m=.8,heading_center_rad=0.,half_width_rad=math.radians(10))
    return health,quality,sector

class ActiveTests(unittest.TestCase):
    def test_one_step_rate_limit_and_fresh_post_settle_observations(self):
        f=FiniteYawObservation(C);measured=0.;previous=0.;complete=None
        for i in range(160):
            t=i*.05;h,q,s=inputs(t);h['gyro_yaw_rate_deg_s']=abs(math.degrees(wrap(previous-measured))/.2)
            d=f.update(t,P,measured,h,q,s);self.assertEqual(d.speed_limit_m_s,0)
            self.assertLessEqual(abs(wrap(d.yaw_rad-previous)),math.radians(.1)+1e-10)
            self.assertLessEqual(abs(d.yaw_rad),math.radians(2)+1e-10)
            measured+=wrap(d.yaw_rad-measured)*.25;previous=d.yaw_rad
            if d.action=='complete_land_requested':complete=t;break
        self.assertIsNotNone(complete);self.assertGreater(complete,2.)
        self.assertEqual(f.state,'COMPLETE');self.assertGreaterEqual(f.qualified[-1]-f.qualified[0],.4-1e-8)
    def test_unknown_pauses_yaw_rejected_requests_land(self):
        f=FiniteYawObservation(C);last=0.
        for i in range(18):
            t=i*.05;h,q,s=inputs(t);last=f.update(t,P,last,h,q,s).yaw_rad
        h,q,s=inputs(.9);q['state']='unknown';paused=f.update(.9,P,last,h,q,s)
        self.assertEqual(paused.action,'hold');self.assertEqual(paused.yaw_rad,last)
        h,q,s=inputs(.95);q['state']='rejected';self.assertEqual(f.update(.95,P,last,h,q,s).action,'auto_land_requested')
    def test_stale_health_and_authority_dominate(self):
        for key in ['px4_valid','slam_valid','gyro_valid','integrity_valid','authority_valid','range_valid']:
            f=FiniteYawObservation(C);h,q,s=inputs(0);h[key]=False
            self.assertEqual(f.update(0,P,0,h,q,s).action,'integrity_stop')
        f=FiniteYawObservation(C);h,q,s=inputs(1);h['gyro_stamp_s']=0
        self.assertEqual(f.update(1,P,0,h,q,s).action,'integrity_stop')
    def test_weak_sector_known_bad_and_moving_never_slew(self):
        for mode in ['unknownsector','near','bad','moving']:
            f=FiniteYawObservation(C)
            for i in range(40):
                t=i*.05;h,q,s=inputs(t)
                if mode=='unknownsector':s['state']='unknown'
                if mode=='near':s['minimum_measured_ray_distance_m']=.1
                if mode=='bad':h['known_bad_sector']=True
                if mode=='moving':h['speed_m_s']=.1
                d=f.update(t,P,0,h,q,s);self.assertEqual(d.yaw_rad,0);self.assertEqual(d.action,'hold')
    def test_outside_hover_cylinder_requests_land(self):
        f=FiniteYawObservation(C);h,q,s=inputs(0);p=[P[0]+.16,P[1],P[2]]
        self.assertEqual(f.update(0,p,0,h,q,s).action,'auto_land_requested')
    def test_actual_standoff_contract_never_claims_free_space(self):
        quality=dict(state='qualified',stamp_s=0.,evidence_id='unit-only',calibration=dict(K=[[381.,0,320],[0,381.,200],[0,0,1]],baseline_m=.075,body_from_left_optical=[[0,0,1,.1053],[-1,0,0,.0375],[0,-1,0,.0304],[0,0,0,1]]),landmarks_current_camera=[dict(xyz_m=[0,0,1.])])
        result=measured_fixed_surface_standoff(quality)
        self.assertEqual(result['state'],'measured_fixed_surface_standoff')
        self.assertEqual(result['scope'],'observed_fixed_surfaces_only_not_free_space')
        self.assertGreater(result['minimum_observed_body_range_lower_bound_m'],.35)
        self.assertIn('rear',result['blind_zones'])
        self.assertEqual(measured_fixed_surface_standoff(dict(quality,state='unknown'))['state'],'unknown')
        self.assertEqual(measured_fixed_surface_standoff(quality,-1)['state'],'unknown')

    def test_missing_yaw_and_unknown_stamp_fail_closed(self):
        f=FiniteYawObservation(C);h,q,s=inputs(0)
        self.assertEqual(f.update(0,P,None,h,q,s).action,'integrity_stop')
        f=FiniteYawObservation(C);q['stamp_s']=None
        self.assertEqual(f.update(0,P,0,h,q,s).action,'hold')

    def test_invalid_large_step_rejected(self):
        with self.assertRaises(ValueError):FiniteYawObservation(dict(C,increment_deg=10))
        with self.assertRaises(ValueError):FiniteYawObservation(dict(C,hover_radius_m=.5))

if __name__=='__main__':unittest.main()
