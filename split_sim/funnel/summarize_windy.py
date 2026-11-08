"""Aggregate a batch from its run ledger and independent, run-local results."""
import json
import statistics
import sys
from pathlib import Path


def summary(batch):
    runs = batch.parents[1]
    outcomes = []
    for line in (batch/'run-ids.txt').read_text().splitlines():
        index, rid = line.split()
        path = runs/rid/'flight/result.json'
        if path.exists():
            r=json.loads(path.read_text())
            outcomes.append({k:r[k] for k in ('index','run_id','seed','configured_offset_m','measured_spawn_xy_m',
                'gust_period_s','wind_mean_y_m_s','wind_min_m_s','wind_max_m_s','windy_link_verified','airborne_samples',
                'crossing_verified','lateral_error_m','max_centerline_error_last_meter_m','landing_distance_beyond_plane_m',
                'contacts_verified','wall_contacts','final_landed_disarmed','mission_exit')})
            outcomes[-1]['operator_stopped'] = bool(r.get('operator_stopped'))
    passed = [r for r in outcomes if r['crossing_verified']]
    errors = [abs(r['lateral_error_m']) for r in passed]
    landed = [r['landing_distance_beyond_plane_m'] for r in passed]
    speed_min=[r['wind_min_m_s'] for r in outcomes if r['wind_min_m_s'] is not None]
    speed_max=[r['wind_max_m_s'] for r in outcomes if r['wind_max_m_s'] is not None]
    return dict(finished=len(outcomes),airborne=sum(bool(r['airborne_samples']) for r in outcomes),
                verified_crossings=len(passed),planned_landings=sum(r['mission_exit']==0 and not r['operator_stopped'] for r in passed),final_landed_disarmed=sum(r['final_landed_disarmed'] for r in outcomes),
                contact_events=sum(r['wall_contacts'] for r in outcomes),
                verified_windy_runs=sum(r['windy_link_verified'] for r in outcomes),
                median_absolute_crossing_error_m=statistics.median(errors) if errors else None,
                worst_absolute_crossing_error_m=max(errors,default=None),
                verified_landing_range_m=[min(landed),max(landed)] if landed else None,
                sampled_airborne_wind_range_m_s=[min(speed_min),max(speed_max)] if speed_min else None,
                runs=outcomes)


if __name__=='__main__':
    batch=Path(sys.argv[1])
    data=summary(batch)
    (batch/'summary.json').write_text(json.dumps(data,indent=2))
    print({k:v for k,v in data.items() if k!='runs'})
